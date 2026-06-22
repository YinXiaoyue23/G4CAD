#include "G4StepSolid.hh"
#include "G4SystemOfUnits.hh"
#include "G4VoxelLimits.hh"
#include "G4AffineTransform.hh"
#include "G4Polyhedron.hh"
#include "G4PolyhedronArbitrary.hh"
#include "G4BoundingEnvelope.hh"
#include "G4VGraphicsScene.hh"
#include "G4AutoLock.hh"
#include "Randomize.hh"
#include "G4RunManager.hh"
#include "G4Run.hh"
#include "G4EventManager.hh"
#include "G4TrackingManager.hh"
#include "G4Track.hh"

// OCCT Headers
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <vector>

namespace {
    constexpr G4double kMeshDeflection  = 1.0;
    constexpr G4double kProbeMultiplier = 10.0;
    constexpr int      kMaxSurfRetry    = 20;

    G4Mutex polyhedronMutex = G4MUTEX_INITIALIZER;

    struct HitGroup {
        G4double dist;
        int      nIn  = 0;
        int      nOut = 0;
        int      firstOutIdx = -1;
        int      anyIdx      = -1;  // any hit index in this group (for face→solid lookup)
    };

    std::vector<HitGroup> GroupHits(const IntCurvesFace_ShapeIntersector* isect,
                                    G4double tol)
    {
        int n = isect->NbPnt();
        if (n == 0) return {};

        std::vector<std::pair<G4double,int>> raw;
        raw.reserve(n);
        for (int i = 1; i <= n; ++i)
            raw.push_back({isect->WParameter(i), i});
        std::sort(raw.begin(), raw.end(),
                  [](const auto& a, const auto& b){ return a.first < b.first; });

        std::vector<HitGroup> groups;
        for (auto& [d, idx] : raw) {
            auto trans = isect->Transition(idx);
            // Tangent intersections do not cross the boundary; skip them.
            // Counting a tangent as In or Out would corrupt the net crossing count.
            if (trans == IntCurveSurface_Tangent) continue;
            if (!groups.empty() && d - groups.back().dist < tol) {
                if (trans == IntCurveSurface_In) groups.back().nIn++;
                else {
                    groups.back().nOut++;
                    if (groups.back().firstOutIdx < 0) groups.back().firstOutIdx = idx;
                }
            } else {
                HitGroup g;
                g.dist = d;
                g.anyIdx = idx;
                if (trans == IntCurveSurface_In) { g.nIn = 1; }
                else { g.nOut = 1; g.firstOutIdx = idx; }
                groups.push_back(g);
            }
        }
        return groups;
    }

    inline gp_Pnt       G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Dir& v)       { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Pnt& v)       { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
} // namespace

// ====================================================================
// Static member definitions
// ====================================================================
std::vector<G4StepSolid::AlgoCache*>    G4StepSolid::sAllCaches;
std::mutex                               G4StepSolid::sAllCachesMutex;

std::vector<const G4StepSolid*>         G4StepSolid::sTimedSolids;
std::mutex                               G4StepSolid::sTimedSolidsMutex;
std::once_flag                           G4StepSolid::sAtexitFlag;

// Query recorder (debug-only)
bool           G4StepSolid::sRecordEnabled   = false;
int            G4StepSolid::sRecordOnlyEvent = -1;
std::ofstream  G4StepSolid::sRecordStream;
std::mutex     G4StepSolid::sRecordMutex;
std::once_flag G4StepSolid::sRecordInitFlag;
long           G4StepSolid::sRecordSeq       = 0;

namespace {
    // Max BRep tolerance over a sub-shape's faces/edges/vertices = its geometric noise σ.
    G4double ShapeNoise(const TopoDS_Shape& s) {
        G4double m = 0.0;
        for (TopExp_Explorer e(s, TopAbs_FACE);   e.More(); e.Next())
            m = std::max(m, BRep_Tool::Tolerance(TopoDS::Face(e.Current())));
        for (TopExp_Explorer e(s, TopAbs_EDGE);   e.More(); e.Next())
            m = std::max(m, BRep_Tool::Tolerance(TopoDS::Edge(e.Current())));
        for (TopExp_Explorer e(s, TopAbs_VERTEX); e.More(); e.Next())
            m = std::max(m, BRep_Tool::Tolerance(TopoDS::Vertex(e.Current())));
        return m;
    }
} // namespace

int G4StepSolid::AlgoCache::FaceToSolid(const TopoDS_Face& f) const {
    if (faceSolidMap.IsBound(f)) return faceSolidMap.Find(f);
    return -1;
}

// ====================================================================
// Query recorder (debug-only)
// ====================================================================
namespace {
    const char* EInsideName(int e) {
        return (e == kInside) ? "kInside"
             : (e == kOutside) ? "kOutside"
             : (e == kSurface) ? "kSurface" : "";
    }
}

void G4StepSolid::InitRecorderOnce() {
    const char* path = std::getenv("G4CAD_RECORD");
    if (!path || !*path) { sRecordEnabled = false; return; }
    sRecordStream.open(path, std::ios::out | std::ios::trunc);
    if (!sRecordStream.is_open()) {
        G4cerr << "[G4StepSolid] G4CAD_RECORD set but cannot open " << path << G4endl;
        sRecordEnabled = false;
        return;
    }
    const char* evEnv = std::getenv("G4CAD_RECORD_EVENT");
    sRecordOnlyEvent = (evEnv && *evEnv) ? std::atoi(evEnv) : -1;
    sRecordStream << "seq,event,trackid,parentid,type,px,py,pz,vx,vy,vz,"
                  << "result,dist,inside_at_p,solid\n";
    sRecordEnabled = true;
    G4cout << "[G4StepSolid] query recorder ON -> " << path
           << "  (event tag = G4Run::GetRunID())";
    if (sRecordOnlyEvent >= 0) G4cout << " [event " << sRecordOnlyEvent << " only]";
    G4cout << G4endl;
}

void G4StepSolid::RecordQuery(QueryType type, const G4ThreeVector& p,
                              const G4ThreeVector& v, int insideResult,
                              G4double dist, int insideAtP) const {
    if (!sRecordEnabled) return;

    int evid = -1;
    if (const G4Run* run = G4RunManager::GetRunManager()->GetCurrentRun())
        evid = run->GetRunID();
    if (sRecordOnlyEvent >= 0 && evid != sRecordOnlyEvent) return;

    // Track identity, so FreeCAD can group queries into per-track polylines.
    int tid = -1, pid = -1;
    if (G4TrackingManager* tm = G4EventManager::GetEventManager()->GetTrackingManager())
        if (const G4Track* trk = tm->GetTrack()) {
            tid = trk->GetTrackID();
            pid = trk->GetParentID();
        }

    const char* tname = (type == QueryType::Inside)   ? "INSIDE"
                      : (type == QueryType::DistToIn)  ? "DTI" : "DTO";
    const char* res   = (type == QueryType::Inside) ? EInsideName(insideResult) : "";
    const char* inAtP = (type == QueryType::DistToOut) ? EInsideName(insideAtP) : "";

    std::lock_guard<std::mutex> lock(sRecordMutex);
    sRecordStream << sRecordSeq++ << ',' << evid << ',' << tid << ',' << pid << ','
                  << tname << ','
                  << p.x() << ',' << p.y() << ',' << p.z() << ','
                  << v.x() << ',' << v.y() << ',' << v.z() << ','
                  << res << ',';
    if (type == QueryType::Inside) sRecordStream << ',';      // dist empty for INSIDE
    else                           sRecordStream << dist << ',';
    sRecordStream << inAtP << ',' << GetName() << '\n';
    if ((sRecordSeq & 0x7F) == 0) sRecordStream.flush();      // survive stuck-kill
}

// ====================================================================
// AlgoCache
// ====================================================================
G4StepSolid::AlgoCache::AlgoCache(const TopoDS_Shape& shape, G4double tolerance,
                                   const G4StepSolid* ownerSolid)
    : owner(ownerSolid)
{
    requestedTol = (tolerance > 0.0) ? tolerance : 1e-7;

    // Per-solid toggle. Default OFF (global single tolerance) — empirically the dev
    // hybrid (BRepClass3d Inside + ray DistanceToOut) is already stable at the small
    // global tolerance, and widening a noisy solid's band only desyncs DistanceToOut
    // from BRepClass3d (creates zero-step contradictions). Opt in with G4CAD_PERSOLID_TOL=1.
    const char* psEnv = std::getenv("G4CAD_PERSOLID_TOL");
    perSolidTol = (psEnv && psEnv[0] == '1');

    // Build per-solid classifiers, per-solid bands, and face→solid map in one pass.
    G4double globalMax = requestedTol;
    int si = 0;
    for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next(), ++si) {
        const TopoDS_Shape& s = ex.Current();
        solidClassifiers.push_back(std::make_unique<BRepClass3d_SolidClassifier>(s));
        G4double sigma = ShapeNoise(s);
        G4double band  = std::max(requestedTol, 2.0 * sigma);  // self-consistency: band >= 2σ
        solidTol.push_back(band);
        globalMax = std::max(globalMax, band);
        for (TopExp_Explorer fe(s, TopAbs_FACE); fe.More(); fe.Next())
            if (!faceSolidMap.IsBound(fe.Current()))
                faceSolidMap.Bind(fe.Current(), si);
    }
    // Global mode: OCCT search/grouping uses plain requested tol (pure "current dev").
    occtTol = perSolidTol ? globalMax : requestedTol;
    if (solidTol.empty()) { solidTol.push_back(requestedTol); occtTol = requestedTol; }

    // One-time tolerance table (env G4CAD_TOL_VERBOSE), to confirm which solids got widened.
    static std::once_flag tolPrintFlag;
    if (std::getenv("G4CAD_TOL_VERBOSE"))
        std::call_once(tolPrintFlag, [&]{
            G4cout << "[G4StepSolid] per-solid tolerance "
                   << (perSolidTol ? "ON" : "OFF(global)")
                   << "  requested=" << requestedTol << " occtTol=" << occtTol << " mm\n";
            for (size_t i = 0; i < solidTol.size(); ++i)
                G4cout << "    solid#" << i << "  band=" << solidTol[i] << " mm\n";
            G4cout << G4endl;
        });

    intersector = new IntCurvesFace_ShapeIntersector();
    intersector->Load(shape, occtTol);
}

G4StepSolid::AlgoCache::~AlgoCache() {
    delete intersector;
}

// ====================================================================
// Construction and destruction
// ====================================================================
G4StepSolid::G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape,
                          G4double tolerance)
    : G4VSolid(name), fShape(stepShape), fTolerance(tolerance)
{
    // Resolve the debug query recorder once (reads G4CAD_RECORD env); zero cost when off.
    // Per-solid tolerances are computed lazily per thread in AlgoCache (keeps ABI stable).
    std::call_once(sRecordInitFlag, &G4StepSolid::InitRecorderOnce);

    CalcBBox();
    BuildFaceWeights();
}

G4StepSolid::~G4StepSolid() {
    delete fpPolyhedron;
    std::lock_guard<std::mutex> lock(sAllCachesMutex);
    for (AlgoCache* c : sAllCaches) delete c;
    sAllCaches.clear();
}

G4StepSolid::G4StepSolid(const G4StepSolid& rhs)
    : G4VSolid(rhs), fVerboseLevel(rhs.fVerboseLevel),
      fTimingEnabled(rhs.fTimingEnabled),
      fShape(rhs.fShape),
      fXmin(rhs.fXmin), fXmax(rhs.fXmax),
      fYmin(rhs.fYmin), fYmax(rhs.fYmax),
      fZmin(rhs.fZmin), fZmax(rhs.fZmax),
      fTolerance(rhs.fTolerance),
      fFaces(rhs.fFaces), fCumulativeArea(rhs.fCumulativeArea)
{
    fpPolyhedron      = nullptr;
    fRebuildPolyhedron = true;
}

G4StepSolid& G4StepSolid::operator=(const G4StepSolid& rhs) {
    if (this == &rhs) return *this;
    G4VSolid::operator=(rhs);
    fVerboseLevel   = rhs.fVerboseLevel;
    fTimingEnabled  = rhs.fTimingEnabled;
    fShape          = rhs.fShape;
    fXmin = rhs.fXmin; fXmax = rhs.fXmax;
    fYmin = rhs.fYmin; fYmax = rhs.fYmax;
    fZmin = rhs.fZmin; fZmax = rhs.fZmax;
    fTolerance      = rhs.fTolerance;
    fFaces          = rhs.fFaces;
    fCumulativeArea = rhs.fCumulativeArea;

    delete fpPolyhedron;
    fpPolyhedron       = nullptr;
    fRebuildPolyhedron = true;

    AlgoCache* algo = fAlgoCache.Get();
    if (algo) {
        std::lock_guard<std::mutex> lock(sAllCachesMutex);
        auto it = std::find(sAllCaches.begin(), sAllCaches.end(), algo);
        if (it != sAllCaches.end()) sAllCaches.erase(it);
        delete algo;
        fAlgoCache.Put(nullptr);
    }
    return *this;
}

// ====================================================================
// Private helpers
// ====================================================================
void G4StepSolid::CalcBBox() {
    Bnd_Box box;
    BRepBndLib::Add(fShape, box);
    box.Get(fXmin, fYmin, fZmin, fXmax, fYmax, fZmax);
}

void G4StepSolid::BuildFaceWeights() {
    fFaces.clear();
    fCumulativeArea.clear();
    G4double cumArea = 0.0;

    for (TopExp_Explorer ex(fShape, TopAbs_FACE); ex.More(); ex.Next()) {
        TopoDS_Face face = TopoDS::Face(ex.Current());

        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        G4double area = props.Mass();
        if (area <= 0.0) continue;

        BRepAdaptor_Surface surf(face);
        FaceEntry fe;
        fe.face = face;
        fe.u0   = surf.FirstUParameter();
        fe.u1   = surf.LastUParameter();
        fe.v0   = surf.FirstVParameter();
        fe.v1   = surf.LastVParameter();
        fFaces.push_back(fe);

        cumArea += area;
        fCumulativeArea.push_back(cumArea);
    }
}

G4StepSolid::AlgoCache* G4StepSolid::GetAlgo() const {
    AlgoCache* algo = fAlgoCache.Get();
    if (!algo) {
        algo = new AlgoCache(fShape, fTolerance, this);
        fAlgoCache.Put(algo);
        std::lock_guard<std::mutex> lock(sAllCachesMutex);
        sAllCaches.push_back(algo);
    }
    return algo;
}

G4ThreeVector G4StepSolid::GetNormalAtIntersection(int index, const AlgoCache* algo) const {
    const TopoDS_Face& face = algo->intersector->Face(index);
    double u = algo->intersector->UParameter(index);
    double v = algo->intersector->VParameter(index);
    BRepAdaptor_Surface surf(face);
    gp_Pnt pSurf; gp_Vec d1u, d1v;
    surf.D1(u, v, pSurf, d1u, d1v);
    gp_Vec norm = d1u.Crossed(d1v);
    if (face.Orientation() == TopAbs_REVERSED) norm.Reverse();
    return OCCTtoG4(gp_Dir(norm));
}

// ====================================================================
// RayCastToBoundary
// ====================================================================
G4double G4StepSolid::RayCastToBoundary(const G4ThreeVector& p, const G4ThreeVector& v,
                                         BoundaryKind kind,
                                         G4bool calcNorm, G4bool* validNorm,
                                         G4ThreeVector* n) const
{
    if (validNorm) *validNorm = false;

    AlgoCache* algo = GetAlgo();
    const G4double octTol = algo->occtTol;
    if (v.mag2() < octTol * octTol) return kInfinity;

    gp_Lin ray(G4toOCCT(p), gp_Dir(v.x(), v.y(), v.z()));
    algo->intersector->Perform(ray, -octTol, kInfinity);

    if (!algo->intersector->IsDone() || algo->intersector->NbPnt() == 0)
        return (kind == BoundaryKind::Entry) ? kInfinity : 0.0;

    const bool wantExit = (kind == BoundaryKind::Exit);
    const char* fnName  = wantExit ? "DistanceToOut" : "DistanceToIn";

    // Surface band for the solid owning a hit group (per-solid tolerance).
    auto bandOf = [&](const HitGroup& g) -> G4double {
        int idx = (g.firstOutIdx >= 0) ? g.firstOutIdx : g.anyIdx;
        int si  = (idx >= 0) ? algo->FaceToSolid(algo->intersector->Face(idx)) : -1;
        return algo->SolidBand(si);
    };

    auto isBoundaryGroup = [&](const HitGroup& g) -> bool {
        int net = wantExit ? (g.nOut - g.nIn) : (g.nIn - g.nOut);
        if (net > 0) return true;
        if (net < 0) return false;
        G4ThreeVector probe = p + (g.dist + octTol * kProbeMultiplier) * v;
        bool probeInside = false, probeOn = false;
        for (size_t i = 0; i < algo->solidClassifiers.size(); ++i) {
            algo->solidClassifiers[i]->Perform(G4toOCCT(probe), algo->SolidBand((int)i));
            auto s = algo->solidClassifiers[i]->State();
            if (s == TopAbs_IN) { probeInside = true; break; }
            if (s == TopAbs_ON) probeOn = true;
        }
        bool probeOutside = !probeInside && !probeOn;
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "] coincident-face probe"
                   << " dist=" << g.dist
                   << " -> " << (probeOutside ? "real boundary" : "internal pass-through")
                   << G4endl;
        }
        return wantExit ? probeOutside : !probeOutside;
    };

    for (const auto& g : GroupHits(algo->intersector, octTol)) {
        if (!isBoundaryGroup(g)) continue;

        const G4double band = bandOf(g);  // per-solid surface band
        if (std::abs(g.dist) < band) {
            if (calcNorm && validNorm && g.firstOutIdx >= 0) {
                *n = GetNormalAtIntersection(g.firstOutIdx, algo);
                *validNorm = true;
            }
            if (fVerboseLevel >= 2) {
                G4cout << "[G4StepSolid::" << GetName() << "::" << fnName << "][local]"
                       << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                       << " v=(" << v.x() << "," << v.y() << "," << v.z() << ")"
                       << " -> 0.0 (on boundary, band=" << band << ")" << G4endl;
            }
            return 0.0;
        }
        if (g.dist > band) {
            if (calcNorm && validNorm && g.firstOutIdx >= 0) {
                *n = GetNormalAtIntersection(g.firstOutIdx, algo);
                *validNorm = true;
            }
            if (fVerboseLevel >= 2) {
                G4cout << "[G4StepSolid::" << GetName() << "::" << fnName << "][local]"
                       << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                       << " v=(" << v.x() << "," << v.y() << "," << v.z() << ")"
                       << " -> " << g.dist << G4endl;
            }
            return g.dist;
        }
    }

    if (fVerboseLevel >= 2) {
        G4cout << "[G4StepSolid::" << GetName() << "::" << fnName << "][local]"
               << " no real boundary -> "
               << (kind == BoundaryKind::Entry ? "kInfinity" : "0.0") << G4endl;
    }
    return (kind == BoundaryKind::Entry) ? kInfinity : 0.0;
}

// ====================================================================
// Inside
// ====================================================================
EInside G4StepSolid::Inside(const G4ThreeVector& p) const {
    AlgoCache* algo = nullptr;
    Clock::time_point t0;
    if (fTimingEnabled) { algo = GetAlgo(); t0 = Clock::now(); }

    if (p.x() < fXmin || p.x() > fXmax ||
        p.y() < fYmin || p.y() > fYmax ||
        p.z() < fZmin || p.z() > fZmax)
    {
        if (fTimingEnabled) {
            algo->timing.insideNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
            ++algo->timing.insideCount;
        }
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::Inside][local]"
                   << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                   << " -> kOutside (bbox)" << G4endl;
        }
        return kOutside;
    }

    if (!algo) algo = GetAlgo();
    EInside result = kOutside;
    for (size_t i = 0; i < algo->solidClassifiers.size(); ++i) {
        algo->solidClassifiers[i]->Perform(G4toOCCT(p), algo->SolidBand((int)i));
        auto s = algo->solidClassifiers[i]->State();
        if (s == TopAbs_IN) { result = kInside; break; }
        if (s == TopAbs_ON) result = kSurface;
    }

    if (fTimingEnabled) {
        algo->timing.insideNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
        ++algo->timing.insideCount;
    }
    if (fVerboseLevel >= 2) {
        G4cout << "[G4StepSolid::" << GetName() << "::Inside][local]"
               << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
               << " -> " << (result==kInside?"kInside":result==kOutside?"kOutside":"kSurface")
               << G4endl;
    }
    if (sRecordEnabled)
        RecordQuery(QueryType::Inside, p, G4ThreeVector(0,0,0), result, 0.0, -1);
    return result;
}

// ====================================================================
// DistanceToIn
// ====================================================================
G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const {
    auto t0 = fTimingEnabled ? Clock::now() : Clock::time_point{};
    G4double result = RayCastToBoundary(p, v, BoundaryKind::Entry);
    if (fTimingEnabled) {
        AlgoCache* algo = GetAlgo();
        algo->timing.dtiVecNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
        ++algo->timing.dtiVecCount;
    }
    if (sRecordEnabled)
        RecordQuery(QueryType::DistToIn, p, v, 0, result, -1);
    return result;
}

G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p) const {
    AlgoCache* algo = fTimingEnabled ? GetAlgo() : nullptr;
    auto t0 = fTimingEnabled ? Clock::now() : Clock::time_point{};

    G4double dx = std::max({fXmin - p.x(), p.x() - fXmax, 0.0});
    G4double dy = std::max({fYmin - p.y(), p.y() - fYmax, 0.0});
    G4double dz = std::max({fZmin - p.z(), p.z() - fZmax, 0.0});
    G4double bboxDist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (bboxDist > 0.0) {
        if (fTimingEnabled) {
            algo->timing.dtiScalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
            ++algo->timing.dtiScalCount;
        }
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToIn(p)][local]"
                   << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                   << " -> " << bboxDist << " (bbox)" << G4endl;
        }
        return bboxDist;
    }

    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    G4double result = extrema.IsDone() ? extrema.Value() : 0.0;
    if (fTimingEnabled) {
        algo->timing.dtiScalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
        ++algo->timing.dtiScalCount;
    }
    if (fVerboseLevel >= 2) {
        G4cout << "[G4StepSolid::" << GetName() << "::DistanceToIn(p)][local]"
               << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
               << " -> " << result << G4endl;
    }
    return result;
}

// ====================================================================
// DistanceToOut
// ====================================================================
G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                                    const G4bool calcNorm, G4bool* validNorm,
                                    G4ThreeVector* n) const {
    auto t0 = fTimingEnabled ? Clock::now() : Clock::time_point{};
    G4double result = RayCastToBoundary(p, v, BoundaryKind::Exit, calcNorm, validNorm, n);
    if (fTimingEnabled) {
        AlgoCache* algo = GetAlgo();
        algo->timing.dtoVecNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
        ++algo->timing.dtoVecCount;
    }
    if (sRecordEnabled) {
        // Rigorous stuck signature: Inside==kInside AND DistanceToOut≈0.
        int insideAtP = Inside(p);
        RecordQuery(QueryType::DistToOut, p, v, 0, result, insideAtP);
    }
    return result;
}

G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p) const {
    AlgoCache* algo = fTimingEnabled ? GetAlgo() : nullptr;
    auto t0 = fTimingEnabled ? Clock::now() : Clock::time_point{};

    if (Inside(p) == kOutside) {
        if (fTimingEnabled) {
            algo->timing.dtoScalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
            ++algo->timing.dtoScalCount;
        }
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToOut(p)][local]"
                   << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                   << " -> 0.0 (outside)" << G4endl;
        }
        return 0.0;
    }

    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    G4double result = extrema.IsDone() ? extrema.Value() : 0.0;
    if (fTimingEnabled) {
        algo->timing.dtoScalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
        ++algo->timing.dtoScalCount;
    }
    if (fVerboseLevel >= 2) {
        G4cout << "[G4StepSolid::" << GetName() << "::DistanceToOut(p)][local]"
               << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
               << " -> " << result << G4endl;
    }
    return result;
}

// ====================================================================
// SurfaceNormal
// ====================================================================
G4ThreeVector G4StepSolid::SurfaceNormal(const G4ThreeVector& p) const {
    AlgoCache* algo = fTimingEnabled ? GetAlgo() : nullptr;
    auto t0 = fTimingEnabled ? Clock::now() : Clock::time_point{};

    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    if (!extrema.IsDone() || extrema.NbSolution() == 0) {
        if (fTimingEnabled) {
            algo->timing.normalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
            ++algo->timing.normalCount;
        }
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::SurfaceNormal][local]"
                   << " no solution -> (0,0,1)" << G4endl;
        }
        return G4ThreeVector(0, 0, 1);
    }

    G4ThreeVector result;
    TopoDS_Shape support = extrema.SupportOnShape2(1);
    if (support.ShapeType() == TopAbs_FACE) {
        TopoDS_Face face = TopoDS::Face(support);
        BRepAdaptor_Surface surf(face);
        gp_Pnt pLoc = extrema.PointOnShape2(1);
        GeomAPI_ProjectPointOnSurf proj(pLoc, surf.Surface().Surface());
        Standard_Real u, v;
        proj.LowerDistanceParameters(u, v);
        gp_Pnt pSurf; gp_Vec d1u, d1v;
        surf.D1(u, v, pSurf, d1u, d1v);
        gp_Vec norm = d1u.Crossed(d1v);
        if (face.Orientation() == TopAbs_REVERSED) norm.Reverse();
        result = OCCTtoG4(gp_Dir(norm));
    } else {
        gp_Pnt pLoc = extrema.PointOnShape2(1);
        G4ThreeVector dir = p - OCCTtoG4(pLoc);
        result = (dir.mag2() > 0) ? dir.unit() : G4ThreeVector(0, 0, 1);
    }

    if (fTimingEnabled) {
        algo->timing.normalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
        ++algo->timing.normalCount;
    }
    if (fVerboseLevel >= 2) {
        G4cout << "[G4StepSolid::" << GetName() << "::SurfaceNormal][local]"
               << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
               << " -> (" << result.x() << "," << result.y() << "," << result.z() << ")"
               << G4endl;
    }
    return result;
}

// ====================================================================
// GetPointOnSurface
// ====================================================================
G4ThreeVector G4StepSolid::GetPointOnSurface() const {
    if (fFaces.empty()) return G4ThreeVector(0, 0, 0);

    const G4double totalArea = fCumulativeArea.back();

    for (int attempt = 0; attempt < kMaxSurfRetry; ++attempt) {
        G4double r   = G4UniformRand() * totalArea;
        auto     it  = std::lower_bound(fCumulativeArea.begin(), fCumulativeArea.end(), r);
        int      idx = static_cast<int>(it - fCumulativeArea.begin());
        if (idx >= static_cast<int>(fFaces.size())) idx = static_cast<int>(fFaces.size()) - 1;

        const FaceEntry& fe = fFaces[idx];
        G4double u = fe.u0 + G4UniformRand() * (fe.u1 - fe.u0);
        G4double v = fe.v0 + G4UniformRand() * (fe.v1 - fe.v0);

        BRepAdaptor_Surface surf(fe.face);
        gp_Pnt pt = surf.Value(u, v);
        G4ThreeVector candidate(pt.X(), pt.Y(), pt.Z());

        if (Inside(candidate) != kOutside) return candidate;
    }

    const FaceEntry& fe = fFaces[0];
    BRepAdaptor_Surface surf(fe.face);
    gp_Pnt pt = surf.Value(0.5*(fe.u0+fe.u1), 0.5*(fe.v0+fe.v1));
    return G4ThreeVector(pt.X(), pt.Y(), pt.Z());
}

// ====================================================================
// BoundingLimits / CalculateExtent
// ====================================================================
void G4StepSolid::BoundingLimits(G4ThreeVector& pMin, G4ThreeVector& pMax) const {
    pMin.set(fXmin, fYmin, fZmin);
    pMax.set(fXmax, fYmax, fZmax);
    if (fVerboseLevel >= 2) {
        G4cout << "[G4StepSolid::" << GetName() << "::BoundingLimits]"
               << " pMin=(" << fXmin << "," << fYmin << "," << fZmin << ")"
               << " pMax=(" << fXmax << "," << fYmax << "," << fZmax << ")" << G4endl;
    }
}

G4bool G4StepSolid::CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit,
                                    const G4AffineTransform& pTransform,
                                    G4double& pMin, G4double& pMax) const {
    G4ThreeVector bmin(fXmin, fYmin, fZmin), bmax(fXmax, fYmax, fZmax);
    G4BoundingEnvelope bbox(bmin, bmax);
    G4bool result = bbox.CalculateExtent(pAxis, pVoxelLimit, pTransform, pMin, pMax);
    if (fVerboseLevel >= 2) {
        G4cout << "[G4StepSolid::" << GetName() << "::CalculateExtent]"
               << " axis=" << pAxis
               << " -> [" << pMin << "," << pMax << "]  ok=" << result << G4endl;
    }
    return result;
}

// ====================================================================
// Visualisation
// ====================================================================
void G4StepSolid::DescribeYourselfTo(G4VGraphicsScene& scene) const {
    scene.AddSolid(*this);
}

G4Polyhedron* G4StepSolid::GetPolyhedron() const {
    G4AutoLock l(&polyhedronMutex);
    if (!fpPolyhedron || fRebuildPolyhedron ||
        fpPolyhedron->GetNumberOfRotationStepsAtTimeOfCreation() !=
        fpPolyhedron->GetNumberOfRotationSteps())
    {
        delete fpPolyhedron;
        fpPolyhedron       = CreatePolyhedron();
        fRebuildPolyhedron = false;
    }
    return fpPolyhedron;
}

G4Polyhedron* G4StepSolid::CreatePolyhedron() const {
    if (fVerboseLevel >= 1) {
        G4cout << "G4StepSolid: generating mesh for " << GetName() << " ..." << G4endl;
    }

    try {
        BRepMesh_IncrementalMesh meshGen(fShape, kMeshDeflection);
    } catch (const std::exception& e) {
        G4cerr << "G4StepSolid::CreatePolyhedron: mesh failed for "
               << GetName() << ": " << e.what() << G4endl;
    } catch (...) {
        G4cerr << "G4StepSolid::CreatePolyhedron: mesh failed for "
               << GetName() << " (unknown exception)" << G4endl;
    }

    G4int nVertices = 0, nFacets = 0;
    for (TopExp_Explorer ex(fShape, TopAbs_FACE); ex.More(); ex.Next()) {
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri =
            BRep_Tool::Triangulation(TopoDS::Face(ex.Current()), loc);
        if (!tri.IsNull()) {
            nVertices += tri->NbNodes();
            nFacets   += tri->NbTriangles();
        }
    }

    if (nFacets == 0) {
        G4cerr << "G4StepSolid::CreatePolyhedron: no triangles for "
               << GetName() << ", using fallback box" << G4endl;
        return new G4PolyhedronBox(10, 10, 10);
    }

    G4PolyhedronArbitrary* poly = new G4PolyhedronArbitrary(nVertices, nFacets);
    G4int vertexOffset = 0;
    for (TopExp_Explorer ex(fShape, TopAbs_FACE); ex.More(); ex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;
        gp_Trsf trsf = loc.Transformation();
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt pt = tri->Node(i);
            pt.Transform(trsf);
            poly->AddVertex(G4ThreeVector(pt.X(), pt.Y(), pt.Z()));
        }
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);
            int i1 = vertexOffset + n1;
            int i2 = vertexOffset + n2;
            int i3 = vertexOffset + n3;
            if (face.Orientation() == TopAbs_REVERSED)
                poly->AddFacet(i1, i3, i2);
            else
                poly->AddFacet(i1, i2, i3);
        }
        vertexOffset += tri->NbNodes();
    }
    return poly;
}

// ====================================================================
// Timing
// ====================================================================
void G4StepSolid::SetTimingEnabled(G4bool enable) {
    fTimingEnabled = enable;
    if (!enable) return;
    {
        std::lock_guard<std::mutex> lock(sTimedSolidsMutex);
        if (std::find(sTimedSolids.begin(), sTimedSolids.end(), this) == sTimedSolids.end())
            sTimedSolids.push_back(this);
    }
    // Register atexit handler once so the report is always printed at exit.
    std::call_once(sAtexitFlag, []{ std::atexit(G4StepSolid::PrintAllTimingReports); });
}

namespace {
    void PrintTimingRow(const char* name, int64_t ns, int64_t count) {
        if (count == 0) return;
        G4cout << "  " << std::left << std::setw(24) << name
               << std::right << std::setw(10) << count << " calls"
               << std::setw(12) << (ns / count) << " ns/call"
               << std::setw(12) << (ns / 1000) << " µs total"
               << G4endl;
    }
}

void G4StepSolid::PrintTimingReport() const {
    TimingStats total;
    {
        std::lock_guard<std::mutex> lock(sAllCachesMutex);
        for (const AlgoCache* c : sAllCaches) {
            if (c->owner != this) continue;
            total.insideNs    += c->timing.insideNs;
            total.insideCount += c->timing.insideCount;
            total.dtiVecNs    += c->timing.dtiVecNs;
            total.dtiVecCount += c->timing.dtiVecCount;
            total.dtiScalNs   += c->timing.dtiScalNs;
            total.dtiScalCount+= c->timing.dtiScalCount;
            total.dtoVecNs    += c->timing.dtoVecNs;
            total.dtoVecCount += c->timing.dtoVecCount;
            total.dtoScalNs   += c->timing.dtoScalNs;
            total.dtoScalCount+= c->timing.dtoScalCount;
            total.normalNs    += c->timing.normalNs;
            total.normalCount += c->timing.normalCount;
        }
    }
    G4cout << "[G4StepSolid timing] " << GetName() << G4endl;
    PrintTimingRow("Inside",              total.insideNs,   total.insideCount);
    PrintTimingRow("DistanceToIn(p,v)",   total.dtiVecNs,   total.dtiVecCount);
    PrintTimingRow("DistanceToIn(p)",     total.dtiScalNs,  total.dtiScalCount);
    PrintTimingRow("DistanceToOut(p,v)",  total.dtoVecNs,   total.dtoVecCount);
    PrintTimingRow("DistanceToOut(p)",    total.dtoScalNs,  total.dtoScalCount);
    PrintTimingRow("SurfaceNormal",       total.normalNs,   total.normalCount);
}

void G4StepSolid::PrintAllTimingReports() {
    std::lock_guard<std::mutex> lock(sTimedSolidsMutex);
    for (const G4StepSolid* s : sTimedSolids)
        s->PrintTimingReport();
}

std::ostream& G4StepSolid::StreamInfo(std::ostream& os) const {
    os << "G4StepSolid: " << GetName()
       << "  bbox=[" << fXmin << "," << fXmax << "] x ["
       << fYmin << "," << fYmax << "] x ["
       << fZmin << "," << fZmax << "]"
       << "  tolerance=" << fTolerance
       << "  faces=" << fFaces.size();
    return os;
}
