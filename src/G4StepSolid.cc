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
#include <IntCurvesFace_Intersector.hxx>
#include <TopoDS_Face.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <vector>

namespace {
    constexpr G4double kMeshDeflection  = 1.0;
    constexpr G4double kProbeMultiplier = 10.0;
    constexpr int      kMaxSurfRetry    = 20;

    G4Mutex polyhedronMutex = G4MUTEX_INITIALIZER;

    struct RayHit {
        G4double                          w;
        IntCurveSurface_TransitionOnCurve trans;
        TopoDS_Face                       face;
        G4double                          u, v;
        RayHit(G4double w_, IntCurveSurface_TransitionOnCurve t,
               const TopoDS_Face& f, G4double u_, G4double v_)
            : w(w_), trans(t), face(f), u(u_), v(v_) {}
    };

    struct HitGroup {
        G4double    dist;
        int         nIn     = 0;
        int         nOut    = 0;
        TopoDS_Face outFace;       // first Out-transition face (for normal + band)
        G4double    outU    = 0;
        G4double    outV    = 0;
        TopoDS_Face anyFace;       // first any-transition face (fallback for band)
    };

    // Takes hits by value so we can sort in place.
    std::vector<HitGroup> GroupHits(std::vector<RayHit> hits, G4double tol) {
        if (hits.empty()) return {};
        std::sort(hits.begin(), hits.end(),
                  [](const RayHit& a, const RayHit& b){ return a.w < b.w; });

        std::vector<HitGroup> groups;
        for (const RayHit& h : hits) {
            // Tangent intersections do not cross the boundary; skip them.
            if (h.trans == IntCurveSurface_Tangent) continue;
            if (!groups.empty() && h.w - groups.back().dist < tol) {
                if (h.trans == IntCurveSurface_In) {
                    groups.back().nIn++;
                } else {
                    groups.back().nOut++;
                    if (groups.back().outFace.IsNull()) {
                        groups.back().outFace = h.face;
                        groups.back().outU    = h.u;
                        groups.back().outV    = h.v;
                    }
                }
                if (groups.back().anyFace.IsNull()) groups.back().anyFace = h.face;
            } else {
                HitGroup g;
                g.dist    = h.w;
                g.anyFace = h.face;
                if (h.trans == IntCurveSurface_In) {
                    g.nIn = 1;
                } else {
                    g.nOut    = 1;
                    g.outFace = h.face;
                    g.outU    = h.u;
                    g.outV    = h.v;
                }
                groups.push_back(g);
            }
        }
        return groups;
    }

    inline gp_Pnt       G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Dir& v)       { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Pnt& v)       { return G4ThreeVector(v.X(), v.Y(), v.Z()); }

    // Minimum possible distance from a point to an axis-aligned bounding box.
    // Returns 0 if the point is inside the box.
    inline G4double BBoxPointDist(const Bnd_Box& box, const gp_Pnt& p) {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        double dx = std::max({xmin - p.X(), p.X() - xmax, 0.0});
        double dy = std::max({ymin - p.Y(), p.Y() - ymax, 0.0});
        double dz = std::max({zmin - p.Z(), p.Z() - zmax, 0.0});
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // Slab test: true if the ray (orig, invDir = 1/dir) could intersect the box
    // for W >= -wMin (wMin matches the back-search tolerance from Perform's first arg).
    // Uses IEEE arithmetic — 1/0 = ±inf is handled correctly.
    inline bool rayHitsBox(const gp_Pnt& orig, const gp_Vec& invDir,
                           const Bnd_Box& box, G4double wMin) {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        // Expand spatially by wMin to catch near-miss cases at face boundaries.
        xmin -= wMin; ymin -= wMin; zmin -= wMin;
        xmax += wMin; ymax += wMin; zmax += wMin;

        double tx0 = (xmin - orig.X()) * invDir.X();
        double tx1 = (xmax - orig.X()) * invDir.X();
        double tEnter = std::min(tx0, tx1), tExit = std::max(tx0, tx1);

        double ty0 = (ymin - orig.Y()) * invDir.Y();
        double ty1 = (ymax - orig.Y()) * invDir.Y();
        tEnter = std::max(tEnter, std::min(ty0, ty1));
        tExit  = std::min(tExit,  std::max(ty0, ty1));

        double tz0 = (zmin - orig.Z()) * invDir.Z();
        double tz1 = (zmax - orig.Z()) * invDir.Z();
        tEnter = std::max(tEnter, std::min(tz0, tz1));
        tExit  = std::min(tExit,  std::max(tz0, tz1));

        return tEnter <= tExit && tExit >= -wMin;
    }
} // namespace

// ====================================================================
// Static member definitions
// ====================================================================
std::vector<G4StepSolid::AlgoCache*>    G4StepSolid::sAllCaches;
std::mutex                               G4StepSolid::sAllCachesMutex;

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
        { Bnd_Box b; BRepBndLib::Add(s, b); b.Enlarge(band); solidBoxes.push_back(b); }
        for (TopExp_Explorer fe(s, TopAbs_FACE); fe.More(); fe.Next())
            if (!faceSolidMap.IsBound(fe.Current()))
                faceSolidMap.Bind(fe.Current(), si);
    }
    // Global mode: OCCT search/grouping uses plain requested tol (pure "current dev").
    occtTol = perSolidTol ? globalMax : requestedTol;
    if (solidTol.empty()) { solidTol.push_back(requestedTol); occtTol = requestedTol; }

    // Per-face intersectors — same class as IntCurvesFace_ShapeIntersector uses internally,
    // so transition semantics are identical. Built once per thread; reused across ray casts.
    for (const auto& fe : ownerSolid->fFaces)
        faceIntersectors.push_back(new IntCurvesFace_Intersector(fe.face, occtTol));

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

}

G4StepSolid::AlgoCache::~AlgoCache() {
    for (auto* p : faceIntersectors) delete p;
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

    std::map<GeomAbs_SurfaceType, int> typeCount;
    for (TopExp_Explorer ex(fShape, TopAbs_FACE); ex.More(); ex.Next()) {
        TopoDS_Face face = TopoDS::Face(ex.Current());

        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        G4double area = props.Mass();
        if (area <= 0.0) continue;

        BRepAdaptor_Surface surf(face);
        GeomAbs_SurfaceType st = surf.GetType();
        typeCount[st]++;

        FaceEntry fe;
        fe.face = face;
        BRepBndLib::Add(face, fe.bbox);
        fe.u0 = surf.FirstUParameter();
        fe.u1 = surf.LastUParameter();
        fe.v0 = surf.FirstVParameter();
        fe.v1 = surf.LastVParameter();

        fFaces.push_back(fe);

        cumArea += area;
        fCumulativeArea.push_back(cumArea);
    }

    static const char* typeName[] = {
        "Plane","Cylinder","Cone","Sphere","Torus",
        "Bezier","BSpline","Revolution","Extrusion","Offset","Other"
    };
    G4cout << "[G4StepSolid] " << GetName() << ": " << fFaces.size() << " faces — ";
    for (auto& kv : typeCount)
        G4cout << typeName[std::min((int)kv.first,10)] << "=" << kv.second << " ";
    G4cout << G4endl;
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

G4ThreeVector G4StepSolid::GetNormalAtIntersection(const TopoDS_Face& face,
                                                    G4double u, G4double v) const {
    BRepAdaptor_Surface surf(face);
    gp_Pnt pSurf; gp_Vec d1u, d1v;
    surf.D1(u, v, pSurf, d1u, d1v);
    gp_Vec norm = d1u.Crossed(d1v);
    if (face.Orientation() == TopAbs_REVERSED) norm.Reverse();
    return OCCTtoG4(gp_Dir(norm));
}

// ====================================================================
// NearestFace
// ====================================================================
G4StepSolid::NearestFaceResult G4StepSolid::NearestFace(const gp_Pnt& pt) const {
    NearestFaceResult result;
    if (fFaces.empty()) return result;

    // Load the query point once; iterate faces as S2 so only the shape side varies.
    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(pt).Shape());

    for (int i = 0; i < (int)fFaces.size(); ++i) {
        // Skip faces whose bbox lower-bound already exceeds the current best.
        if (BBoxPointDist(fFaces[i].bbox, pt) >= result.dist) continue;
        extrema.LoadS2(fFaces[i].face);
        extrema.Perform();
        if (!extrema.IsDone()) continue;
        G4double d = extrema.Value();
        if (d < result.dist) {
            result.dist    = d;
            result.idx     = i;
            result.pOnFace = extrema.PointOnShape2(1);
        }
    }
    return result;
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
    const gp_Vec invDir(1.0/v.x(), 1.0/v.y(), 1.0/v.z());

    std::vector<RayHit> rawHits;
    for (int fi = 0; fi < (int)fFaces.size(); ++fi) {
        const FaceEntry& fe = fFaces[fi];
        if (!rayHitsBox(ray.Location(), invDir, fe.bbox, octTol)) continue;

        IntCurvesFace_Intersector& inter = *algo->faceIntersectors[fi];
        inter.Perform(ray, -octTol, kInfinity);
        for (int j = 1; j <= inter.NbPnt(); ++j) {
            rawHits.emplace_back(inter.WParameter(j), inter.Transition(j),
                                 fe.face, inter.UParameter(j), inter.VParameter(j));
        }
    }

    if (rawHits.empty())
        return (kind == BoundaryKind::Entry) ? kInfinity : 0.0;

    const bool wantExit = (kind == BoundaryKind::Exit);
    const char* fnName  = wantExit ? "DistanceToOut" : "DistanceToIn";

    // Surface band for the solid owning a hit group (per-solid tolerance).
    auto bandOf = [&](const HitGroup& g) -> G4double {
        const TopoDS_Face& f = !g.outFace.IsNull() ? g.outFace : g.anyFace;
        int si = f.IsNull() ? -1 : algo->FaceToSolid(f);
        return algo->SolidBand(si);
    };

    auto isBoundaryGroup = [&](const HitGroup& g) -> bool {
        int net = wantExit ? (g.nOut - g.nIn) : (g.nIn - g.nOut);
        return net > 0;  // net=0 → assembly joint (In+Out cancel) → not a real boundary
    };

    for (const auto& g : GroupHits(std::move(rawHits), octTol)) {
        if (!isBoundaryGroup(g)) continue;

        const G4double band = bandOf(g);  // per-solid surface band
        if (std::abs(g.dist) < band) {
            if (calcNorm && validNorm && !g.outFace.IsNull()) {
                *n = GetNormalAtIntersection(g.outFace, g.outU, g.outV);
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
            if (calcNorm && validNorm && !g.outFace.IsNull()) {
                *n = GetNormalAtIntersection(g.outFace, g.outU, g.outV);
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
    if (p.x() < fXmin || p.x() > fXmax ||
        p.y() < fYmin || p.y() > fYmax ||
        p.z() < fZmin || p.z() > fZmax)
    {
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::Inside][local]"
                   << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                   << " -> kOutside (bbox)" << G4endl;
        }
        return kOutside;
    }

    AlgoCache* algo = GetAlgo();
    EInside result = kOutside;
    const gp_Pnt pt = G4toOCCT(p);
    for (size_t i = 0; i < algo->solidClassifiers.size(); ++i) {
        if (algo->solidBoxes[i].IsOut(pt)) continue;
        algo->solidClassifiers[i]->Perform(pt, algo->SolidBand((int)i));
        auto s = algo->solidClassifiers[i]->State();
        if (s == TopAbs_IN) { result = kInside; break; }
        if (s == TopAbs_ON) result = kSurface;
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
    G4double result = RayCastToBoundary(p, v, BoundaryKind::Entry);
    if (sRecordEnabled)
        RecordQuery(QueryType::DistToIn, p, v, 0, result, -1);
    return result;
}

G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p) const {
    G4double dx = std::max({fXmin - p.x(), p.x() - fXmax, 0.0});
    G4double dy = std::max({fYmin - p.y(), p.y() - fYmax, 0.0});
    G4double dz = std::max({fZmin - p.z(), p.z() - fZmax, 0.0});
    G4double bboxDist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (bboxDist > 0.0) {
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToIn(p)][local]"
                   << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                   << " -> " << bboxDist << " (bbox)" << G4endl;
        }
        return bboxDist;
    }

    NearestFaceResult nr = NearestFace(G4toOCCT(p));
    G4double result = (nr.idx >= 0) ? nr.dist : 0.0;
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
    G4double result = RayCastToBoundary(p, v, BoundaryKind::Exit, calcNorm, validNorm, n);
    if (sRecordEnabled) {
        // Rigorous stuck signature: Inside==kInside AND DistanceToOut≈0.
        int insideAtP = Inside(p);
        RecordQuery(QueryType::DistToOut, p, v, 0, result, insideAtP);
    }
    return result;
}

G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p) const {
    if (Inside(p) == kOutside) {
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToOut(p)][local]"
                   << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                   << " -> 0.0 (outside)" << G4endl;
        }
        return 0.0;
    }

    NearestFaceResult nr = NearestFace(G4toOCCT(p));
    G4double result = (nr.idx >= 0) ? nr.dist : 0.0;
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
    NearestFaceResult nr = NearestFace(G4toOCCT(p));
    if (nr.idx < 0) {
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::SurfaceNormal][local]"
                   << " no solution -> (0,0,1)" << G4endl;
        }
        return G4ThreeVector(0, 0, 1);
    }

    const TopoDS_Face& face = fFaces[nr.idx].face;
    BRepAdaptor_Surface surf(face);
    GeomAPI_ProjectPointOnSurf proj(nr.pOnFace, surf.Surface().Surface());
    Standard_Real u, v;
    proj.LowerDistanceParameters(u, v);
    gp_Pnt pSurf; gp_Vec d1u, d1v;
    surf.D1(u, v, pSurf, d1u, d1v);
    gp_Vec norm = d1u.Crossed(d1v);
    if (face.Orientation() == TopAbs_REVERSED) norm.Reverse();
    G4ThreeVector result = OCCTtoG4(gp_Dir(norm));

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

std::ostream& G4StepSolid::StreamInfo(std::ostream& os) const {
    os << "G4StepSolid: " << GetName()
       << "  bbox=[" << fXmin << "," << fXmax << "] x ["
       << fYmin << "," << fYmax << "] x ["
       << fZmin << "," << fZmax << "]"
       << "  tolerance=" << fTolerance
       << "  faces=" << fFaces.size();
    return os;
}
