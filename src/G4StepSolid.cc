#include "G4StepSolid.hh"
#include "G4StepConfig.hh"
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
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Pln.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <TopoDS_Face.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <map>
#include <vector>

namespace {
    constexpr G4double kMeshDeflection  = 1.0;
    constexpr G4double kProbeMultiplier = 10.0;
    constexpr int      kMaxSurfRetry    = 20;

    G4Mutex polyhedronMutex = G4MUTEX_INITIALIZER;

    inline gp_Pnt        G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Dir& v)        { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Pnt& v)        { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
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
bool           G4StepSolid::sInsideForceOCCT = false;
std::ofstream  G4StepSolid::sRecordStream;
std::mutex     G4StepSolid::sRecordMutex;
std::once_flag G4StepSolid::sRecordInitFlag;
long           G4StepSolid::sRecordSeq       = 0;

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
    const std::string& path = G4StepConfig::Get().recordPath;
    if (path.empty()) { sRecordEnabled = false; return; }
    sRecordStream.open(path, std::ios::out | std::ios::trunc);
    if (!sRecordStream.is_open()) {
        G4cerr << "[G4StepSolid] G4CAD_RECORD set but cannot open " << path << G4endl;
        sRecordEnabled = false;
        return;
    }
    sRecordOnlyEvent = G4StepConfig::Get().recordEvent;
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
// Construction and destruction
// ====================================================================
G4StepSolid::G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape,
                          G4double tolerance)
    : G4VSolid(name), fShape(stepShape), fTolerance(tolerance)
{
    // Resolve the debug query recorder once (reads G4CAD_RECORD env); zero cost when off.
    // Per-solid tolerances are computed lazily per thread in AlgoCache (keeps ABI stable).
    std::call_once(sRecordInitFlag, &G4StepSolid::InitRecorderOnce);

    // CT-IN differential test gate (centralised in G4StepConfig, read once).
    if (G4StepConfig::Get().insideForceOCCT) sInsideForceOCCT = true;

    CalcBBox();
    BuildFaceWeights();

    // Enable per-call timing when G4CAD_TIMER is set; zero overhead otherwise.
    if (G4StepConfig::Get().timing) SetTimingEnabled(true);
}

G4StepSolid::~G4StepSolid() {
    delete fpPolyhedron;
    // Free ONLY this solid's per-thread AlgoCaches. sAllCaches is a process-wide
    // registry shared by every G4StepSolid instance (and read by the timing
    // aggregation, which already filters by owner). The old code deleted and cleared
    // the WHOLE registry here, so destroying one solid freed every other live solid's
    // caches, leaving their fAlgoCache G4Cache holding dangling pointers → UAF. Delete
    // only entries owned by `this` and compact the survivors back into the registry.
    std::lock_guard<std::mutex> lock(sAllCachesMutex);
    auto keep = sAllCaches.begin();
    for (AlgoCache* c : sAllCaches) {
        if (c->owner == this) delete c;
        else                  *keep++ = c;
    }
    sAllCaches.erase(keep, sAllCaches.end());
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

        switch (st) {
            case GeomAbs_Plane: {
                gp_Pln pln = surf.Plane();
                fe.surfType = FaceEntry::SurfType::Plane;
                fe.pos      = pln.Position();
                // Detect rectangular face: exactly 4 edges, all linear.
                // Only rectangular faces have UV bounding box == actual trim boundary.
                {
                    int nEdges = 0;
                    bool allLine = true;
                    for (TopExp_Explorer ew(face, TopAbs_EDGE); ew.More() && allLine; ew.Next()) {
                        BRepAdaptor_Curve c(TopoDS::Edge(ew.Current()));
                        allLine = (c.GetType() == GeomAbs_Line);
                        ++nEdges;
                    }
                    fe.isRectPlane = (allLine && nEdges == 4);
                }
                fe.analyticTrimSafe = fe.isRectPlane;  // CT-IN: plane trim-safe ⇔ rectangular
                break;
            }
            case GeomAbs_Cylinder: {
                gp_Cylinder cyl = surf.Cylinder();
                fe.surfType = FaceEntry::SurfType::Cylinder;
                fe.pos      = cyl.Position();
                fe.cylR     = cyl.Radius();
                // CT-IN: trim-safe iff full-periodic in u AND exactly 4 edges, all
                // Line (seams) or Circle (caps). Cutout/partial faces (curved edges
                // or edge count ≠ 4) over-cover the UV box → route through OCCT.
                {
                    bool fullU = (surf.LastUParameter() - surf.FirstUParameter()
                                  >= 2.0*M_PI - 1e-7);
                    int nEdges = 0;
                    bool simpleEdges = true;
                    for (TopExp_Explorer ew(face, TopAbs_EDGE); ew.More() && simpleEdges; ew.Next()) {
                        BRepAdaptor_Curve c(TopoDS::Edge(ew.Current()));
                        GeomAbs_CurveType ct = c.GetType();
                        simpleEdges = (ct == GeomAbs_Line || ct == GeomAbs_Circle);
                        ++nEdges;
                    }
                    fe.analyticTrimSafe = (fullU && simpleEdges && nEdges == 4);
                }
                break;
            }
            case GeomAbs_Sphere: {
                gp_Sphere sph = surf.Sphere();
                fe.surfType = FaceEntry::SurfType::Sphere;
                fe.pos      = sph.Position();
                fe.sphR     = sph.Radius();
                {
                    bool fullU = (surf.LastUParameter() - surf.FirstUParameter()
                                  >= 2.0*M_PI - 1e-7);
                    int nEdges = 0;
                    bool simpleEdges = true;
                    for (TopExp_Explorer ew(face, TopAbs_EDGE); ew.More() && simpleEdges; ew.Next()) {
                        BRepAdaptor_Curve c(TopoDS::Edge(ew.Current()));
                        GeomAbs_CurveType ct = c.GetType();
                        simpleEdges = (ct == GeomAbs_Line || ct == GeomAbs_Circle);
                        ++nEdges;
                    }
                    fe.analyticTrimSafe = (fullU && simpleEdges && nEdges == 4);
                }
                break;
            }
            case GeomAbs_Cone:
                // Phase A (Cone/Torus): classify only; no analytic surface math yet.
                // analyticTrimSafe=false routes the face through the trim-exact OCCT
                // per-face IntCurvesFace_Intersector in the ray-parity path, so that
                // cone/torus solids can use analytic-parity Inside() instead of the
                // BRepClass3d_SolidClassifier pathology. Geometric params (half-angle,
                // radii) are extracted later if Phase B/C adds analytic intersection.
                fe.surfType         = FaceEntry::SurfType::Cone;
                fe.analyticTrimSafe = false;
                break;
            case GeomAbs_Torus:
                fe.surfType         = FaceEntry::SurfType::Torus;
                fe.analyticTrimSafe = false;
                break;
            default:
                fe.surfType = FaceEntry::SurfType::Other;
                break;
        }

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
    const gp_Pnt pt = G4toOCCT(p);
    for (size_t i = 0; i < algo->solidClassifiers.size(); ++i) {
        if (algo->solidBoxes[i].IsOut(pt)) continue;

        // CT-IN: analytic point-in-solid via ray parity for solids whose faces are
        // ALL analytic-complete types (Plane/Cylinder/Sphere). Solids with Cone or
        // Torus faces fall back to BRepClass3d_SolidClassifier::Perform unchanged.
        // This replaces the per-call OCCT classifier (594k–1.5M ns/call pathology
        // on RP.step parts 0/1/2) with a sub-µs analytic ray cast.
        if (algo->analyticInsideOK[i] && !sInsideForceOCCT) {
            bool ok = true;
            EInside ai = InsideSolidAnalytic((int)i, p, algo, ok);
            if (ok) {
                // CT-IN debug self-check: compare analytic vs OCCT per solid.
                static bool selfcheck = G4StepConfig::Get().insideSelfcheck;
                if (selfcheck) {
                    algo->solidClassifiers[i]->Perform(pt, algo->SolidBand((int)i));
                    auto st = algo->solidClassifiers[i]->State();
                    EInside occt = (st==TopAbs_IN) ? kInside : (st==TopAbs_ON) ? kSurface : kOutside;
                    if (occt != ai) {
                        G4cout << "[SELFCHECK MISMATCH] " << GetName() << " solid#" << i
                               << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                               << " analytic=" << (int)ai << " occt=" << (int)occt << G4endl;
                    }
                }
                if (fTimingEnabled) ++algo->timing.insideAnalyticCount;
                if (ai == kInside) { result = kInside; break; }
                if (ai == kSurface) result = kSurface;
                continue;
            }
            // Ambiguous (degenerate ray after perturbations) → fall through to OCCT.
        }

        if (fTimingEnabled) ++algo->timing.insideFallbackCount;
        algo->solidClassifiers[i]->Perform(pt, algo->SolidBand((int)i));
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

    NearestFaceResult nr = NearestFace(G4toOCCT(p));
    G4double result = (nr.idx >= 0) ? nr.dist : 0.0;
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

    // G4 contract: DistanceToOut(p) is only called for points already inside the solid.
    // Replacing Inside() (which calls BRepClass3d_SolidClassifier::Perform()) with a
    // cheap AABB guard eliminates the dominant single-thread and MT bottleneck.
    if (p.x() < fXmin || p.x() > fXmax ||
        p.y() < fYmin || p.y() > fYmax ||
        p.z() < fZmin || p.z() > fZmax) {
        if (fTimingEnabled) {
            algo->timing.dtoScalNs += std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now() - t0).count();
            ++algo->timing.dtoScalCount;
        }
        if (fVerboseLevel >= 2) {
            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToOut(p)][local]"
                   << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                   << " -> 0.0 (outside AABB)" << G4endl;
        }
        return 0.0;
    }

    NearestFaceResult nr = NearestFace(G4toOCCT(p));
    G4double result = (nr.idx >= 0) ? nr.dist : 0.0;

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

    NearestFaceResult nr = NearestFace(G4toOCCT(p));
    if (nr.idx < 0) {
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
    int64_t uvTrimAccept = 0, uvTrimBandFallback = 0;  // CT-TRIM (AlgoCache members)
    {
        std::lock_guard<std::mutex> lock(sAllCachesMutex);
        for (const AlgoCache* c : sAllCaches) {
            if (c->owner != this) continue;
            uvTrimAccept       += c->uvTrimAccept;
            uvTrimBandFallback += c->uvTrimBandFallback;
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
            // Sub-counters
            total.rayFacesVisited  += c->timing.rayFacesVisited;
            total.rayPerformCount  += c->timing.rayPerformCount;
            total.rayHitsTotal     += c->timing.rayHitsTotal;
            total.rayAnalyticCount += c->timing.rayAnalyticCount;
            total.rayFallbackCount += c->timing.rayFallbackCount;
            total.nfFacesVisited   += c->timing.nfFacesVisited;
            total.nfExtremaCount   += c->timing.nfExtremaCount;
            total.nfAnalyticCount  += c->timing.nfAnalyticCount;
            total.nfFallbackCount  += c->timing.nfFallbackCount;
            total.insideAnalyticCount += c->timing.insideAnalyticCount;
            total.insideFallbackCount += c->timing.insideFallbackCount;
        }
    }
    G4cout << "[G4StepSolid timing] " << GetName() << G4endl;
    PrintTimingRow("Inside",              total.insideNs,   total.insideCount);
    PrintTimingRow("DistanceToIn(p,v)",   total.dtiVecNs,   total.dtiVecCount);
    PrintTimingRow("DistanceToIn(p)",     total.dtiScalNs,  total.dtiScalCount);
    PrintTimingRow("DistanceToOut(p,v)",  total.dtoVecNs,   total.dtoVecCount);
    PrintTimingRow("DistanceToOut(p)",    total.dtoScalNs,  total.dtoScalCount);
    PrintTimingRow("SurfaceNormal",       total.normalNs,   total.normalCount);
    // Inner-loop sub-counters
    int64_t dtiVecCalls = total.dtiVecCount;
    int64_t dtoVecCalls = total.dtoVecCount;
    int64_t vecCalls    = dtiVecCalls + dtoVecCalls;
    int64_t nfCalls     = total.dtoScalCount + total.normalCount;
    G4cout << "  -- RayCast inner (DistanceToIn/Out(p,v)) --" << G4endl;
    G4cout << "    ray vec calls (DTI+DTO):   " << vecCalls << G4endl;
    G4cout << "    ray faces visited (slab):  " << total.rayFacesVisited
           << (vecCalls ? G4String("  (~") + std::to_string(total.rayFacesVisited/(vecCalls?vecCalls:1)) + " faces/call)" : "") << G4endl;
    G4cout << "    ray Perform (OCCT):        " << total.rayPerformCount << G4endl;
    G4cout << "    ray analytic hits:         " << total.rayAnalyticCount << G4endl;
    G4cout << "    ray fallback hits:         " << total.rayFallbackCount << G4endl;
    G4cout << "    ray total hits produced:   " << total.rayHitsTotal << G4endl;
    G4cout << "  -- NearestFace (DistanceToOut(p) + SurfaceNormal) --" << G4endl;
    G4cout << "    nf scalar+normal calls:    " << nfCalls << G4endl;
    G4cout << "    nf faces visited (bbox):   " << total.nfFacesVisited
           << (nfCalls ? G4String("  (~") + std::to_string(total.nfFacesVisited/(nfCalls?nfCalls:1)) + " faces/call)" : "") << G4endl;
    G4cout << "    nf Extrema (OCCT):         " << total.nfExtremaCount << G4endl;
    G4cout << "    nf analytic hits:          " << total.nfAnalyticCount << G4endl;
    G4cout << "    nf fallback hits:          " << total.nfFallbackCount << G4endl;
    G4cout << "  -- Inside (CT-IN: analytic ray parity vs OCCT classifier) --" << G4endl;
    G4cout << "    inside analytic (parity):  " << total.insideAnalyticCount << G4endl;
    G4cout << "    inside fallback (OCCT):    " << total.insideFallbackCount << G4endl;
    G4cout << "  -- CT-TRIM/CT-TRIM-2 (UV v-band + seam-aware polygon trim) --" << G4endl;
    G4cout << "    uv-trim analytic accept:   " << uvTrimAccept << G4endl;
    G4cout << "    uv-trim band → OCCT:       " << uvTrimBandFallback << G4endl;
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
