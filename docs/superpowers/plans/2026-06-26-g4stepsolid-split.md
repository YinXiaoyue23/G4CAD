# G4StepSolid File Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the 896-line `src/G4StepSolid.cc` into an interface layer (`G4StepSolid.cc`, ~380 lines) and a geometry engine (`G4StepSolidImpl.cc`, ~520 lines) without changing any public headers or build system files.

**Architecture:** All G4VSolid virtual functions, lifecycle, visualisation, and debug recorder stay in `G4StepSolid.cc`. The OCCT geometry engine — anonymous-namespace ray/distance helpers, `AlgoCache` constructor/destructor, `GetAlgo`, `GetNormalAtIntersection`, `NearestFace`, and `RayCastToBoundary` — moves to the new `G4StepSolidImpl.cc`. The two coordinate converters (`G4toOCCT`, `OCCTtoG4`) are trivial one-liners and are redefined in each file's own anonymous namespace. `include/G4StepSolid.hh` and `CMakeLists.txt` are unchanged.

**Tech Stack:** C++17, OCCT 7.x, Geant4 11.x, CMake with `file(GLOB SOURCES "src/*.cc")` (auto-picks up new file).

## Global Constraints

- `include/G4StepSolid.hh` must not be modified.
- `CMakeLists.txt` must not be modified.
- Zero new compiler warnings after the split.
- Post-split safety diagnostic: `ratio_median = 1.000`, `frac_lt_0.1 = 0` for all five geometries.
- Step counts within prior tolerances (0.99–1.01× for box/sphere/cylinder/touching_boxes, ~0.92× for box_hole).
- Working directory for all commands: `/home/yinxy/work/IR3/library/G4CAD`.

---

### Task 1: Create `src/G4StepSolidImpl.cc`

**Files:**
- Create: `src/G4StepSolidImpl.cc`

**Interfaces:**
- Consumes: `G4StepSolid.hh` (full class definition, `AlgoCache`, `FaceEntry`, `NearestFaceResult`, `BoundaryKind`)
- Produces: definitions of `AlgoCache::AlgoCache`, `AlgoCache::~AlgoCache`, `AlgoCache::FaceToSolid`, `G4StepSolid::GetAlgo`, `G4StepSolid::GetNormalAtIntersection`, `G4StepSolid::NearestFace`, `G4StepSolid::RayCastToBoundary`

- [ ] **Step 1: Verify the source lines to move**

Read the current file to confirm exact content before creating the new one:

```bash
wc -l src/G4StepSolid.cc   # expect 896
grep -n "^G4StepSolid::AlgoCache::AlgoCache\|^G4StepSolid::AlgoCache::~AlgoCache\|^int G4StepSolid::AlgoCache::FaceToSolid\|^G4StepSolid::AlgoCache\* G4StepSolid::GetAlgo\|^G4ThreeVector G4StepSolid::GetNormalAtIntersection\|^G4StepSolid::NearestFaceResult G4StepSolid::NearestFace\|^G4double G4StepSolid::RayCastToBoundary" src/G4StepSolid.cc
```

Expected output (line numbers may shift ±5 from edits):
```
184:int G4StepSolid::AlgoCache::FaceToSolid(const TopoDS_Face& f) const {
258:G4StepSolid::AlgoCache::AlgoCache(const TopoDS_Shape& shape, ...
317:G4StepSolid::AlgoCache::~AlgoCache() {
435:G4StepSolid::AlgoCache* G4StepSolid::GetAlgo() const {
446:G4ThreeVector G4StepSolid::GetNormalAtIntersection(const TopoDS_Face& face, ...
459:G4StepSolid::NearestFaceResult G4StepSolid::NearestFace(const gp_Pnt& pt) const {
489:G4double G4StepSolid::RayCastToBoundary(const G4ThreeVector& p, ...
```

- [ ] **Step 2: Create `src/G4StepSolidImpl.cc`**

Write the complete new file:

```cpp
#include "G4StepSolid.hh"
#include "G4SystemOfUnits.hh"

// OCCT
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <Bnd_Box.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <mutex>

namespace {
    // Coordinate converters — redefined here to avoid a shared private header
    // for three trivial one-liners.
    inline gp_Pnt        G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Dir& d)        { return G4ThreeVector(d.X(), d.Y(), d.Z()); }

    // Ray-casting primitives
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
        TopoDS_Face outFace;
        G4double    outU    = 0;
        G4double    outV    = 0;
        TopoDS_Face anyFace;
    };

    // Takes hits by value so we can sort in place.
    std::vector<HitGroup> GroupHits(std::vector<RayHit> hits, G4double tol) {
        if (hits.empty()) return {};
        std::sort(hits.begin(), hits.end(),
                  [](const RayHit& a, const RayHit& b){ return a.w < b.w; });

        std::vector<HitGroup> groups;
        for (const RayHit& h : hits) {
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

    // Minimum possible distance from point to AABB; 0 if inside.
    inline G4double BBoxPointDist(const Bnd_Box& box, const gp_Pnt& p) {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        double dx = std::max({xmin - p.X(), p.X() - xmax, 0.0});
        double dy = std::max({ymin - p.Y(), p.Y() - ymax, 0.0});
        double dz = std::max({zmin - p.Z(), p.Z() - zmax, 0.0});
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // Slab test: true if ray (orig, invDir=1/dir) could hit the box for W >= -wMin.
    inline bool rayHitsBox(const gp_Pnt& orig, const gp_Vec& invDir,
                           const Bnd_Box& box, G4double wMin) {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
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

// ====================================================================
// AlgoCache
// ====================================================================
int G4StepSolid::AlgoCache::FaceToSolid(const TopoDS_Face& f) const {
    if (faceSolidMap.IsBound(f)) return faceSolidMap.Find(f);
    return -1;
}

G4StepSolid::AlgoCache::AlgoCache(const TopoDS_Shape& shape, G4double tolerance,
                                   const G4StepSolid* ownerSolid)
{
    requestedTol = (tolerance > 0.0) ? tolerance : 1e-7;

    const char* psEnv = std::getenv("G4CAD_PERSOLID_TOL");
    perSolidTol = (psEnv && psEnv[0] == '1');

    G4double globalMax = requestedTol;
    int si = 0;
    for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next(), ++si) {
        const TopoDS_Shape& s = ex.Current();
        solidClassifiers.push_back(std::make_unique<BRepClass3d_SolidClassifier>(s));
        G4double sigma = ShapeNoise(s);
        G4double band  = std::max(requestedTol, 2.0 * sigma);
        solidTol.push_back(band);
        globalMax = std::max(globalMax, band);
        { Bnd_Box b; BRepBndLib::Add(s, b); b.Enlarge(band); solidBoxes.push_back(b); }
        for (TopExp_Explorer fe(s, TopAbs_FACE); fe.More(); fe.Next())
            if (!faceSolidMap.IsBound(fe.Current()))
                faceSolidMap.Bind(fe.Current(), si);
    }
    occtTol = perSolidTol ? globalMax : requestedTol;
    if (solidTol.empty()) { solidTol.push_back(requestedTol); occtTol = requestedTol; }

    for (const auto& fe : ownerSolid->fFaces)
        faceIntersectors.push_back(new IntCurvesFace_Intersector(fe.face, occtTol));

    for (const auto& fe : ownerSolid->fFaces) {
        auto* dss = new BRepExtrema_DistShapeShape();
        dss->LoadS2(fe.face);
        faceExtrema.push_back(dss);
    }

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
    for (auto* p : faceExtrema)      delete p;
}

// ====================================================================
// GetAlgo
// ====================================================================
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

// ====================================================================
// GetNormalAtIntersection
// ====================================================================
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

    const TopoDS_Shape vtxShape = BRepBuilderAPI_MakeVertex(pt).Shape();

    AlgoCache* algo = GetAlgo();
    for (int i = 0; i < (int)fFaces.size(); ++i) {
        if (BBoxPointDist(fFaces[i].bbox, pt) >= result.dist) continue;
        BRepExtrema_DistShapeShape& extrema = *algo->faceExtrema[i];
        extrema.LoadS1(vtxShape);
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

    if (rawHits.empty()) {
        if (kind == BoundaryKind::Entry) return kInfinity;
        G4double t = kInfinity;
        if (v.x() > 0.0)       t = std::min(t, (fXmax - p.x()) / v.x());
        else if (v.x() < 0.0)  t = std::min(t, (fXmin - p.x()) / v.x());
        if (v.y() > 0.0)       t = std::min(t, (fYmax - p.y()) / v.y());
        else if (v.y() < 0.0)  t = std::min(t, (fYmin - p.y()) / v.y());
        if (v.z() > 0.0)       t = std::min(t, (fZmax - p.z()) / v.z());
        else if (v.z() < 0.0)  t = std::min(t, (fZmin - p.z()) / v.z());
        return (t > 0.0 && std::isfinite(t)) ? t : 0.0;
    }

    const bool wantExit = (kind == BoundaryKind::Exit);
    const char* fnName  = wantExit ? "DistanceToOut" : "DistanceToIn";

    auto bandOf = [&](const HitGroup& g) -> G4double {
        const TopoDS_Face& f = !g.outFace.IsNull() ? g.outFace : g.anyFace;
        int si = f.IsNull() ? -1 : algo->FaceToSolid(f);
        return algo->SolidBand(si);
    };

    auto isBoundaryGroup = [&](const HitGroup& g) -> bool {
        int net = wantExit ? (g.nOut - g.nIn) : (g.nIn - g.nOut);
        return net > 0;
    };

    for (const auto& g : GroupHits(std::move(rawHits), octTol)) {
        if (!isBoundaryGroup(g)) continue;

        const G4double band = bandOf(g);
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

    if (kind == BoundaryKind::Entry) {
        if (fVerboseLevel >= 2)
            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToIn][local]"
                   << " no real boundary -> kInfinity" << G4endl;
        return kInfinity;
    }
    G4double t = kInfinity;
    if (v.x() > 0.0)       t = std::min(t, (fXmax - p.x()) / v.x());
    else if (v.x() < 0.0)  t = std::min(t, (fXmin - p.x()) / v.x());
    if (v.y() > 0.0)       t = std::min(t, (fYmax - p.y()) / v.y());
    else if (v.y() < 0.0)  t = std::min(t, (fYmin - p.y()) / v.y());
    if (v.z() > 0.0)       t = std::min(t, (fZmax - p.z()) / v.z());
    else if (v.z() < 0.0)  t = std::min(t, (fZmin - p.z()) / v.z());
    if (fVerboseLevel >= 2)
        G4cout << "[G4StepSolid::" << GetName() << "::DistanceToOut][local]"
               << " no real boundary -> bbox=" << t << G4endl;
    return (t > 0.0 && std::isfinite(t)) ? t : 0.0;
}
```

- [ ] **Step 3: Verify the file was written**

```bash
wc -l src/G4StepSolidImpl.cc
grep -c "^G4" src/G4StepSolidImpl.cc  # expect ≥ 7 (function definitions)
```

---

### Task 2: Trim `src/G4StepSolid.cc` to the interface layer

**Files:**
- Modify: `src/G4StepSolid.cc`

**Interfaces:**
- Consumes: `G4StepSolidImpl.cc` definitions via the linker (no header dependency, same class)
- Produces: trimmed `G4StepSolid.cc` containing only interface, lifecycle, and recorder code

The trimming has three parts performed in order: (A) replace the include block, (B) replace the anonymous namespace block (lines 45–198), (C) remove the five engine function bodies (AlgoCache ctor/dtor, FaceToSolid, GetAlgo, GetNormalAtIntersection, NearestFace, RayCastToBoundary).

**Part A — includes**

- [ ] **Step 1: Replace the include block at the top of `G4StepSolid.cc`**

The current include block is lines 1–43.  Replace with a tighter set (remove headers only needed by the engine: `BRepExtrema_DistShapeShape.hxx`, `BRepBuilderAPI_MakeVertex.hxx`, `gp_Lin.hxx`, `IntCurvesFace_Intersector.hxx`, `BRepTools.hxx`, `TopExp.hxx`).

Find and replace the entire include block:

Old:
```cpp
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
```

New:
```cpp
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
#include <TopoDS_Face.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <vector>
```

**Part B — anonymous namespace cleanup**

- [ ] **Step 2: Replace the first anonymous namespace (lines ~45–154)**

The first `namespace { ... }` block contains: constants, RayHit, HitGroup, GroupHits, G4toOCCT, OCCTtoG4, BBoxPointDist, rayHitsBox.  Only the constants, G4toOCCT, and OCCTtoG4 stay.

Old (the entire first namespace block from `namespace {` through `} // namespace`):
```cpp
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
```

New (constants + converters only):
```cpp
namespace {
    constexpr G4double kMeshDeflection  = 1.0;
    constexpr G4double kProbeMultiplier = 10.0;
    constexpr int      kMaxSurfRetry    = 20;

    G4Mutex polyhedronMutex = G4MUTEX_INITIALIZER;

    inline gp_Pnt        G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Dir& v)        { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Pnt& v)        { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
} // namespace
```

- [ ] **Step 3: Remove the second anonymous namespace (ShapeNoise)**

This block sits between the static member definitions and the recorder section.

Old:
```cpp
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
```

New: *(delete entirely — no replacement)*

**Part C — remove engine function bodies**

- [ ] **Step 4: Remove `AlgoCache::FaceToSolid`**

Old:
```cpp
int G4StepSolid::AlgoCache::FaceToSolid(const TopoDS_Face& f) const {
    if (faceSolidMap.IsBound(f)) return faceSolidMap.Find(f);
    return -1;
}
```

New: *(delete entirely)*

- [ ] **Step 5: Remove `AlgoCache::AlgoCache` and `AlgoCache::~AlgoCache`**

Delete from the `// ====================================================================` comment before `// AlgoCache` through the closing `}` of `AlgoCache::~AlgoCache`:

Old (the whole AlgoCache section, lines ~255–320):
```cpp
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

    // Pre-built BRepExtrema objects for scalar safety (NearestFace).
    // S2 = face is set here (once per thread); per query only LoadS1+Perform() is called,
    // avoiding the per-call cost of re-decomposing the face shape.
    for (const auto& fe : ownerSolid->fFaces) {
        auto* dss = new BRepExtrema_DistShapeShape();
        dss->LoadS2(fe.face);
        faceExtrema.push_back(dss);
    }

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
    for (auto* p : faceExtrema)      delete p;
}
```

New: *(delete entirely — both ctor and dtor)*

- [ ] **Step 6: Remove `GetAlgo`, `GetNormalAtIntersection`, `NearestFace`, `RayCastToBoundary`**

Delete from `// ====================================================================` before `// NearestFace` all the way through the end of `RayCastToBoundary`. Also delete the standalone `GetAlgo` and `GetNormalAtIntersection` definitions that precede it.

The block to remove is everything from (and including) `G4StepSolid::AlgoCache* G4StepSolid::GetAlgo() const {` through the closing `}` of `RayCastToBoundary`:

Old (lines ~435–601):
```cpp
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

    // Build vertex shape once for this query point.
    const TopoDS_Shape vtxShape = BRepBuilderAPI_MakeVertex(pt).Shape();

    // Use per-thread pre-built BRepExtrema objects (S2=face, set at AlgoCache construction).
    // Only S1 (the query vertex) changes per call.  This avoids re-decomposing each face
    // on every safety query, eliminating the dominant per-call OCCT allocation.
    AlgoCache* algo = GetAlgo();
    for (int i = 0; i < (int)fFaces.size(); ++i) {
        if (BBoxPointDist(fFaces[i].bbox, pt) >= result.dist) continue;
        BRepExtrema_DistShapeShape& extrema = *algo->faceExtrema[i];
        extrema.LoadS1(vtxShape);
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

    if (rawHits.empty()) {
        if (kind == BoundaryKind::Entry) return kInfinity;
        // No face intersections found for DistanceToOut.  This occurs at
        // degenerate surface points (e.g. sphere poles where OCCT classifies
        // all ray–surface intersections as tangent and they are skipped).
        // Returning 0 would tell G4 "already at the boundary", causing
        // premature particle exit and energy loss.  Use the bbox wall distance
        // in direction v as a conservative upper bound instead.
        G4double t = kInfinity;
        if (v.x() > 0.0)       t = std::min(t, (fXmax - p.x()) / v.x());
        else if (v.x() < 0.0)  t = std::min(t, (fXmin - p.x()) / v.x());
        if (v.y() > 0.0)       t = std::min(t, (fYmax - p.y()) / v.y());
        else if (v.y() < 0.0)  t = std::min(t, (fYmin - p.y()) / v.y());
        if (v.z() > 0.0)       t = std::min(t, (fZmax - p.z()) / v.z());
        else if (v.z() < 0.0)  t = std::min(t, (fZmin - p.z()) / v.z());
        return (t > 0.0 && std::isfinite(t)) ? t : 0.0;
    }

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

    // No valid boundary group found (all hit groups cancelled out or were skipped).
    // For DistanceToIn, no entry → infinity.
    // For DistanceToOut, avoid returning 0 (premature exit): fall back to bbox.
    if (kind == BoundaryKind::Entry) {
        if (fVerboseLevel >= 2)
            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToIn][local]"
                   << " no real boundary -> kInfinity" << G4endl;
        return kInfinity;
    }
    G4double t = kInfinity;
    if (v.x() > 0.0)       t = std::min(t, (fXmax - p.x()) / v.x());
    else if (v.x() < 0.0)  t = std::min(t, (fXmin - p.x()) / v.x());
    if (v.y() > 0.0)       t = std::min(t, (fYmax - p.y()) / v.y());
    else if (v.y() < 0.0)  t = std::min(t, (fYmin - p.y()) / v.y());
    if (v.z() > 0.0)       t = std::min(t, (fZmax - p.z()) / v.z());
    else if (v.z() < 0.0)  t = std::min(t, (fZmin - p.z()) / v.z());
    if (fVerboseLevel >= 2)
        G4cout << "[G4StepSolid::" << GetName() << "::DistanceToOut][local]"
               << " no real boundary -> bbox=" << t << G4endl;
    return (t > 0.0 && std::isfinite(t)) ? t : 0.0;
}
```

New: *(delete entirely)*

- [ ] **Step 7: Verify trimmed file line count**

```bash
wc -l src/G4StepSolid.cc   # expect ~380 (was 896)
grep -c "^G4double G4StepSolid::RayCastToBoundary\|^G4StepSolid::AlgoCache::AlgoCache\|^G4StepSolid::AlgoCache\* G4StepSolid::GetAlgo" src/G4StepSolid.cc
# expect 0 — these definitions must be gone
```

---

### Task 3: Build, install, and verify correctness

**Files:**
- No file changes — verification only.

**Interfaces:**
- Consumes: both .cc files compiled and linked
- Produces: confirmation that the split did not change any observable behavior

- [ ] **Step 1: Build the library**

```bash
cd /home/yinxy/work/IR3/library/G4CAD
cmake --build build -j$(nproc) 2>&1 | tail -20
```

Expected: zero errors, zero new warnings.  If there are errors:
- `undefined reference to G4StepSolid::...`: the function was removed from both files — check that it is present in `G4StepSolidImpl.cc`.
- `multiple definition of ...`: the function was NOT removed from `G4StepSolid.cc` — re-check Task 2 steps 4–6.
- `error: 'RayHit' was not declared`: an engine helper is still referenced in `G4StepSolid.cc` — check that `G4toOCCT`/`OCCTtoG4` are redefined in `G4StepSolid.cc`'s anonymous namespace and no other engine types are referenced there.

- [ ] **Step 2: Install the updated library**

```bash
cmake --install build --prefix install
```

Expected: completes without errors.  This updates the shared library that the benchmark binary links against (previously encountered problem: old library in `install/` would silently shadow the new one).

- [ ] **Step 3: Run safety diagnostic for box and cylinder (quick check)**

```bash
cd examples/validation_benchmark
source ../../install/bin/G4CADConfig.sh 2>/dev/null || true
./build/g4cad_bench \
    --geometry box --mode step \
    --step-file step_files/box.step \
    --events 0 --safety-diagnostic \
    --output /tmp/split_check_box
cat /tmp/split_check_box_safety_summary.csv
```

Expected: `ratio_median` column = `1.0`, `frac_lt_0p1` = `0`.

```bash
./build/g4cad_bench \
    --geometry box_hole --mode step \
    --step-file step_files/box_hole.step \
    --events 0 --safety-diagnostic \
    --output /tmp/split_check_boxhole
cat /tmp/split_check_boxhole_safety_summary.csv
```

Expected: same — `ratio_median = 1.0`, `frac_lt_0.1 = 0`.

- [ ] **Step 4: Run a quick physics run to confirm step counts unchanged**

```bash
./build/g4cad_bench \
    --geometry box_hole --mode step \
    --step-file step_files/box_hole.step \
    --events 500 --output /tmp/split_physics_boxhole
tail -1 /tmp/split_physics_boxhole_physics_summary.csv
```

Expected: `steps_mean` in range 24–25 (pre-split baseline was 24.81 ± 0.11).

- [ ] **Step 5: Commit**

```bash
cd /home/yinxy/work/IR3/library/G4CAD
git add src/G4StepSolidImpl.cc src/G4StepSolid.cc
git commit -m "refactor: split G4StepSolid.cc into interface + geometry engine

G4StepSolid.cc (~380 lines): G4VSolid overrides, lifecycle, visualisation,
debug recorder.
G4StepSolidImpl.cc (~520 lines): AlgoCache, GetAlgo, GetNormalAtIntersection,
NearestFace, RayCastToBoundary, anonymous-namespace ray/distance helpers.

No behaviour change; safety ratio=1.000 confirmed post-split."
```

---

## Self-Review

**Spec coverage:**
- ✓ `G4StepSolid.cc` → interface layer (G4VSolid overrides, lifecycle, visualisation, recorder)
- ✓ `G4StepSolidImpl.cc` → geometry engine (AlgoCache, helpers, NearestFace, RayCastToBoundary)
- ✓ `G4toOCCT`/`OCCTtoG4` redefined locally in each file's anonymous namespace
- ✓ `include/G4StepSolid.hh` unchanged
- ✓ `CMakeLists.txt` unchanged
- ✓ Verification: build + safety diagnostic + step count check

**Placeholder scan:** No TBDs, no vague steps — all code is explicit.

**Type consistency:** `NearestFaceResult`, `FaceEntry`, `AlgoCache`, `BoundaryKind` — all from the header; used identically in both task 1 and task 2.
