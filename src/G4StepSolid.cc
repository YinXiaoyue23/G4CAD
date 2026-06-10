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

// OCCT Headers
#include <TopoDS.hxx>
#include <BRep_Tool.hxx>
#include <BRepTools.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
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
#include <vector>

// ====================================================================
// Constants
// ====================================================================
namespace {
    constexpr G4double kMeshDeflection  = 1.0;   // BRepMesh deflection (mm)
    constexpr G4double kProbeMultiplier = 10.0;   // probe offset = tol * kProbeMultiplier
    constexpr int      kMaxSurfRetry    = 20;     // GetPointOnSurface max retry attempts

    G4Mutex polyhedronMutex = G4MUTEX_INITIALIZER;

    // ====================================================================
    // Helper: group ray-face intersections within tolerance.
    // net = nOut - nIn per group:
    //   net > 0  → real exit   (use for DistanceToOut)
    //   net < 0  → real entry  (use for DistanceToIn)
    //   net = 0  → internal coincident face; classify further via probe point
    // ====================================================================
    struct HitGroup {
        G4double dist;
        int      nIn  = 0;
        int      nOut = 0;
        int      firstOutIdx = -1;
    };

    std::vector<HitGroup> GroupHits(const IntCurvesFace_ShapeIntersector* isect,
                                    G4double tol)
    {
        // Precondition: isect->Perform() has been called for the current ray.
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
            if (!groups.empty() && d - groups.back().dist < tol) {
                if (trans == IntCurveSurface_In) groups.back().nIn++;
                else {
                    groups.back().nOut++;
                    if (groups.back().firstOutIdx < 0) groups.back().firstOutIdx = idx;
                }
            } else {
                HitGroup g;
                g.dist = d;
                if (trans == IntCurveSurface_In) { g.nIn = 1; }
                else { g.nOut = 1; g.firstOutIdx = idx; }
                groups.push_back(g);
            }
        }
        return groups;
    }

    // Helper conversions
    inline gp_Pnt       G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Dir& v)       { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Pnt& v)       { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
} // namespace

// ====================================================================
// Static member definitions
// ====================================================================
std::vector<G4StepSolid::AlgoCache*> G4StepSolid::sAllCaches;
std::mutex                            G4StepSolid::sAllCachesMutex;

// ====================================================================
// AlgoCache
// ====================================================================
G4StepSolid::AlgoCache::AlgoCache(const TopoDS_Shape& shape, G4double tolerance) {
    classifier  = new BRepClass3d_SolidClassifier(shape);
    intersector = new IntCurvesFace_ShapeIntersector();
    intersector->Load(shape, tolerance);
}

G4StepSolid::AlgoCache::~AlgoCache() {
    delete classifier;
    delete intersector;
}

// ====================================================================
// Construction and destruction
// ====================================================================
G4StepSolid::G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape,
                          G4double tolerance)
    : G4VSolid(name), fShape(stepShape), fTolerance(tolerance)
{
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
    : G4VSolid(rhs), fShape(rhs.fShape),
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
        algo = new AlgoCache(fShape, fTolerance);
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
// RayCastToBoundary — unified core used by DistanceToIn/Out(p,v)
//
// For Entry: finds the first real entry group (net = nIn-nOut > 0)
// For Exit:  finds the first real exit  group (net = nOut-nIn > 0)
// net == 0 is resolved by probing a point just past the group:
//   Exit probe outside → real exit;  inside → internal coincident face
//   Entry probe inside → real entry; outside → external coincident face
// ====================================================================
G4double G4StepSolid::RayCastToBoundary(const G4ThreeVector& p, const G4ThreeVector& v,
                                         BoundaryKind kind,
                                         G4bool calcNorm, G4bool* validNorm,
                                         G4ThreeVector* n) const
{
    if (validNorm) *validNorm = false;
    if (v.mag2() < fTolerance * fTolerance) return kInfinity;

    AlgoCache* algo = GetAlgo();
    gp_Lin ray(G4toOCCT(p), gp_Dir(v.x(), v.y(), v.z()));
    algo->intersector->Perform(ray, -fTolerance, kInfinity);

    if (!algo->intersector->IsDone() || algo->intersector->NbPnt() == 0)
        return (kind == BoundaryKind::Entry) ? kInfinity : 0.0;

    const bool wantExit = (kind == BoundaryKind::Exit);

    auto isBoundaryGroup = [&](const HitGroup& g) -> bool {
        int net = wantExit ? (g.nOut - g.nIn) : (g.nIn - g.nOut);
        if (net > 0) return true;
        if (net < 0) return false;
        // net == 0: probe just past to distinguish internal from external coincident face
        G4ThreeVector probe = p + (g.dist + fTolerance * kProbeMultiplier) * v;
        algo->classifier->Perform(G4toOCCT(probe), fTolerance);
        TopAbs_State state = algo->classifier->State();
        bool probeOutside = (state == TopAbs_OUT);
        G4CAD_TRACE("[G4StepSolid::" << GetName() << "] coincident-face probe dist=" << g.dist
                    << " -> " << (probeOutside ? "exit" : "pass-through"));
        return wantExit ? probeOutside : !probeOutside;
    };

    for (const auto& g : GroupHits(algo->intersector, fTolerance)) {
        if (!isBoundaryGroup(g)) continue;

        if (std::abs(g.dist) < fTolerance) {
            if (calcNorm && validNorm && g.firstOutIdx >= 0) {
                *n = GetNormalAtIntersection(g.firstOutIdx, algo);
                *validNorm = true;
            }
            G4CAD_TRACE("[G4StepSolid::" << GetName() << "::"
                        << (wantExit ? "DistanceToOut" : "DistanceToIn") << "][local]"
                        << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                        << " v=(" << v.x() << "," << v.y() << "," << v.z() << ")"
                        << " -> 0.0 (on boundary)");
            return 0.0;
        }
        if (g.dist > fTolerance) {
            if (calcNorm && validNorm && g.firstOutIdx >= 0) {
                *n = GetNormalAtIntersection(g.firstOutIdx, algo);
                *validNorm = true;
            }
            G4CAD_TRACE("[G4StepSolid::" << GetName() << "::"
                        << (wantExit ? "DistanceToOut" : "DistanceToIn") << "][local]"
                        << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                        << " v=(" << v.x() << "," << v.y() << "," << v.z() << ")"
                        << " -> " << g.dist);
            return g.dist;
        }
    }

    if (kind == BoundaryKind::Entry) {
        G4CAD_TRACE("[G4StepSolid::" << GetName() << "::DistanceToIn][local]"
                    << " no real entry -> kInfinity");
        return kInfinity;
    }
    G4CAD_TRACE("[G4StepSolid::" << GetName() << "::DistanceToOut][local]"
                << " no real exit -> 0.0 (point outside)");
    return 0.0;
}

// ====================================================================
// Inside
// ====================================================================
EInside G4StepSolid::Inside(const G4ThreeVector& p) const {
    if (p.x() < fXmin || p.x() > fXmax ||
        p.y() < fYmin || p.y() > fYmax ||
        p.z() < fZmin || p.z() > fZmax)
    {
        G4CAD_TRACE("[G4StepSolid::" << GetName() << "::Inside][local]"
                    << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                    << " -> kOutside (bbox)");
        return kOutside;
    }

    AlgoCache* algo = GetAlgo();
    algo->classifier->Perform(G4toOCCT(p), fTolerance);
    TopAbs_State state = algo->classifier->State();

    EInside result = (state == TopAbs_IN)  ? kInside
                   : (state == TopAbs_OUT) ? kOutside
                   :                         kSurface;

    G4CAD_TRACE("[G4StepSolid::" << GetName() << "::Inside][local]"
                << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                << " -> " << (result==kInside?"kInside":result==kOutside?"kOutside":"kSurface"));
    return result;
}

// ====================================================================
// DistanceToIn
// ====================================================================
G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const {
    return RayCastToBoundary(p, v, BoundaryKind::Entry);
}

G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p) const {
    G4double dx = std::max({fXmin - p.x(), p.x() - fXmax, 0.0});
    G4double dy = std::max({fYmin - p.y(), p.y() - fYmax, 0.0});
    G4double dz = std::max({fZmin - p.z(), p.z() - fZmax, 0.0});
    G4double bboxDist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (bboxDist > 0.0) {
        G4CAD_TRACE("[G4StepSolid::" << GetName() << "::DistanceToIn(p)][local]"
                    << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                    << " -> " << bboxDist << " (bbox)");
        return bboxDist;
    }

    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    G4double result = extrema.IsDone() ? extrema.Value() : 0.0;
    G4CAD_TRACE("[G4StepSolid::" << GetName() << "::DistanceToIn(p)][local]"
                << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                << " -> " << result);
    return result;
}

// ====================================================================
// DistanceToOut
// ====================================================================
G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                                    const G4bool calcNorm, G4bool* validNorm,
                                    G4ThreeVector* n) const {
    return RayCastToBoundary(p, v, BoundaryKind::Exit, calcNorm, validNorm, n);
}

G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p) const {
    if (Inside(p) == kOutside) {
        G4CAD_TRACE("[G4StepSolid::" << GetName() << "::DistanceToOut(p)][local]"
                    << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                    << " -> 0.0 (outside)");
        return 0.0;
    }

    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    G4double result = extrema.IsDone() ? extrema.Value() : 0.0;
    G4CAD_TRACE("[G4StepSolid::" << GetName() << "::DistanceToOut(p)][local]"
                << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                << " -> " << result);
    return result;
}

// ====================================================================
// SurfaceNormal
// ====================================================================
G4ThreeVector G4StepSolid::SurfaceNormal(const G4ThreeVector& p) const {
    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    if (!extrema.IsDone() || extrema.NbSolution() == 0) {
        G4CAD_TRACE("[G4StepSolid::" << GetName() << "::SurfaceNormal][local]"
                    << " no solution -> (0,0,1)");
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

    G4CAD_TRACE("[G4StepSolid::" << GetName() << "::SurfaceNormal][local]"
                << " p=(" << p.x() << "," << p.y() << "," << p.z() << ")"
                << " -> (" << result.x() << "," << result.y() << "," << result.z() << ")");
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

    // Fallback: UV midpoint of first face
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
    G4CAD_TRACE("[G4StepSolid::" << GetName() << "::BoundingLimits]"
                << " pMin=(" << fXmin << "," << fYmin << "," << fZmin << ")"
                << " pMax=(" << fXmax << "," << fYmax << "," << fZmax << ")");
}

G4bool G4StepSolid::CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit,
                                    const G4AffineTransform& pTransform,
                                    G4double& pMin, G4double& pMax) const {
    G4ThreeVector bmin(fXmin, fYmin, fZmin), bmax(fXmax, fYmax, fZmax);
    G4BoundingEnvelope bbox(bmin, bmax);
    G4bool result = bbox.CalculateExtent(pAxis, pVoxelLimit, pTransform, pMin, pMax);
    G4CAD_TRACE("[G4StepSolid::" << GetName() << "::CalculateExtent]"
                << " axis=" << pAxis
                << " -> [" << pMin << "," << pMax << "]  ok=" << result);
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
    G4cout << ">>> G4StepSolid: generating mesh for " << GetName() << " ..." << G4endl;

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
