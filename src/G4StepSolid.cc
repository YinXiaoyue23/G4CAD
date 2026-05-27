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

// ====================================================================
// Static member definitions
// ====================================================================
std::vector<G4StepSolid::AlgoCache*> G4StepSolid::sAllCaches;
std::mutex                            G4StepSolid::sAllCachesMutex;

// ====================================================================
// Mutex for polyhedron visualisation
// ====================================================================
namespace {
    G4Mutex polyhedronMutex = G4MUTEX_INITIALIZER;
}

// ====================================================================
// Helper conversion functions
// ====================================================================
static inline gp_Pnt G4toOCCT(const G4ThreeVector& v)  { return gp_Pnt(v.x(), v.y(), v.z()); }
static inline G4ThreeVector OCCTtoG4(const gp_Dir& v)  { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
static inline G4ThreeVector OCCTtoG4(const gp_Pnt& v)  { return G4ThreeVector(v.X(), v.Y(), v.Z()); }

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

    // Destroy all per-thread AlgoCaches registered in the global registry
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
    fShape         = rhs.fShape;
    fXmin = rhs.fXmin; fXmax = rhs.fXmax;
    fYmin = rhs.fYmin; fYmax = rhs.fYmax;
    fZmin = rhs.fZmin; fZmax = rhs.fZmax;
    fTolerance     = rhs.fTolerance;
    fFaces         = rhs.fFaces;
    fCumulativeArea = rhs.fCumulativeArea;

    delete fpPolyhedron;
    fpPolyhedron       = nullptr;
    fRebuildPolyhedron = true;

    // Remove the current thread's cache from the registry and destroy it
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
// CalcBBox
// ====================================================================
void G4StepSolid::CalcBBox() {
    Bnd_Box box;
    BRepBndLib::Add(fShape, box);
    box.Get(fXmin, fYmin, fZmin, fXmax, fYmax, fZmax);
}

// ====================================================================
// BuildFaceWeights — run once at construction to precompute data needed by GetPointOnSurface
// ====================================================================
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

// ====================================================================
// GetAlgo — lazily initialise the current thread's AlgoCache
// ====================================================================
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

// ====================================================================
// Inside
// ====================================================================
EInside G4StepSolid::Inside(const G4ThreeVector& p) const {
    if (p.x() < fXmin || p.x() > fXmax ||
        p.y() < fYmin || p.y() > fYmax ||
        p.z() < fZmin || p.z() > fZmax)
        return kOutside;

    AlgoCache* algo = GetAlgo();
    algo->classifier->Perform(G4toOCCT(p), fTolerance);
    TopAbs_State state = algo->classifier->State();

    if (state == TopAbs_IN)  return kInside;
    if (state == TopAbs_OUT) return kOutside;
    return kSurface;
}

// ====================================================================
// DistanceToIn(p, v)
// ====================================================================
G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const {
    if (v.mag2() < fTolerance * fTolerance) return kInfinity;

    AlgoCache* algo = GetAlgo();
    gp_Lin ray(G4toOCCT(p), gp_Dir(v.x(), v.y(), v.z()));
    algo->intersector->Perform(ray, -fTolerance, kInfinity);

    if (!algo->intersector->IsDone() || algo->intersector->NbPnt() == 0)
        return kInfinity;

    G4double minDist = kInfinity;
    for (int i = 1; i <= algo->intersector->NbPnt(); ++i) {
        G4double dist = algo->intersector->WParameter(i);
        if (dist > fTolerance) {
            if (dist < minDist) minDist = dist;
        } else if (dist > -fTolerance &&
                   algo->intersector->Transition(i) == IntCurveSurface_In) {
            return 0.0;
        }
    }
    return minDist;
}

// ====================================================================
// DistanceToIn(p) — safety distance from an external point to the surface
// ====================================================================
G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p) const {
    // Point is outside the bounding box: bbox distance is a conservative lower bound, use as fast path
    G4double dx = std::max({fXmin - p.x(), p.x() - fXmax, 0.0});
    G4double dy = std::max({fYmin - p.y(), p.y() - fYmax, 0.0});
    G4double dz = std::max({fZmin - p.z(), p.z() - fZmax, 0.0});
    G4double bboxDist = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (bboxDist > 0.0) return bboxDist;

    // Point is inside the bounding box: compute exact distance to the shape surface
    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    return extrema.IsDone() ? extrema.Value() : 0.0;
}

// ====================================================================
// DistanceToOut(p, v)
// ====================================================================
G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                                    const G4bool calcNorm, G4bool* validNorm,
                                    G4ThreeVector* n) const {
    if (v.mag2() < fTolerance * fTolerance) {
        if (validNorm) *validNorm = false;
        return kInfinity;
    }

    AlgoCache* algo = GetAlgo();
    gp_Lin ray(G4toOCCT(p), gp_Dir(v.x(), v.y(), v.z()));
    algo->intersector->Perform(ray, -fTolerance, kInfinity);

    if (!algo->intersector->IsDone() || algo->intersector->NbPnt() == 0) {
        if (validNorm) *validNorm = false;
        return 0.0;  // point is outside; prevent infinite step
    }

    G4double minDist = kInfinity;
    int      minIdx  = -1;
    for (int i = 1; i <= algo->intersector->NbPnt(); ++i) {
        G4double dist = algo->intersector->WParameter(i);
        if (algo->intersector->Transition(i) != IntCurveSurface_Out) continue;

        if (std::abs(dist) < fTolerance) {
            if (calcNorm && validNorm) {
                *n        = GetNormalAtIntersection(i, algo);
                *validNorm = true;
            }
            return 0.0;
        }
        if (dist > fTolerance && dist < minDist) { minDist = dist; minIdx = i; }
    }

    if (minIdx != -1) {
        if (calcNorm && validNorm) {
            *n        = GetNormalAtIntersection(minIdx, algo);
            *validNorm = true;
        }
        return minDist;
    }
    if (validNorm) *validNorm = false;
    return 0.0;  // no outgoing intersection; point is outside
}

// ====================================================================
// DistanceToOut(p) — safety distance from an interior point to the surface
// ====================================================================
G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p) const {
    if (Inside(p) == kOutside) return 0.0;

    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    return extrema.IsDone() ? extrema.Value() : 0.0;
}

// ====================================================================
// SurfaceNormal
// ====================================================================
G4ThreeVector G4StepSolid::SurfaceNormal(const G4ThreeVector& p) const {
    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    if (!extrema.IsDone() || extrema.NbSolution() == 0)
        return G4ThreeVector(0, 0, 1);

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
        return OCCTtoG4(gp_Dir(norm));
    }
    gp_Pnt pLoc = extrema.PointOnShape2(1);
    G4ThreeVector dir = p - OCCTtoG4(pLoc);
    return (dir.mag2() > 0) ? dir.unit() : G4ThreeVector(0, 0, 1);
}

// ====================================================================
// GetNormalAtIntersection
// ====================================================================
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
// GetPointOnSurface
// ====================================================================
G4ThreeVector G4StepSolid::GetPointOnSurface() const {
    if (fFaces.empty()) return G4ThreeVector(0, 0, 0);

    const G4double totalArea = fCumulativeArea.back();

    // Retry up to 20 times; trimmed face UV domains may extend beyond the actual face boundary
    for (int attempt = 0; attempt < 20; ++attempt) {
        // Select a face by area weight
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

    // Fallback: use the UV midpoint of the first face, guaranteed to be on the surface
    const FaceEntry& fe = fFaces[0];
    BRepAdaptor_Surface surf(fe.face);
    gp_Pnt pt = surf.Value(0.5*(fe.u0+fe.u1), 0.5*(fe.v0+fe.v1));
    return G4ThreeVector(pt.X(), pt.Y(), pt.Z());
}

// ====================================================================
// BoundingLimits
// ====================================================================
void G4StepSolid::BoundingLimits(G4ThreeVector& pMin, G4ThreeVector& pMax) const {
    pMin.set(fXmin, fYmin, fZmin);
    pMax.set(fXmax, fYmax, fZmax);
}

// ====================================================================
// CalculateExtent
// ====================================================================
G4bool G4StepSolid::CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit,
                                    const G4AffineTransform& pTransform,
                                    G4double& pMin, G4double& pMax) const {
    G4ThreeVector bmin(fXmin, fYmin, fZmin), bmax(fXmax, fYmax, fZmax);
    G4BoundingEnvelope bbox(bmin, bmax);
    return bbox.CalculateExtent(pAxis, pVoxelLimit, pTransform, pMin, pMax);
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
    G4cout << ">>> Generating mesh for " << GetName() << " ..." << G4endl;

    try {
        BRepMesh_IncrementalMesh meshGen(fShape, 1.0);
    } catch (const std::exception& e) {
        G4cerr << "G4StepSolid::CreatePolyhedron: mesh generation failed for "
               << GetName() << ": " << e.what() << G4endl;
    } catch (...) {
        G4cerr << "G4StepSolid::CreatePolyhedron: mesh generation failed for "
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
