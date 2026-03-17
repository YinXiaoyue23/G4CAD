#include "G4StepSolid.hh"
#include "G4SystemOfUnits.hh"
#include "G4VoxelLimits.hh"
#include "G4AffineTransform.hh"
#include "G4Polyhedron.hh"
#include "G4PolyhedronArbitrary.hh"
#include "G4BoundingEnvelope.hh" 
#include "G4VGraphicsScene.hh"
#include "G4AutoLock.hh" // Mutex for thread-safe polyhedron creation
#include "G4GeometryTolerance.hh"
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
#include <BRepClass_FaceClassifier.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

// Global mutex protecting CreatePolyhedron().
namespace {
  G4Mutex polyhedronMutex = G4MUTEX_INITIALIZER;

  constexpr Standard_Real kOcctTolerance = 1e-7;
  constexpr G4double kDirectionTolerance = 1e-18;

  G4double NavigationTolerance() {
    return std::max(10.0 * G4GeometryTolerance::GetInstance()->GetSurfaceTolerance(), 1e-7);
  }
}

// Helper conversion functions.
static inline gp_Pnt G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
static inline G4ThreeVector OCCTtoG4(const gp_Dir& v) { return G4ThreeVector(v.X(), v.Y(), v.Z()); }
static inline G4ThreeVector OCCTtoG4(const gp_Pnt& v) { return G4ThreeVector(v.X(), v.Y(), v.Z()); }

static G4double DistanceToShape(const TopoDS_Shape& shape, const G4ThreeVector& p) {
    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(shape);
    extrema.Perform();
    return extrema.IsDone() ? extrema.Value() : kInfinity;
}

// ====================================================================
// AlgoCache implementation (construction and destruction of thread-local data).
// ====================================================================
G4StepSolid::AlgoCache::AlgoCache(const TopoDS_Shape& shape) {
    classifier = new BRepClass3d_SolidClassifier(shape);
    intersector = new IntCurvesFace_ShapeIntersector();
    intersector->Load(shape, 1e-7); // Loading is relatively expensive, so each thread does it only once.
}

G4StepSolid::AlgoCache::~AlgoCache() {
    if (classifier) delete classifier;
    if (intersector) delete intersector;
}

// ====================================================================
// G4StepSolid construction and destruction.
// ====================================================================
G4StepSolid::G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape)
    : G4VSolid(name), fShape(stepShape)
{
    // Only compute the static bounding box here; do not initialize OCCT algorithms on construction.
    CalcBBox();
}

G4StepSolid::~G4StepSolid() {
    delete fpPolyhedron; fpPolyhedron = nullptr;
    
    // Clean up the cache owned by the current thread.
    // Note: caches created by other threads are reclaimed when those threads terminate,
    // or eventually by normal process cleanup at exit.
    AlgoCache* algo = fAlgoCache.Get();
    if(algo) delete algo; 
}

G4StepSolid::G4StepSolid(const G4StepSolid& rhs)
    : G4VSolid(rhs), fShape(rhs.fShape),
      fXmin(rhs.fXmin), fXmax(rhs.fXmax),
      fYmin(rhs.fYmin), fYmax(rhs.fYmax),
      fZmin(rhs.fZmin), fZmax(rhs.fZmax)
{
    // Do not copy the cache. A new cache will be created in the destination thread when needed.
    fpPolyhedron = nullptr;
    fRebuildPolyhedron = true;
}

G4StepSolid& G4StepSolid::operator=(const G4StepSolid& rhs) {
    if (this == &rhs) return *this;
    G4VSolid::operator=(rhs);
    fShape = rhs.fShape;
    fXmin = rhs.fXmin; fXmax = rhs.fXmax;
    fYmin = rhs.fYmin; fYmax = rhs.fYmax;
    fZmin = rhs.fZmin; fZmax = rhs.fZmax;
    
    delete fpPolyhedron; fpPolyhedron = nullptr;
    fRebuildPolyhedron = true;
    
    // Clear the cache for the current thread.
    AlgoCache* algo = fAlgoCache.Get();
    if(algo) { delete algo; fAlgoCache.Put(nullptr); }
    
    return *this;
}

void G4StepSolid::CalcBBox() {
    Bnd_Box box;
    BRepBndLib::Add(fShape, box);
    box.Get(fXmin, fYmin, fZmin, fXmax, fYmax, fZmax);
}

// ====================================================================
// Core helper: lazily retrieve the algorithm object for the current thread.
// ====================================================================
G4StepSolid::AlgoCache* G4StepSolid::GetAlgo() const {
    // Retrieve the pointer from thread-local storage.
    AlgoCache* algo = fAlgoCache.Get();
    
    // Create a new cache on first access in this thread.
    if (!algo) {
        algo = new AlgoCache(fShape);
        fAlgoCache.Put(algo);
    }
    return algo;
}

// ====================================================================
// Geometry navigation interface methods (all routed through GetAlgo()).
// ====================================================================

EInside G4StepSolid::Inside(const G4ThreeVector& p) const {
    const G4double tol = NavigationTolerance();

    if (p.x() < fXmin - tol || p.x() > fXmax + tol ||
        p.y() < fYmin - tol || p.y() > fYmax + tol ||
        p.z() < fZmin - tol || p.z() > fZmax + tol) {
        return kOutside;
    }

    AlgoCache* algo = GetAlgo();
    algo->classifier->Perform(G4toOCCT(p), kOcctTolerance);
    const TopAbs_State state = algo->classifier->State();

    if (state == TopAbs_IN) return kInside;
    if (state == TopAbs_ON) return kSurface;
    if (state == TopAbs_OUT) {
        return DistanceToShape(fShape, p) <= tol ? kSurface : kOutside;
    }
    return kSurface;
}

G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const {
    if (v.mag2() <= kDirectionTolerance) return kInfinity;

    const EInside state = Inside(p);
    if (state == kInside) return 0.0;

    AlgoCache* algo = GetAlgo();
    gp_Lin ray(G4toOCCT(p), gp_Dir(v.x(), v.y(), v.z()));
    algo->intersector->Perform(ray, -kOcctTolerance, kInfinity);

    if (!algo->intersector->IsDone() || algo->intersector->NbPnt() == 0) return kInfinity;

    const G4double tol = NavigationTolerance();
    G4double minDist = kInfinity;
    bool found = false;
    for (int i = 1; i <= algo->intersector->NbPnt(); ++i) {
        const G4double dist = algo->intersector->WParameter(i);
        if (dist > tol) {
            if (dist < minDist) { minDist = dist; found = true; }
        } else if (std::abs(dist) <= tol && algo->intersector->Transition(i) == IntCurveSurface_In) {
            return 0.0;
        }
    }

    if (state == kSurface) return found ? minDist : kInfinity;
    return found ? minDist : kInfinity;
}

G4double G4StepSolid::DistanceToIn(const G4ThreeVector& p) const {
    return Inside(p) == kOutside ? DistanceToShape(fShape, p) : 0.0;
}

G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                                    const G4bool calcNorm, G4bool *validNorm, G4ThreeVector *n) const {
    if (validNorm) *validNorm = false;
    if (v.mag2() <= kDirectionTolerance) return kInfinity;

    const EInside state = Inside(p);
    if (state == kOutside) return 0.0;

    AlgoCache* algo = GetAlgo();
    gp_Lin ray(G4toOCCT(p), gp_Dir(v.x(), v.y(), v.z()));
    algo->intersector->Perform(ray, -kOcctTolerance, kInfinity);

    if (!algo->intersector->IsDone() || algo->intersector->NbPnt() == 0) return 0.0;

    const G4double tol = NavigationTolerance();
    G4double minDist = kInfinity;
    int minIdx = -1;
    for (int i = 1; i <= algo->intersector->NbPnt(); ++i) {
        const G4double dist = algo->intersector->WParameter(i);
        if (algo->intersector->Transition(i) != IntCurveSurface_Out) continue;

        if (std::abs(dist) <= tol) {
            if (calcNorm && validNorm && n) {
                *n = GetNormalAtIntersection(i, algo);
                *validNorm = true;
            }
            return 0.0;
        }
        if (dist > tol && dist < minDist) { minDist = dist; minIdx = i; }
    }

    if (minIdx != -1) {
        if (calcNorm && validNorm && n) {
            *n = GetNormalAtIntersection(minIdx, algo);
            *validNorm = true;
        }
        return minDist;
    }
    return 0.0;
}

G4double G4StepSolid::DistanceToOut(const G4ThreeVector& p) const {
    const EInside state = Inside(p);
    if (state == kOutside) return 0.0;

    const G4double dist = DistanceToShape(fShape, p);
    return std::isfinite(dist) ? dist : 0.0;
}

G4ThreeVector G4StepSolid::SurfaceNormal(const G4ThreeVector& p) const {
    BRepExtrema_DistShapeShape extrema;
    extrema.LoadS1(BRepBuilderAPI_MakeVertex(G4toOCCT(p)).Shape());
    extrema.LoadS2(fShape);
    extrema.Perform();
    if (!extrema.IsDone() || extrema.NbSolution() == 0) return G4ThreeVector(0,0,1);

    const gp_Pnt p_surf_loc = extrema.PointOnShape2(1);
    TopoDS_Shape support = extrema.SupportOnShape2(1);
    if (support.ShapeType() == TopAbs_FACE) {
        TopoDS_Face face = TopoDS::Face(support);
        BRepAdaptor_Surface surf(face);
        GeomAPI_ProjectPointOnSurf proj(p_surf_loc, surf.Surface().Surface());
        Standard_Real u, v;
        proj.LowerDistanceParameters(u, v);
        gp_Pnt pc; gp_Vec d1u, d1v;
        surf.D1(u, v, pc, d1u, d1v);
        gp_Vec n = d1u.Crossed(d1v);
        if (n.SquareMagnitude() > kOcctTolerance * kOcctTolerance) {
            if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
            return OCCTtoG4(gp_Dir(n));
        }
    }

    G4ThreeVector fallback = p - OCCTtoG4(p_surf_loc);
    return (fallback.mag2() > 0) ? fallback.unit() : G4ThreeVector(0,0,1);
}

// Helper using the cached algorithm object passed in by the caller.
G4ThreeVector G4StepSolid::GetNormalAtIntersection(int index, const AlgoCache* algo) const {
    const TopoDS_Face& face = algo->intersector->Face(index);
    const double u = algo->intersector->UParameter(index);
    const double v = algo->intersector->VParameter(index);
    BRepAdaptor_Surface surf(face);
    gp_Pnt p; gp_Vec d1u, d1v;
    surf.D1(u, v, p, d1u, d1v);
    gp_Vec n = d1u.Crossed(d1v);
    if (n.SquareMagnitude() <= kOcctTolerance * kOcctTolerance) {
        return G4ThreeVector(0,0,1);
    }
    if (face.Orientation() == TopAbs_REVERSED) n.Reverse();
    return OCCTtoG4(gp_Dir(n));
}

// ====================================================================
// Visualization helpers (must be protected by a lock).
// ====================================================================

void G4StepSolid::DescribeYourselfTo(G4VGraphicsScene& scene) const {
    scene.AddSolid(*this);
}

G4Polyhedron* G4StepSolid::GetPolyhedron() const {
    // Prevent multiple threads from seeing a null pointer and building the polyhedron at the same time.
    G4AutoLock l(&polyhedronMutex);
    
    if (!fpPolyhedron || fRebuildPolyhedron ||
        fpPolyhedron->GetNumberOfRotationStepsAtTimeOfCreation() !=
        fpPolyhedron->GetNumberOfRotationSteps()) 
    {
        delete fpPolyhedron;
        fpPolyhedron = CreatePolyhedron();
        fRebuildPolyhedron = false;
    }
    return fpPolyhedron;
}

G4Polyhedron* G4StepSolid::CreatePolyhedron() const {
    // Meshing is read-only with respect to fShape and should be thread-safe, but it is expensive.
    // GetPolyhedron() already holds the lock, so it is safe to run here.
    
    G4cout << ">>> Generating Mesh for " << GetName() << " ..." << G4endl;

    try {
        BRepMesh_IncrementalMesh meshGen(fShape, 1.0); // Mesh with 1.0 mm deflection.
    } catch (...) {}
    
    G4int nVertices = 0;
    G4int nFacets = 0;
    TopExp_Explorer ex(fShape, TopAbs_FACE);
    for (; ex.More(); ex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (!tri.IsNull()) {
            nVertices += tri->NbNodes();
            nFacets   += tri->NbTriangles();
        }
    }

    if (nFacets == 0) {
        return new G4PolyhedronBox(10, 10, 10); // Fallback box if no facets are available.
    }

    G4PolyhedronArbitrary* poly = new G4PolyhedronArbitrary(nVertices, nFacets);
    G4int currentVertexOffset = 0;
    ex.ReInit();
    for (; ex.More(); ex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;
        gp_Trsf trsf = loc.Transformation();
        for (int i = 1; i <= tri->NbNodes(); i++) {
            gp_Pnt p = tri->Node(i); p.Transform(trsf);
            poly->AddVertex(G4ThreeVector(p.X(), p.Y(), p.Z()));
        }
        for (int i = 1; i <= tri->NbTriangles(); i++) {
            const Poly_Triangle& triangle = tri->Triangle(i);
            int n1, n2, n3; triangle.Get(n1, n2, n3);
            int i1 = currentVertexOffset + n1;
            int i2 = currentVertexOffset + n2;
            int i3 = currentVertexOffset + n3;
            if (face.Orientation() == TopAbs_REVERSED) poly->AddFacet(i1, i3, i2);
            else poly->AddFacet(i1, i2, i3);
        }
        currentVertexOffset += tri->NbNodes();
    }
    return poly;
}

void G4StepSolid::BoundingLimits(G4ThreeVector& pMin, G4ThreeVector& pMax) const {
    pMin.set(fXmin, fYmin, fZmin);
    pMax.set(fXmax, fYmax, fZmax);
}

G4bool G4StepSolid::CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit,
                                    const G4AffineTransform& pTransform, G4double& pMin, G4double& pMax) const {
    G4ThreeVector bmin(fXmin, fYmin, fZmin), bmax(fXmax, fYmax, fZmax);
    G4BoundingEnvelope bbox(bmin,bmax);
    return bbox.CalculateExtent(pAxis,pVoxelLimit,pTransform,pMin,pMax);
}

G4ThreeVector G4StepSolid::GetPointOnSurface() const {
    std::vector<G4ThreeVector> surfacePoints;
    constexpr Standard_Real uvTol = 1e-7;

    TopExp_Explorer ex(fShape, TopAbs_FACE);
    for (; ex.More(); ex.Next()) {
        const TopoDS_Face& face = TopoDS::Face(ex.Current());
        Standard_Real uMin, uMax, vMin, vMax;
        BRepTools::UVBounds(face, uMin, uMax, vMin, vMax);
        if (uMin >= uMax || vMin >= vMax) {
            continue;
        }

        BRepAdaptor_Surface surf(face);
        const Standard_Real du = uMax - uMin;
        const Standard_Real dv = vMax - vMin;
        const gp_Pnt2d candidates[] = {
            gp_Pnt2d(0.5 * (uMin + uMax), 0.5 * (vMin + vMax)),
            gp_Pnt2d(uMin + 0.25 * du, vMin + 0.25 * dv),
            gp_Pnt2d(uMin + 0.75 * du, vMin + 0.25 * dv),
            gp_Pnt2d(uMin + 0.25 * du, vMin + 0.75 * dv),
            gp_Pnt2d(uMin + 0.75 * du, vMin + 0.75 * dv)
        };

        for (const auto& uv : candidates) {
            BRepClass_FaceClassifier faceClassifier(face, uv, uvTol);
            const TopAbs_State state = faceClassifier.State();
            if (state != TopAbs_IN && state != TopAbs_ON) {
                continue;
            }

            const gp_Pnt p = surf.Value(uv.X(), uv.Y());
            surfacePoints.emplace_back(p.X(), p.Y(), p.Z());
        }
    }

    if (!surfacePoints.empty()) {
        const auto index =
            static_cast<std::size_t>(G4RandFlat::shootInt(static_cast<G4int>(surfacePoints.size())));
        return surfacePoints[index];
    }

    G4ThreeVector center((fXmax+fXmin)*0.5, (fYmax+fYmin)*0.5, (fZmax+fZmin)*0.5);
    return center;
}

std::ostream& G4StepSolid::StreamInfo(std::ostream& os) const {
    os << "G4StepSolid: " << GetName();
    return os;
}