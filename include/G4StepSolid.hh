#ifndef G4StepSolid_HH
#define G4StepSolid_HH

#include "G4VSolid.hh"
#include "G4SystemOfUnits.hh"
#include "globals.hh"
#include "G4Cache.hh"

// OCCT Headers
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>

#include <vector>
#include <mutex>

// Compile with -DG4CAD_DEBUG_TRACE to enable per-call geometry tracing.
#ifdef G4CAD_DEBUG_TRACE
#  define G4CAD_TRACE(...) G4cout << __VA_ARGS__ << G4endl
#else
#  define G4CAD_TRACE(...) do {} while(0)
#endif

class G4StepSolid : public G4VSolid {
public:
    // tolerance controls OCCT algorithm precision (same units as shape coordinates, typically mm)
    G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape,
                G4double tolerance = 1e-7);
    virtual ~G4StepSolid();

    G4StepSolid(const G4StepSolid& rhs);
    G4StepSolid& operator=(const G4StepSolid& rhs);

    // --- Geant4 core geometry interface ---
    virtual EInside Inside(const G4ThreeVector& p) const override;
    virtual G4double DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const override;
    virtual G4double DistanceToIn(const G4ThreeVector& p) const override;
    virtual G4double DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                                   const G4bool calcNorm = false,
                                   G4bool* validNorm = nullptr,
                                   G4ThreeVector* n = nullptr) const override;
    virtual G4double DistanceToOut(const G4ThreeVector& p) const override;
    virtual G4ThreeVector SurfaceNormal(const G4ThreeVector& p) const override;
    virtual void BoundingLimits(G4ThreeVector& pMin, G4ThreeVector& pMax) const override;
    virtual G4bool CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit,
                                   const G4AffineTransform& pTransform,
                                   G4double& pMin, G4double& pMax) const override;

    virtual G4GeometryType GetEntityType() const override { return "G4StepSolid"; }
    virtual std::ostream& StreamInfo(std::ostream& os) const override;
    virtual void DescribeYourselfTo(G4VGraphicsScene& scene) const override;

    virtual G4Polyhedron* CreatePolyhedron() const override;
    virtual G4Polyhedron* GetPolyhedron() const override;
    virtual G4ThreeVector GetPointOnSurface() const override;

private:
    // Read-only geometry data, shared across all threads
    TopoDS_Shape fShape;
    G4double fXmin, fXmax, fYmin, fYmax, fZmin, fZmax;
    G4double fTolerance;

    // For GetPointOnSurface: precomputed at construction, read-only at runtime, lock-free
    struct FaceEntry {
        TopoDS_Face face;
        G4double u0, u1, v0, v1;
    };
    std::vector<FaceEntry> fFaces;
    std::vector<G4double>  fCumulativeArea; // prefix sum of face areas

    // Thread-local OCCT algorithm objects
    struct AlgoCache {
        BRepClass3d_SolidClassifier*    classifier  = nullptr;
        IntCurvesFace_ShapeIntersector* intersector = nullptr;

        AlgoCache(const TopoDS_Shape& shape, G4double tolerance);
        ~AlgoCache();
    };

    mutable G4Cache<AlgoCache*> fAlgoCache;

    // Global registry for destroying all per-thread AlgoCaches on object destruction
    static std::vector<AlgoCache*> sAllCaches;
    static std::mutex               sAllCachesMutex;

    // Visualisation cache (mutex-protected)
    mutable G4Polyhedron* fpPolyhedron      = nullptr;
    mutable G4bool        fRebuildPolyhedron = false;

    void CalcBBox();
    void BuildFaceWeights();
    AlgoCache* GetAlgo() const;
    G4ThreeVector GetNormalAtIntersection(int index, const AlgoCache* algo) const;

    // Unified ray-boundary helper used by both DistanceToIn(p,v) and DistanceToOut(p,v).
    enum class BoundaryKind { Entry, Exit };
    G4double RayCastToBoundary(const G4ThreeVector& p, const G4ThreeVector& v,
                               BoundaryKind kind,
                               G4bool calcNorm = false,
                               G4bool* validNorm = nullptr,
                               G4ThreeVector* n = nullptr) const;
};

#endif
