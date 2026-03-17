#ifndef G4StepSolid_HH
#define G4StepSolid_HH

#include "G4VSolid.hh"
#include "G4SystemOfUnits.hh"
#include "globals.hh"
#include "G4Cache.hh"

// OCCT Headers
#include <TopoDS_Shape.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>

class G4StepSolid : public G4VSolid {
public:
    G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape);
    virtual ~G4StepSolid();

    G4StepSolid(const G4StepSolid& rhs);
    G4StepSolid& operator=(const G4StepSolid& rhs);

    // Geant4 geometry interface
    virtual EInside Inside(const G4ThreeVector& p) const override;
    virtual G4double DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const override;
    virtual G4double DistanceToIn(const G4ThreeVector& p) const override;
    virtual G4double DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                                   const G4bool calcNorm=false,
                                   G4bool *validNorm=0, G4ThreeVector *n=0) const override;
    virtual G4double DistanceToOut(const G4ThreeVector& p) const override;
    virtual G4ThreeVector SurfaceNormal(const G4ThreeVector& p) const override;
    virtual G4ThreeVector GetPointOnSurface() const override;

    virtual G4GeometryType GetEntityType() const override { return "G4StepSolid"; }
    virtual std::ostream& StreamInfo(std::ostream& os) const override;
    virtual void BoundingLimits(G4ThreeVector& pMin, G4ThreeVector& pMax) const override;
    virtual G4bool CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit, 
                                   const G4AffineTransform& pTransform,
                                   G4double& pMin, G4double& pMax) const override;

    virtual void DescribeYourselfTo(G4VGraphicsScene& scene) const override;
    virtual G4Polyhedron* CreatePolyhedron() const override;
    virtual G4Polyhedron* GetPolyhedron() const override;

private:
    TopoDS_Shape fShape;
    G4double fXmin, fXmax, fYmin, fYmax, fZmin, fZmax;

    // Thread-local algorithm cache
    struct AlgoCache {
        BRepClass3d_SolidClassifier* classifier = nullptr;
        IntCurvesFace_ShapeIntersector* intersector = nullptr;
        AlgoCache(const TopoDS_Shape& shape);
        ~AlgoCache();
    };
    mutable G4Cache<AlgoCache*> fAlgoCache;

    // Visualization cache
    mutable G4Polyhedron* fpPolyhedron = nullptr;
    mutable G4bool fRebuildPolyhedron = false;

    void CalcBBox();
    AlgoCache* GetAlgo() const; // Lazily create and return the algorithm cache for the current thread.
    G4ThreeVector GetNormalAtIntersection(int index, const AlgoCache* algo) const;
};

#endif