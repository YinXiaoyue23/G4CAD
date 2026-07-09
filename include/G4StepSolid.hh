#ifndef G4StepSolid_HH
#define G4StepSolid_HH

#include "G4VSolid.hh"
#include <memory>

class TopoDS_Shape;       // ctor takes it by const-reference (forward decl suffices)
class G4StepSolidImpl;    // pImpl — all private state/OCCT lives in src/G4StepSolidImpl.hh

// A G4VSolid backed directly by an OpenCASCADE STEP B-rep shape.
//
// All implementation detail (OCCT objects, per-thread caches, analytic geometry,
// timing/recorder) lives behind an opaque std::unique_ptr<G4StepSolidImpl>, so this
// public header pulls in no OpenCASCADE headers and internal changes never touch the
// ABI or force consumers to recompile against OCCT.
class G4StepSolid : public G4VSolid {
public:
    // tolerance controls OCCT algorithm precision (shape coordinate units, typically mm)
    G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape,
                G4double tolerance = 1e-7);
    ~G4StepSolid() override;

    G4StepSolid(const G4StepSolid& rhs);
    G4StepSolid& operator=(const G4StepSolid& rhs);

    // --- Verbosity (Geant4 convention: 0 silent, 1 construction, 2 per-call trace) ---
    void   SetVerboseLevel(G4int level);
    G4int  GetVerboseLevel() const;

    // --- Per-call timing (disabled by default; per-function summary printed at exit) ---
    void   SetTimingEnabled(G4bool enable);
    G4bool GetTimingEnabled() const;
    void   PrintTimingReport() const;
    static void PrintAllTimingReports();

    // --- Geant4 core geometry interface ---
    EInside Inside(const G4ThreeVector& p) const override;
    G4double DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const override;
    G4double DistanceToIn(const G4ThreeVector& p) const override;
    G4double DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                           const G4bool calcNorm = false,
                           G4bool* validNorm = nullptr,
                           G4ThreeVector* n = nullptr) const override;
    G4double DistanceToOut(const G4ThreeVector& p) const override;
    G4ThreeVector SurfaceNormal(const G4ThreeVector& p) const override;
    void BoundingLimits(G4ThreeVector& pMin, G4ThreeVector& pMax) const override;
    G4bool CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit,
                           const G4AffineTransform& pTransform,
                           G4double& pMin, G4double& pMax) const override;

    G4GeometryType GetEntityType() const override { return "G4StepSolid"; }
    std::ostream& StreamInfo(std::ostream& os) const override;
    void DescribeYourselfTo(G4VGraphicsScene& scene) const override;

    G4Polyhedron* CreatePolyhedron() const override;
    G4Polyhedron* GetPolyhedron() const override;
    G4ThreeVector GetPointOnSurface() const override;

private:
    std::unique_ptr<G4StepSolidImpl> fImpl;
};

#endif
