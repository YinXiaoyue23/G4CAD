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
#include <IntCurvesFace_Intersector.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Face.hxx>

#include <vector>
#include <mutex>
#include <memory>
#include <fstream>

class G4StepSolid : public G4VSolid {
public:
    // tolerance controls OCCT algorithm precision (same units as shape coordinates, typically mm)
    G4StepSolid(const G4String& name, const TopoDS_Shape& stepShape,
                G4double tolerance = 1e-7);
    virtual ~G4StepSolid();

    G4StepSolid(const G4StepSolid& rhs);
    G4StepSolid& operator=(const G4StepSolid& rhs);

    // --- Verbosity (Geant4 convention) ---
    // 0 = silent (default)
    // 1 = construction messages (mesh generation, load progress)
    // 2 = per-call geometry tracing (Inside, DistanceToIn/Out, ...)
    void   SetVerboseLevel(G4int level) { fVerboseLevel = level; }
    G4int  GetVerboseLevel() const      { return fVerboseLevel; }

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
    G4int  fVerboseLevel  = 0;

    // Read-only geometry data, shared across all threads
    TopoDS_Shape fShape;
    G4double fXmin, fXmax, fYmin, fYmax, fZmin, fZmax;
    G4double fTolerance;

    // For GetPointOnSurface: precomputed at construction, read-only at runtime, lock-free
    struct FaceEntry {
        TopoDS_Face face;
        Bnd_Box     bbox;
        G4double u0, u1, v0, v1;
    };

    struct NearestFaceResult {
        G4double dist   = kInfinity;
        int      idx    = -1;
        gp_Pnt   pOnFace;
    };
    std::vector<FaceEntry> fFaces;
    std::vector<G4double>  fCumulativeArea; // prefix sum of face areas

    // Thread-local OCCT algorithm objects
    struct AlgoCache {
        // Per-solid classifiers: point is inside the assembly if inside ANY solid.
        // Using individual solids avoids BRepClass3d_SolidClassifier's known misclassification
        // of points near assembly joints when applied to a compound shape.
        std::vector<std::unique_ptr<BRepClass3d_SolidClassifier>> solidClassifiers;
        std::vector<Bnd_Box>                                       solidBoxes;
        std::vector<IntCurvesFace_Intersector*>                    faceIntersectors;

        // --- Per-solid tolerance ---
        // Each solid carries geometric noise σ_i (max BRep tol over its faces/edges/vertices).
        // A clean solid gets a tight surface band; a noisy one (RP.step #5/#6) gets 2σ_i so
        // Inside()/DistanceToOut() stay self-consistent.
        //   solidTol[i] = max(requested, 2·σ_i)   (per-solid surface band)
        //   occtTol     = max over solidTol       (global OCCT search/grouping window)
        // faceSolidMap maps each face (by identity) to its owning solid index, so a ray hit
        // looks up its band. perSolidTol=false (env G4CAD_PERSOLID_TOL=0) → global single tol.
        std::vector<G4double>          solidTol;
        TopTools_DataMapOfShapeInteger faceSolidMap;
        G4double                       requestedTol = 1e-7;
        G4double                       occtTol      = 1e-7;
        bool                           perSolidTol  = true;

        AlgoCache(const TopoDS_Shape& shape, G4double tolerance,
                  const G4StepSolid* ownerSolid);
        ~AlgoCache();

        int      FaceToSolid(const TopoDS_Face& f) const;  // -1 if not found
        G4double SolidBand(int si) const {                 // surface band for a solid (or global)
            if (perSolidTol && si >= 0 && si < (int)solidTol.size()) return solidTol[si];
            return requestedTol;
        }
    };

    mutable G4Cache<AlgoCache*> fAlgoCache;

    // Global registry for destroying all per-thread AlgoCaches on object destruction
    static std::vector<AlgoCache*> sAllCaches;
    static std::mutex               sAllCachesMutex;

    // --- Query recorder (env-var gated, zero cost when off) ---
    // Enable with  G4CAD_RECORD=/path/queries.csv  (optionally G4CAD_RECORD_EVENT=N).
    // Records every Inside / DistanceToIn(p,v) / DistanceToOut(p,v) for FreeCAD
    // visualisation of navigation queries. Event tag = G4Run::GetRunID().
    enum class QueryType { Inside, DistToIn, DistToOut };
    static bool           sRecordEnabled;
    static int            sRecordOnlyEvent;
    static std::ofstream  sRecordStream;
    static std::mutex     sRecordMutex;
    static std::once_flag sRecordInitFlag;
    static long           sRecordSeq;
    static void InitRecorderOnce();
    void RecordQuery(QueryType type, const G4ThreeVector& p,
                     const G4ThreeVector& v, int insideResult,
                     G4double dist, int insideAtP) const;

    // Visualisation cache (mutex-protected)
    mutable G4Polyhedron* fpPolyhedron      = nullptr;
    mutable G4bool        fRebuildPolyhedron = false;

    void CalcBBox();
    void BuildFaceWeights();
    AlgoCache* GetAlgo() const;
    NearestFaceResult NearestFace(const gp_Pnt& pt) const;
    G4ThreeVector GetNormalAtIntersection(const TopoDS_Face& face, G4double u, G4double v) const;

    // Unified ray-boundary helper used by both DistanceToIn(p,v) and DistanceToOut(p,v).
    enum class BoundaryKind { Entry, Exit };
    G4double RayCastToBoundary(const G4ThreeVector& p, const G4ThreeVector& v,
                               BoundaryKind kind,
                               G4bool calcNorm = false,
                               G4bool* validNorm = nullptr,
                               G4ThreeVector* n = nullptr) const;
};

#endif
