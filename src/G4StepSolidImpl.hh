#ifndef G4StepSolidImpl_HH
#define G4StepSolidImpl_HH

// Internal implementation header for G4StepSolid (pImpl, P4).
//
// All of G4StepSolid's private state, internal types, and worker methods live here,
// behind an opaque std::unique_ptr<G4StepSolidImpl> in the public class. This keeps
// the public header (include/G4StepSolid.hh) free of OpenCASCADE and of internal
// struct definitions, so a consumer #include "G4StepSolid.hh" pulls in only the
// G4VSolid interface, and adding/removing internal members never touches the public
// header or the ABI. NOT installed; used only by G4StepSolid.cc and the algorithm .cc.

#include "G4VSolid.hh"          // EInside, G4ThreeVector, G4double, G4String, ...
#include "G4Cache.hh"

// OCCT headers (the heavy set that used to live in the public header).
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <IntCurveSurface_TransitionOnCurve.hxx>
#include <TopTools_DataMapOfShapeInteger.hxx>
#include <Bnd_Box.hxx>
#include <gp_Pnt.hxx>
#include <gp_Ax3.hxx>

class BRepExtrema_DistShapeShape;
class G4StepSolid;

#include <vector>
#include <mutex>
#include <memory>
#include <fstream>
#include <chrono>
#include <cstdint>

// Ray-cast hit: one surface crossing produced by a single Perform() call or analytic formula.
struct G4StepRayHit {
    G4double                          w;
    IntCurveSurface_TransitionOnCurve trans;
    TopoDS_Face                       face;
    G4double                          u, v;
    G4StepRayHit(G4double w_, IntCurveSurface_TransitionOnCurve t,
                 const TopoDS_Face& f, G4double u_, G4double v_)
        : w(w_), trans(t), face(f), u(u_), v(v_) {}
};

// Grouped hits within distance tolerance: one potential boundary crossing.
struct G4StepHitGroup {
    G4double    dist;
    int         nIn     = 0;
    int         nOut    = 0;
    TopoDS_Face outFace;
    G4double    outU    = 0;
    G4double    outV    = 0;
    TopoDS_Face anyFace;
};

// Precomputed per-face geometry (read-only at runtime, lock-free).
struct G4StepFaceEntry {
    TopoDS_Face face;
    Bnd_Box     bbox;
    G4double u0, u1, v0, v1;

    enum class SurfType : uint8_t { Other=0, Plane=1, Cylinder=2, Sphere=3, Cone=4, Torus=5 };
    SurfType surfType  = SurfType::Other;

    gp_Ax3   pos;           // coordinate system (shared by all analytic types)
    G4double cylR = 0;      // cylinder radius (Cylinder only)
    G4double sphR = 0;      // sphere radius   (Sphere only)
    G4double coneHalfAngle = 0;  // cone semi-angle (rad), Cone only
    G4double coneRefR      = 0;  // cone radius at pos.Location(), Cone only
    G4double torusMajorR   = 0;  // torus major radius R, Torus only
    G4double torusMinorR   = 0;  // torus minor (tube) radius r, Torus only

    bool isRectPlane = false;
    bool analyticTrimSafe = false;
};

struct G4StepNearestFaceResult {
    G4double dist   = kInfinity;
    int      idx    = -1;
    gp_Pnt   pOnFace;
};

// Per-thread accumulated timing (one per thread per solid, lock-free accumulation).
struct G4StepTimingStats {
    int64_t insideNs    = 0, insideCount    = 0;
    int64_t dtiVecNs    = 0, dtiVecCount    = 0;
    int64_t dtiScalNs   = 0, dtiScalCount   = 0;
    int64_t dtoVecNs    = 0, dtoVecCount    = 0;
    int64_t dtoScalNs   = 0, dtoScalCount   = 0;
    int64_t normalNs    = 0, normalCount     = 0;

    int64_t rayFacesVisited  = 0;
    int64_t rayPerformCount  = 0;
    int64_t rayHitsTotal     = 0;
    int64_t rayAnalyticCount = 0;
    int64_t rayFallbackCount = 0;
    int64_t nfFacesVisited   = 0;
    int64_t nfExtremaCount   = 0;
    int64_t nfAnalyticCount  = 0;
    int64_t nfFallbackCount  = 0;
    int64_t insideAnalyticCount = 0;
    int64_t insideFallbackCount = 0;
};

// ====================================================================
// G4StepSolidImpl — all of G4StepSolid's private state + worker methods.
// The public G4StepSolid forwards its G4VSolid overrides here. fOwner is a
// back-pointer to the public object (for GetName()); fVerboseLevel/fTimingEnabled
// live here so the moved method bodies reference them unchanged.
// ====================================================================
class G4StepSolidImpl {
public:
    G4StepSolidImpl(const TopoDS_Shape& stepShape, G4double tolerance, G4StepSolid* owner);
    ~G4StepSolidImpl();
    // Deep-copy geometry for a copied G4StepSolid; fresh (empty) per-thread cache.
    G4StepSolidImpl(const G4StepSolidImpl& rhs, G4StepSolid* owner);

    G4StepSolidImpl(const G4StepSolidImpl&) = delete;
    G4StepSolidImpl& operator=(const G4StepSolidImpl&) = delete;

    using RayHit           = G4StepRayHit;
    using HitGroup         = G4StepHitGroup;
    using FaceEntry        = G4StepFaceEntry;
    using NearestFaceResult= G4StepNearestFaceResult;
    using TimingStats      = G4StepTimingStats;

    // Name: single source of truth is the public object (fOwner). Returns BY VALUE
    // (G4VSolid::GetName() returns by value — a reference would dangle).
    G4String GetName() const;

    // --- public-facing operations (G4StepSolid forwards to these) ---
    EInside Inside(const G4ThreeVector& p) const;
    G4double DistanceToIn(const G4ThreeVector& p, const G4ThreeVector& v) const;
    G4double DistanceToIn(const G4ThreeVector& p) const;
    G4double DistanceToOut(const G4ThreeVector& p, const G4ThreeVector& v,
                           const G4bool calcNorm, G4bool* validNorm, G4ThreeVector* n) const;
    G4double DistanceToOut(const G4ThreeVector& p) const;
    G4ThreeVector SurfaceNormal(const G4ThreeVector& p) const;
    void BoundingLimits(G4ThreeVector& pMin, G4ThreeVector& pMax) const;
    G4bool CalculateExtent(const EAxis pAxis, const G4VoxelLimits& pVoxelLimit,
                           const G4AffineTransform& pTransform,
                           G4double& pMin, G4double& pMax) const;
    std::ostream& StreamInfo(std::ostream& os) const;
    void DescribeYourselfTo(G4VGraphicsScene& scene) const;
    G4Polyhedron* CreatePolyhedron() const;
    G4Polyhedron* GetPolyhedron() const;
    G4ThreeVector GetPointOnSurface() const;

    void SetTimingEnabled(G4bool enable);
    void PrintTimingReport() const;
    static void PrintAllTimingReports();

    G4StepSolid* fOwner = nullptr;
    G4int  fVerboseLevel  = 0;
    G4bool fTimingEnabled = false;

private:
    TopoDS_Shape fShape;
    G4double fXmin, fXmax, fYmin, fYmax, fZmin, fZmax;
    G4double fTolerance;
    G4double kCarTolerance;   // Geant4 surface tolerance (was inherited from G4VSolid)

    static void GroupHits(std::vector<RayHit>& hits, std::vector<HitGroup>& out, G4double tol);

    std::vector<FaceEntry> fFaces;
    std::vector<G4double>  fCumulativeArea;

    // Thread-local OCCT algorithm objects
    struct AlgoCache {
        std::vector<std::unique_ptr<BRepClass3d_SolidClassifier>> solidClassifiers;
        std::vector<Bnd_Box>                                       solidBoxes;
        std::vector<IntCurvesFace_Intersector*>                    faceIntersectors;
        std::vector<BRepExtrema_DistShapeShape*>                   faceExtrema;

        std::vector<G4double>          solidTol;
        TopTools_DataMapOfShapeInteger faceSolidMap;
        G4double                       requestedTol = 1e-7;
        G4double                       occtTol      = 1e-7;
        bool                           perSolidTol  = true;

        std::vector<std::vector<int>>  solidFaceIdx;
        std::vector<char>              analyticInsideOK;

        struct UVTrim {
            bool   valid = false;
            bool   periodic = false;
            double u0 = 0, u1 = 0;
            double v0 = 0, v1 = 0;
            enum class Mode : uint8_t { Band = 0, Poly = 1 };
            Mode   mode = Mode::Band;
            double du = 0;
            int    NB = 0;
            std::vector<double> top, bot;
            std::vector<std::vector<double>> holeU, holeV;
            std::vector<std::vector<double>> outU, outV;
            double band = 0;
        };
        std::vector<UVTrim>            faceUVTrim;
        int64_t                       uvTrimAccept = 0;
        int64_t                       uvTrimBandFallback = 0;

        std::vector<std::vector<G4ThreeVector>> solidParityDirs;
        std::vector<RayHit>            scratchInsideHits;
        std::vector<std::pair<double,int>> scratchFaceOrder;

        TimingStats            timing;
        const G4StepSolidImpl* owner = nullptr;

        std::vector<RayHit>   scratchHits;
        std::vector<HitGroup> scratchGroups;

        AlgoCache(const TopoDS_Shape& shape, G4double tolerance,
                  const G4StepSolidImpl* ownerImpl);
        ~AlgoCache();

        int      FaceToSolid(const TopoDS_Face& f) const;
        G4double SolidBand(int si) const {
            if (perSolidTol && si >= 0 && si < (int)solidTol.size()) return solidTol[si];
            return requestedTol;
        }
    };

    mutable G4Cache<AlgoCache*> fAlgoCache;

    // Process-wide registries (shared across all solids). Static members so both
    // translation units (G4StepSolid.cc forwarders / this Impl .cc) can reach them.
    static std::vector<AlgoCache*>          sAllCaches;
    static std::mutex                        sAllCachesMutex;
    static std::vector<const G4StepSolidImpl*> sTimedSolids;
    static std::mutex                        sTimedSolidsMutex;
    static std::once_flag                    sAtexitFlag;

    mutable G4Polyhedron* fpPolyhedron      = nullptr;
    mutable G4bool        fRebuildPolyhedron = false;

    // --- Query recorder (env-gated via G4StepConfig) ---
    enum class QueryType { Inside, DistToIn, DistToOut };
    static bool           sRecordEnabled;
    static int            sRecordOnlyEvent;
    static bool           sInsideForceOCCT;
    static std::ofstream  sRecordStream;
    static std::mutex     sRecordMutex;
    static std::once_flag sRecordInitFlag;
    static long           sRecordSeq;
    static void InitRecorderOnce();
    void RecordQuery(QueryType type, const G4ThreeVector& p,
                     const G4ThreeVector& v, int insideResult,
                     G4double dist, int insideAtP) const;

    void CalcBBox();
    void BuildFaceWeights();
    AlgoCache* GetAlgo() const;
    NearestFaceResult NearestFace(const gp_Pnt& pt) const;
    bool IntersectFaceForParity(const FaceEntry& fe, const G4ThreeVector& p,
                                const G4ThreeVector& dir, G4double octTol,
                                std::vector<RayHit>& hits) const;
    AlgoCache::UVTrim BuildUVTrim(const FaceEntry& fe, G4double octTol) const;
    static int ClassifyUVTrim(const AlgoCache::UVTrim& t, double u, double v);
    EInside InsideSolidAnalytic(int si, const G4ThreeVector& p,
                                AlgoCache* algo, bool& ok) const;
    G4ThreeVector GetNormalAtIntersection(const TopoDS_Face& face, G4double u, G4double v) const;

    enum class BoundaryKind { Entry, Exit };
    G4double RayCastToBoundary(const G4ThreeVector& p, const G4ThreeVector& v,
                               BoundaryKind kind,
                               G4bool calcNorm = false,
                               G4bool* validNorm = nullptr,
                               G4ThreeVector* n = nullptr) const;

    using Clock = std::chrono::high_resolution_clock;
};

#endif // G4StepSolidImpl_HH
