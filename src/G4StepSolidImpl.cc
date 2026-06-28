#include "G4StepSolid.hh"

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
    : owner(ownerSolid)
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
        if (fTimingEnabled) ++algo->timing.nfFacesVisited;
        BRepExtrema_DistShapeShape& extrema = *algo->faceExtrema[i];
        extrema.LoadS1(vtxShape);
        extrema.Perform();
        if (fTimingEnabled) ++algo->timing.nfExtremaCount;
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
        if (fTimingEnabled) ++algo->timing.rayFacesVisited;

        IntCurvesFace_Intersector& inter = *algo->faceIntersectors[fi];
        inter.Perform(ray, -octTol, kInfinity);
        if (fTimingEnabled) ++algo->timing.rayPerformCount;
        for (int j = 1; j <= inter.NbPnt(); ++j) {
            rawHits.emplace_back(inter.WParameter(j), inter.Transition(j),
                                 fe.face, inter.UParameter(j), inter.VParameter(j));
        }
        if (fTimingEnabled) algo->timing.rayHitsTotal += inter.NbPnt();
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
