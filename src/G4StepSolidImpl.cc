#include "G4StepSolid.hh"
#include "G4StepConfig.hh"

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
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
// CT-TRIM: UV trim discretisation + ground-truth 2D classifier for self-validation
#include <BRepTools_WireExplorer.hxx>
#include <BRepAdaptor_Curve2d.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Edge.hxx>
#include <random>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <mutex>
#include <chrono>

namespace {
    // Coordinate converters — redefined here to avoid a shared private header
    // for three trivial one-liners.
    inline gp_Pnt        G4toOCCT(const G4ThreeVector& v) { return gp_Pnt(v.x(), v.y(), v.z()); }
    inline G4ThreeVector OCCTtoG4(const gp_Dir& d)        { return G4ThreeVector(d.X(), d.Y(), d.Z()); }

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

    // ================================================================
    // Analytic ray-surface intersection cores (P1: single shared impl)
    // ----------------------------------------------------------------
    // These reproduce the EXACT math previously inlined in both
    // RayCastToBoundary and IntersectFaceForParity, so Inside() and
    // DistanceToIn/Out() cannot drift apart. They are pure geometry:
    // root solve + t>=-tol + r>0 + transition (incl. tangent degeneracy)
    // + (u,v). They do NOT touch timing counters and do NOT apply the
    // CT-TRIM band classification — those stay in the callers. The ray is
    // passed as origin (o*) + direction (d*) so both the RayCast `v`
    // (possibly non-unit) and the parity `dir` (unit) callers work; A/nv
    // are always computed from the passed components (never hardcoded).
    struct RaySurfCand {
        G4double                          t;
        IntCurveSurface_TransitionOnCurve trans;
        G4double                          u, v;
    };

    // Ray vs rectangular-trimmed plane. Returns 0 or 1. The UV-rectangle
    // trim is identical in both call sites (and there is no CT-TRIM plane
    // variant), so it is folded in here.
    [[gnu::always_inline]] inline int RayIntersectPlane(const gp_Ax3& ax,
                                 G4double u0, G4double u1, G4double v0, G4double v1,
                                 bool fwd,
                                 G4double ox, G4double oy, G4double oz,
                                 G4double dx, G4double dy, G4double dz,
                                 G4double tol, RaySurfCand out[1])
    {
        const gp_Dir& nm  = ax.Direction();
        const gp_Pnt& loc = ax.Location();
        G4double nv = nm.X()*dx + nm.Y()*dy + nm.Z()*dz;
        if (std::abs(nv) < 1e-15) return 0;
        G4double np = nm.X()*(ox-loc.X()) + nm.Y()*(oy-loc.Y()) + nm.Z()*(oz-loc.Z());
        G4double t = -np / nv;
        if (t < -tol) return 0;
        G4double hx = ox+t*dx-loc.X();
        G4double hy = oy+t*dy-loc.Y();
        G4double hz = oz+t*dz-loc.Z();
        const gp_Dir& xu = ax.XDirection();
        const gp_Dir& yv = ax.YDirection();
        G4double uf = xu.X()*hx + xu.Y()*hy + xu.Z()*hz;
        G4double vf = yv.X()*hx + yv.Y()*hy + yv.Z()*hz;
        if (uf < u0-tol || uf > u1+tol || vf < v0-tol || vf > v1+tol) return 0;
        G4double n_out_v = fwd ? nv : -nv;
        IntCurveSurface_TransitionOnCurve trans;
        if      (n_out_v < -tol) trans = IntCurveSurface_In;
        else if (n_out_v >  tol) trans = IntCurveSurface_Out;
        else                     trans = IntCurveSurface_Tangent;
        out[0] = { t, trans, uf, vf };
        return 1;
    }

    // Ray vs cylinder. Returns 0..2 candidates in root order (near root first).
    // Applies only the UNIVERSAL checks (t>=-tol, axial-v range, r>0) and the uf
    // normalization (needs u0). It deliberately does NOT apply the angular u-range
    // reject nor the CT-TRIM band — those differ per caller (rect-UV vs
    // ClassifyUVTrim) and stay in the caller. trans is computed for every returned
    // candidate; callers discard off-trim ones (a discarded candidate's trans is
    // never used, so result is identical to the old compute-after-reject order).
    [[gnu::always_inline]] inline int RayIntersectCylinder(const gp_Ax3& ax, G4double R,
                                    G4double u0, G4double v0, G4double v1,
                                    bool fwd,
                                    G4double ox, G4double oy, G4double oz,
                                    G4double dx, G4double dy, G4double dz,
                                    G4double tol, RaySurfCand out[2])
    {
        const gp_Dir& axDir = ax.Direction();
        const gp_Pnt& axLoc = ax.Location();
        G4double dpx = ox-axLoc.X(), dpy = oy-axLoc.Y(), dpz = oz-axLoc.Z();
        G4double da  = axDir.X()*dpx + axDir.Y()*dpy + axDir.Z()*dpz;
        G4double va_ = axDir.X()*dx  + axDir.Y()*dy  + axDir.Z()*dz;
        G4double ppx = dpx - da*axDir.X(), ppy = dpy - da*axDir.Y(), ppz = dpz - da*axDir.Z();
        G4double vpx = dx-va_*axDir.X(),   vpy = dy-va_*axDir.Y(),   vpz = dz-va_*axDir.Z();
        G4double A = vpx*vpx + vpy*vpy + vpz*vpz;
        if (A < 1e-30) return 0;
        G4double B    = ppx*vpx + ppy*vpy + ppz*vpz;
        G4double C    = ppx*ppx + ppy*ppy + ppz*ppz - R*R;
        G4double disc = B*B - A*C;
        if (disc < 0) return 0;
        G4double sqrtD = std::sqrt(std::max(disc, 0.0));
        bool     tang  = (sqrtD < tol * std::sqrt(A));
        const gp_Dir& xu_c = ax.XDirection();
        const gp_Dir& yv_c = ax.YDirection();
        int n = 0;
        auto tryRoot = [&](G4double t) {
            if (t < -tol) return;
            G4double v_ax = da + t * va_;
            if (v_ax < v0 - tol || v_ax > v1 + tol) return;
            G4double qx = ppx+t*vpx, qy = ppy+t*vpy, qz = ppz+t*vpz;
            G4double r  = std::sqrt(qx*qx + qy*qy + qz*qz);
            if (r < 1e-15) return;
            G4double uf = std::atan2(yv_c.X()*qx + yv_c.Y()*qy + yv_c.Z()*qz,
                                     xu_c.X()*qx + xu_c.Y()*qy + xu_c.Z()*qz);
            if (uf < u0 - M_PI) uf += 2.0*M_PI;
            IntCurveSurface_TransitionOnCurve trans;
            if (tang) {
                trans = IntCurveSurface_Tangent;
            } else {
                G4double n_dot_v = (qx*dx + qy*dy + qz*dz) / r;
                G4double n_out_v = fwd ? n_dot_v : -n_dot_v;
                if      (n_out_v < -tol) trans = IntCurveSurface_In;
                else if (n_out_v >  tol) trans = IntCurveSurface_Out;
                else                     trans = IntCurveSurface_Tangent;
            }
            out[n++] = { t, trans, uf, v_ax };
        };
        if (tang) {
            tryRoot(-B / A);
        } else {
            tryRoot((-B - sqrtD) / A);
            tryRoot((-B + sqrtD) / A);
        }
        return n;
    }

    // Ray vs sphere. Returns 0..2 candidates in root order. There is no CT-TRIM
    // sphere variant and the !fullSph UV-rectangle reject is identical at both
    // sphere sites, so it is folded in here. fullSph is derived from the UV span
    // exactly as before.
    [[gnu::always_inline]] inline int RayIntersectSphere(const gp_Ax3& pos, G4double R,
                                  G4double u0, G4double u1, G4double v0, G4double v1,
                                  bool fwd,
                                  G4double ox, G4double oy, G4double oz,
                                  G4double dx, G4double dy, G4double dz,
                                  G4double tol, RaySurfCand out[2])
    {
        const gp_Pnt& cen = pos.Location();
        G4double dpx = ox-cen.X(), dpy = oy-cen.Y(), dpz = oz-cen.Z();
        G4double A    = dx*dx + dy*dy + dz*dz;
        G4double B    = dpx*dx + dpy*dy + dpz*dz;
        G4double C    = dpx*dpx + dpy*dpy + dpz*dpz - R*R;
        G4double disc = B*B - A*C;
        if (disc < 0) return 0;
        G4double sqrtD   = std::sqrt(std::max(disc, 0.0));
        bool     tang    = (sqrtD < tol * std::sqrt(A));
        bool     fullSph = (u1-u0 >= 2.0*M_PI-tol && v1-v0 >= M_PI-tol);
        int n = 0;
        auto tryRoot = [&](G4double t) {
            if (t < -tol) return;
            G4double qx = dpx+t*dx, qy = dpy+t*dy, qz = dpz+t*dz;
            G4double r  = std::sqrt(qx*qx + qy*qy + qz*qz);
            if (r < 1e-15) return;
            G4double uf = 0.0, vf = 0.0;
            if (!fullSph) {
                const gp_Dir& xu  = pos.XDirection();
                const gp_Dir& yv_ = pos.YDirection();
                const gp_Dir& zv_ = pos.Direction();
                G4double xl = xu.X()*qx + xu.Y()*qy + xu.Z()*qz;
                G4double yl = yv_.X()*qx + yv_.Y()*qy + yv_.Z()*qz;
                G4double zl = zv_.X()*qx + zv_.Y()*qy + zv_.Z()*qz;
                uf = std::atan2(yl, xl);
                vf = std::asin(std::max(-1.0, std::min(1.0, zl/r)));
                if (uf < u0 - tol) uf += 2.0*M_PI;
                if (uf < u0 - tol || uf > u1 + tol) return;
                if (vf < v0 - tol || vf > v1 + tol) return;
            }
            IntCurveSurface_TransitionOnCurve trans;
            if (tang) {
                trans = IntCurveSurface_Tangent;
            } else {
                G4double n_dot_v = (qx*dx + qy*dy + qz*dz) / r;
                G4double n_out_v = fwd ? n_dot_v : -n_dot_v;
                if      (n_out_v < -tol) trans = IntCurveSurface_In;
                else if (n_out_v >  tol) trans = IntCurveSurface_Out;
                else                     trans = IntCurveSurface_Tangent;
            }
            out[n++] = { t, trans, uf, vf };
        };
        if (tang) {
            tryRoot(-B / A);
        } else {
            tryRoot((-B - sqrtD) / A);
            tryRoot((-B + sqrtD) / A);
        }
        return n;
    }
} // namespace

// ====================================================================
// GroupHits — in-place sort + group (modifies hits vector)
// ====================================================================
void G4StepSolid::GroupHits(std::vector<RayHit>& hits, std::vector<HitGroup>& out, G4double tol) {
    out.clear();
    if (hits.empty()) return;
    std::sort(hits.begin(), hits.end(),
              [](const RayHit& a, const RayHit& b){ return a.w < b.w; });

    for (const RayHit& h : hits) {
        if (h.trans == IntCurveSurface_Tangent) continue;
        if (!out.empty() && h.w - out.back().dist < tol) {
            if (h.trans == IntCurveSurface_In) {
                out.back().nIn++;
            } else {
                out.back().nOut++;
                if (out.back().outFace.IsNull()) {
                    out.back().outFace = h.face;
                    out.back().outU    = h.u;
                    out.back().outV    = h.v;
                }
            }
            if (out.back().anyFace.IsNull()) out.back().anyFace = h.face;
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
            out.push_back(g);
        }
    }
}

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

    perSolidTol = G4StepConfig::Get().perSolidTol;

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

    const size_t nFaces = ownerSolid->fFaces.size();
    scratchHits.reserve(2 * nFaces);
    scratchGroups.reserve(nFaces);
    scratchInsideHits.reserve(2 * nFaces);
    scratchFaceOrder.reserve(nFaces);  // CT1: one entry per face

    // --- CT-IN: build per-solid face index lists + analytic-Inside coverage flags ---
    // solidClassifiers has one entry per TopAbs_SOLID (built above, same `si` order).
    // For each face in fFaces, look up its owning solid via faceSolidMap and bucket it.
    const size_t nSolids = solidClassifiers.size();
    solidFaceIdx.assign(nSolids, {});
    analyticInsideOK.assign(nSolids, 1);  // start true; cleared by any non-analytic face
    for (int fi = 0; fi < (int)ownerSolid->fFaces.size(); ++fi) {
        const TopoDS_Face& f = ownerSolid->fFaces[fi].face;
        int si = FaceToSolid(f);
        if (si < 0 || si >= (int)nSolids) {
            // Unmapped face (should not happen): be conservative — no analytic Inside.
            for (auto& flag : analyticInsideOK) flag = 0;
            continue;
        }
        solidFaceIdx[si].push_back(fi);
        const auto st = ownerSolid->fFaces[fi].surfType;
        // Cone/Torus are handled in the parity path via the trim-exact OCCT per-face
        // intersector (Phase A), so they no longer disqualify a solid from analytic
        // (ray-parity) Inside(). Only a genuinely unhandled type (Other = BSpline/
        // Bezier/Revolution/…) forces the whole solid onto BRepClass3d.
        if (st == G4StepSolid::FaceEntry::SurfType::Other) {
            analyticInsideOK[si] = 0;  // unhandled surface → OCCT classifier for this solid
        }
    }
    // A solid with zero registered faces (degenerate) must not use analytic Inside.
    for (size_t i = 0; i < nSolids; ++i)
        if (solidFaceIdx[i].empty()) analyticInsideOK[i] = 0;

    // --- CT-TRIM / CT-TRIM-2: build + self-validate the UV trim model per face ---
    // BuildUVTrim tries a v-band model (CT-TRIM) and, for faces that are not a clean
    // v-band (part_1's seam-notched full-2π cylinders), a general seam-aware polygon
    // model (CT-TRIM-2). A model becomes .valid only on ZERO mismatch vs FClass2d;
    // others keep .valid=false and the OCCT intersector path (no regression). Build is
    // offline (per-thread, once) so its FClass2d cost is irrelevant to runtime navigation.
    {
        const int nF = (int)ownerSolid->fFaces.size();
        faceUVTrim.resize(nF);
        int nValid = 0, nBand = 0, nPoly = 0;
        const bool uvOff = G4StepConfig::Get().uvTrimOff;  // A/B vs CT1
        for (int fi = 0; !uvOff && fi < nF; ++fi) {
            faceUVTrim[fi] = ownerSolid->BuildUVTrim(ownerSolid->fFaces[fi], occtTol);
            if (faceUVTrim[fi].valid) {
                ++nValid;
                if (faceUVTrim[fi].mode == UVTrim::Mode::Poly) ++nPoly; else ++nBand;
            }
        }
        if (G4StepConfig::Get().uvTrimVerbose)
            G4cout << "[CT-TRIM] " << ownerSolid->GetName() << ": "
                   << nValid << "/" << nF << " faces use UV trim ("
                   << nBand << " band, " << nPoly << " polygon)" << G4endl;
    }

    // --- CT-IN: per-solid parity-direction selection ---
    // For each analytic solid, time a fixed set of candidate ray directions against
    // its faces (using the prebuilt OCCT intersectors that the non-trim-safe faces
    // will actually use) and keep the 4 fastest, fastest first. This sidesteps the
    // OCCT IntCurvesFace_Intersector direction pathology on BSpline-trimmed cylinders.
    // The candidate set is deterministic; the probe origins are deterministic too.
    {
        solidParityDirs.assign(nSolids, {});
        // 14 well-spread unit directions (axis-ish + diagonals). Deterministic.
        static const double cand[][3] = {
            {0.9789,0.1449,0.1449},{0.1449,0.9789,0.1449},{0.1449,0.1449,0.9789},
            {0.8018,0.5345,0.2673},{0.2673,0.8018,0.5345},{0.5345,0.2673,0.8018},
            {0.5774,0.5774,0.5774},{-0.5774,0.5774,0.5774},{0.5774,-0.5774,0.5774},
            {0.5774,0.5774,-0.5774},{0.3015,-0.9045,0.3015},{-0.4264,0.6396,0.6396},
            {0.6963,0.6963,0.1741},{0.1741,0.6963,0.6963}
        };
        const int NC = (int)(sizeof(cand)/sizeof(cand[0]));
        for (size_t i = 0; i < nSolids; ++i) {
            if (!analyticInsideOK[i]) continue;
            // Solid bbox centroid + a few interior-ish probe origins.
            Bnd_Box b; { for (int fi : solidFaceIdx[i]) b.Add(ownerSolid->fFaces[fi].bbox); }
            if (b.IsVoid()) continue;
            double x0,y0,z0,x1,y1,z1; b.Get(x0,y0,z0,x1,y1,z1);
            gp_Pnt probes[3] = {
                gp_Pnt(0.5*(x0+x1), 0.5*(y0+y1), 0.5*(z0+z1)),
                gp_Pnt(0.5*(x0+x1)+0.13*(x1-x0), 0.5*(y0+y1)-0.11*(y1-y0), 0.5*(z0+z1)+0.07*(z1-z0)),
                gp_Pnt(0.5*(x0+x1)-0.09*(x1-x0), 0.5*(y0+y1)+0.15*(y1-y0), 0.5*(z0+z1)-0.12*(z1-z0))
            };
            std::vector<std::pair<double,int>> cost(NC);  // (time, candIdx)
            for (int c = 0; c < NC; ++c) {
                double L = std::sqrt(cand[c][0]*cand[c][0]+cand[c][1]*cand[c][1]+cand[c][2]*cand[c][2]);
                gp_Dir gd(cand[c][0]/L, cand[c][1]/L, cand[c][2]/L);
                auto t0 = std::chrono::high_resolution_clock::now();
                for (const gp_Pnt& pr : probes) {
                    gp_Lin ray(pr, gd);
                    for (int fi : solidFaceIdx[i]) {
                        // Only non-trim-safe faces incur the OCCT intersector cost.
                        if (!ownerSolid->fFaces[fi].analyticTrimSafe)
                            faceIntersectors[fi]->Perform(ray, -occtTol, kInfinity);
                    }
                }
                auto t1 = std::chrono::high_resolution_clock::now();
                cost[c] = { (double)std::chrono::duration_cast<std::chrono::nanoseconds>(t1-t0).count(), c };
            }
            std::sort(cost.begin(), cost.end());
            int keep = std::min(4, NC);
            solidParityDirs[i].reserve(keep);
            for (int k = 0; k < keep; ++k) {
                const double* d = cand[cost[k].second];
                double L = std::sqrt(d[0]*d[0]+d[1]*d[1]+d[2]*d[2]);
                solidParityDirs[i].emplace_back(d[0]/L, d[1]/L, d[2]/L);
            }
            if (G4StepConfig::Get().parityDirSel)
                G4cout << "[DIRSEL] solid#" << i << " best cand=" << cost[0].second
                       << " t=" << cost[0].first << "ns  worst t=" << cost.back().first << "ns" << G4endl;
        }
    }

    static std::once_flag tolPrintFlag;
    if (G4StepConfig::Get().tolVerbose)
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
// NearestFace  —  analytic fast-path for Plane/Cylinder/Sphere
// ====================================================================
G4StepSolid::NearestFaceResult G4StepSolid::NearestFace(const gp_Pnt& pt) const {
    NearestFaceResult result;
    if (fFaces.empty()) return result;

    AlgoCache* algo = GetAlgo();
    const G4double eps = algo->occtTol;

    // OCCT fallback vertex shape — only constructed if at least one face
    // actually needs it (lazy, avoids MakeVertex when all faces are analytic).
    bool vtxBuilt = false;
    TopoDS_Shape vtxShape;
    auto getVtx = [&]() -> const TopoDS_Shape& {
        if (!vtxBuilt) { vtxShape = BRepBuilderAPI_MakeVertex(pt).Shape(); vtxBuilt = true; }
        return vtxShape;
    };

    // Inline dot-product helpers (avoids gp_Vec temporaries)
    auto dot3 = [](double ax, double ay, double az,
                   double bx, double by, double bz) {
        return ax*bx + ay*by + az*bz;
    };

    // CT1: sort face indices by bbox lower-bound distance (ascending) so the
    // nearest-bbox faces are processed first, making the existing prune effective.
    // Once the bbox lower bound >= current best, all remaining faces are farther → break.
    // Use the AlgoCache scratch buffer to avoid per-query heap allocation.
    auto& faceOrder = algo->scratchFaceOrder;
    faceOrder.clear();
    const int nf = (int)fFaces.size();
    for (int i = 0; i < nf; ++i)
        faceOrder.emplace_back(BBoxPointDist(fFaces[i].bbox, pt), i);
    std::sort(faceOrder.begin(), faceOrder.end());  // ascending by bbox dist

    for (const auto& [bboxDist, i] : faceOrder) {
        if (bboxDist >= result.dist) break;          // sorted → all remaining are farther
        if (fTimingEnabled) ++algo->timing.nfFacesVisited;

        const FaceEntry& fe = fFaces[i];
        G4double d = kInfinity;
        gp_Pnt   foot;
        bool     analytic = false;

        if (fe.surfType == FaceEntry::SurfType::Plane) {
            const gp_Pnt& loc = fe.pos.Location();
            // Direction() is the plane normal (Z axis of gp_Ax3)
            const gp_Dir& n   = fe.pos.Direction();
            double dpx = pt.X()-loc.X(), dpy = pt.Y()-loc.Y(), dpz = pt.Z()-loc.Z();
            double dn   = dot3(n.X(),n.Y(),n.Z(), dpx,dpy,dpz);
            // UV coords of foot in plane parametric space
            const gp_Dir& xu = fe.pos.XDirection();
            const gp_Dir& yv = fe.pos.YDirection();
            double uf = dot3(xu.X(),xu.Y(),xu.Z(), dpx,dpy,dpz);
            double vf = dot3(yv.X(),yv.Y(),yv.Z(), dpx,dpy,dpz);
            if (uf >= fe.u0 - eps && uf <= fe.u1 + eps &&
                vf >= fe.v0 - eps && vf <= fe.v1 + eps) {
                d    = std::abs(dn);
                foot = gp_Pnt(pt.X() - dn*n.X(),
                              pt.Y() - dn*n.Y(),
                              pt.Z() - dn*n.Z());
                analytic = true;
                if (fTimingEnabled) ++algo->timing.nfAnalyticCount;
            }

        } else if (fe.surfType == FaceEntry::SurfType::Cylinder) {
            const gp_Pnt& loc   = fe.pos.Location();
            const gp_Dir& axDir = fe.pos.Direction();  // cylinder axis
            double dpx = pt.X()-loc.X(), dpy = pt.Y()-loc.Y(), dpz = pt.Z()-loc.Z();
            // Axial (V) parameter
            double vf = dot3(axDir.X(),axDir.Y(),axDir.Z(), dpx,dpy,dpz);
            if (vf >= fe.v0 - eps && vf <= fe.v1 + eps) {
                // Radial component (perp to axis)
                double px = dpx - vf*axDir.X();
                double py = dpy - vf*axDir.Y();
                double pz = dpz - vf*axDir.Z();
                double r  = std::sqrt(px*px + py*py + pz*pz);
                // Angular check — skip for full cylinder (u1-u0 ≈ 2π)
                bool angOK = (fe.u1 - fe.u0 >= 2.0*M_PI - eps);
                if (!angOK && r > eps) {
                    // Compute angle in cylinder's local XY frame
                    const gp_Dir& xu = fe.pos.XDirection();
                    const gp_Dir& yv = fe.pos.YDirection();
                    double uf = std::atan2(dot3(yv.X(),yv.Y(),yv.Z(), px,py,pz),
                                           dot3(xu.X(),xu.Y(),xu.Z(), px,py,pz));
                    if (uf < fe.u0 - eps) uf += 2.0*M_PI;
                    angOK = (uf >= fe.u0 - eps && uf <= fe.u1 + eps);
                }
                if (angOK && r > 1e-15) {
                    d = std::abs(r - fe.cylR);
                    double scale = fe.cylR / r;
                    foot = gp_Pnt(loc.X() + vf*axDir.X() + px*scale,
                                  loc.Y() + vf*axDir.Y() + py*scale,
                                  loc.Z() + vf*axDir.Z() + pz*scale);
                    analytic = true;
                    if (fTimingEnabled) ++algo->timing.nfAnalyticCount;
                }
            }

        } else if (fe.surfType == FaceEntry::SurfType::Sphere) {
            const gp_Pnt& cen = fe.pos.Location();
            double dpx = pt.X()-cen.X(), dpy = pt.Y()-cen.Y(), dpz = pt.Z()-cen.Z();
            double r = std::sqrt(dpx*dpx + dpy*dpy + dpz*dpz);
            // Full sphere check (u span ≥ 2π, v span ≥ π)
            bool fullSphere = (fe.u1 - fe.u0 >= 2.0*M_PI - eps &&
                               fe.v1 - fe.v0 >= M_PI - eps);
            if (fullSphere && r > 1e-15) {
                d = std::abs(r - fe.sphR);
                double scale = fe.sphR / r;
                foot = gp_Pnt(cen.X() + dpx*scale,
                              cen.Y() + dpy*scale,
                              cen.Z() + dpz*scale);
                analytic = true;
                if (fTimingEnabled) ++algo->timing.nfAnalyticCount;
            }
        }

        if (!analytic) {
            // OCCT fallback
            BRepExtrema_DistShapeShape& extrema = *algo->faceExtrema[i];
            extrema.LoadS1(getVtx());
            extrema.Perform();
            if (fTimingEnabled) ++algo->timing.nfExtremaCount;
            if (fTimingEnabled) ++algo->timing.nfFallbackCount;
            if (!extrema.IsDone()) continue;
            d    = extrema.Value();
            foot = extrema.PointOnShape2(1);
        }

        if (d < result.dist) {
            result.dist    = d;
            result.idx     = i;
            result.pOnFace = foot;
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

    std::vector<RayHit>& rawHits = algo->scratchHits;
    rawHits.clear();

    for (int fi = 0; fi < (int)fFaces.size(); ++fi) {
        const FaceEntry& fe = fFaces[fi];
        if (!rayHitsBox(ray.Location(), invDir, fe.bbox, octTol)) continue;
        if (fTimingEnabled) ++algo->timing.rayFacesVisited;

        bool analytic = false;

        // ─── Plane (rectangular only; circle/annulus trim falls back to OCCT) ──
        if (fe.surfType == FaceEntry::SurfType::Plane && fe.isRectPlane) {
            bool fwd = (fe.face.Orientation() != TopAbs_REVERSED);
            RaySurfCand cand[1];
            int nc = RayIntersectPlane(fe.pos, fe.u0, fe.u1, fe.v0, fe.v1, fwd,
                                       p.x(),p.y(),p.z(), v.x(),v.y(),v.z(), octTol, cand);
            for (int c = 0; c < nc; ++c) {
                rawHits.emplace_back(cand[c].t, cand[c].trans, fe.face, cand[c].u, cand[c].v);
                if (fTimingEnabled) algo->timing.rayHitsTotal += 1;
            }
            analytic = true;
            if (fTimingEnabled) ++algo->timing.rayAnalyticCount;

        // ─── Cylinder ────────────────────────────────────────────────────
        } else if (fe.surfType == FaceEntry::SurfType::Cylinder) {
            bool fwd     = (fe.face.Orientation() != TopAbs_REVERSED);
            bool fullAng = (fe.u1 - fe.u0 >= 2.0*M_PI - octTol);
            RaySurfCand cand[2];
            int nc = RayIntersectCylinder(fe.pos, fe.cylR, fe.u0, fe.v0, fe.v1, fwd,
                                          p.x(),p.y(),p.z(), v.x(),v.y(),v.z(), octTol, cand);
            for (int c = 0; c < nc; ++c) {
                if (!fullAng && (cand[c].u < fe.u0 - octTol || cand[c].u > fe.u1 + octTol)) continue;
                rawHits.emplace_back(cand[c].t, cand[c].trans, fe.face, cand[c].u, cand[c].v);
                if (fTimingEnabled) algo->timing.rayHitsTotal += 1;
            }
            analytic = true;
            if (fTimingEnabled) ++algo->timing.rayAnalyticCount;

        // ─── Sphere ──────────────────────────────────────────────────────
        } else if (fe.surfType == FaceEntry::SurfType::Sphere) {
            bool fwd = (fe.face.Orientation() != TopAbs_REVERSED);
            RaySurfCand cand[2];
            int nc = RayIntersectSphere(fe.pos, fe.sphR, fe.u0, fe.u1, fe.v0, fe.v1, fwd,
                                        p.x(),p.y(),p.z(), v.x(),v.y(),v.z(), octTol, cand);
            for (int c = 0; c < nc; ++c) {
                rawHits.emplace_back(cand[c].t, cand[c].trans, fe.face, cand[c].u, cand[c].v);
                if (fTimingEnabled) algo->timing.rayHitsTotal += 1;
            }
            analytic = true;
            if (fTimingEnabled) ++algo->timing.rayAnalyticCount;
        }

        // ─── OCCT fallback for Other surface types ────────────────────────
        if (!analytic) {
            IntCurvesFace_Intersector& inter = *algo->faceIntersectors[fi];
            inter.Perform(ray, -octTol, kInfinity);
            if (fTimingEnabled) { ++algo->timing.rayPerformCount; ++algo->timing.rayFallbackCount; }
            for (int j = 1; j <= inter.NbPnt(); ++j) {
                rawHits.emplace_back(inter.WParameter(j), inter.Transition(j),
                                     fe.face, inter.UParameter(j), inter.VParameter(j));
            }
            if (fTimingEnabled) algo->timing.rayHitsTotal += inter.NbPnt();
        }
    }

    if (rawHits.empty()) {
        if (kind == BoundaryKind::Entry) {
            // No forward intersection found — all cylinder/plane/sphere roots were
            // behind the particle (negative t). This can happen when the particle
            // overshoots a concave boundary by a tiny floating-point amount (e.g.
            // DistanceToOut stepped from Si past the inner cylinder wall by ε, the
            // navigator placed the particle in World, but the physical position is
            // r = R+ε still inside Si). If Inside(p) confirms the particle is on or
            // inside the solid, return kCarTolerance so the navigator can correct the
            // volume assignment with a harmless micro-step.
            if (Inside(p) != kOutside)
                return kCarTolerance;
            return kInfinity;
        }
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

    // Re-entrancy test for the DistanceToOut exit-normal validity flag. Geant4's
    // navigator applies an "exit block" optimisation — after a track exits a
    // daughter through a VALID exit normal, that daughter is skipped for one step
    // to avoid immediate re-entry (see G4NormalNavigation::ComputeStep,
    // localDirection.dot(exitNormal) >= kMinExitingNormalCosine). That is only
    // safe for a locally convex exit. For a concave / re-entrant exit — e.g.
    // leaving the inner wall of a cylindrical hole and crossing the cavity back
    // into the SAME solid — the ray genuinely re-enters, and skipping it makes the
    // track ghost-step through the material as if it were vacuum (energy loss).
    // Native G4SubtractionSolid returns validNorm=false at such exits, which
    // disables the optimisation and lets the navigator re-enter correctly. We
    // reproduce that: report the exit normal as INVALID when the ray has a further
    // net-Entry crossing beyond this exit (using hits already computed here — no
    // extra ray cast). Convex exits (no re-entry ahead) keep validNorm=true so the
    // navigator optimisation is preserved.
    auto reentersAfter = [&](G4double exitDist) -> bool {
        for (const auto& g2 : algo->scratchGroups)
            if ((g2.nIn - g2.nOut) > 0 && g2.dist > exitDist + octTol)
                return true;
        return false;
    };

    GroupHits(rawHits, algo->scratchGroups, octTol);
    for (const auto& g : algo->scratchGroups) {
        // Inside-detection for DistanceToIn. RayCast collects forward hits only,
        // so the FIRST forward net-crossing group decides p's location relative to
        // the solid. If that crossing is an EXIT (nOut>nIn), the ray leaves the
        // solid before entering it → p is already inside → DistanceToIn = 0, which
        // matches native G4SubtractionSolid and the Geant4 convention. The old
        // behaviour skipped Exit groups and returned the FAR-wall re-entry distance,
        // so a particle mis-located in World but physically inside a concave solid
        // (after exiting the inner hole wall) was handed a ~40 mm re-entry distance
        // and ghost-stepped through the material as vacuum, losing energy. Only the
        // first forward net-crossing is consulted; net-zero/grazing groups skipped.
        if (!wantExit) {
            const int net = g.nIn - g.nOut;
            if (net == 0) continue;                    // net-zero / tangent / grazing
            const G4double bandE = bandOf(g);
            if (std::abs(g.dist) < bandE) {
                // On the surface. Only an ENTRY crossing here (net>0, v pointing
                // into the solid) means the particle is entering now → 0. An EXIT
                // crossing (net<0, v pointing out) means the particle is on the
                // boundary LEAVING — it is not entering, so this group must be
                // skipped and a genuine forward re-entry (concave) sought instead;
                // returning 0 here let the navigator re-enter a just-exited solid at
                // zero distance, the solid↔world stuck-track oscillation (RP.step).
                if (net > 0) return 0.0;
                continue;
            }
            if (g.dist > bandE) {
                if (net < 0) {                         // first forward crossing is an EXIT
                    // Genuinely inside a concave solid (box_hole: track mislocated in
                    // World but physically inside) → the far exit is the first forward
                    // crossing and DistanceToIn=0 is correct. But a track sitting ON
                    // the surface and LEAVING (e.g. exiting part_2 axially) also sees a
                    // far exit as its first forward crossing while Inside(p)=kSurface —
                    // it is not inside, so returning 0 lets the navigator re-enter a
                    // just-exited solid → residual solid↔world stuck oscillation.
                    // Discriminate with Inside(p) (analytic & fast here; this net<0
                    // path is rare — DistanceToIn is normally queried from outside).
                    if (Inside(p) == kInside) {
                        if (fVerboseLevel >= 2)
                            G4cout << "[G4StepSolid::" << GetName() << "::DistanceToIn][local]"
                                   << " first forward crossing is Exit & p inside -> 0.0" << G4endl;
                        return 0.0;
                    }
                    continue;                          // on surface / outside & leaving → seek real entry
                }
                return g.dist;                         // net > 0 → genuine forward entry
            }
            continue;                                  // behind the point → ignore
        }

        if (!isBoundaryGroup(g)) continue;

        const G4double band = bandOf(g);
        if (std::abs(g.dist) < band) {
            if (calcNorm && validNorm && !g.outFace.IsNull()) {
                *n = GetNormalAtIntersection(g.outFace, g.outU, g.outV);
                *validNorm = !reentersAfter(g.dist);
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
                *validNorm = !reentersAfter(g.dist);
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
        // No valid Entry boundary group found. Three legitimate causes:
        // (a) On-surface (distance ≈ 0): a group exists at t≈0 but was classified Out
        //     (e.g. inner cylinder of a subtraction solid has REVERSED orientation, so an
        //     inward-approaching particle at the concave surface gets trans=Out).
        // (b) MSC overshoot near surface: DistanceToOut returned exact r=R, but MSC
        //     laterally displaced the endpoint by δ (10–500 nm), putting the particle at
        //     r=R+δ still inside the solid but the navigator already transitioned it to
        //     World. Near-side hit is at t≈δ/v_r and also has trans=Out.
        // (c) Deep mis-tracking: after a physics-limited step that crossed the solid
        //     boundary (e.g. traversing a concave hole), the particle ends up deep inside
        //     the solid but is still tracked in World. All forward hits are Exit transitions
        //     (particle heading outward through the solid). Inside(p) confirms mis-tracking.
        // Cases (a)/(b): smallest group dist < octTol*1e4 ≈ 1 µm.
        // Case (c): Inside(p) != kOutside.
        if (!algo->scratchGroups.empty()) {
            const G4double dMin = algo->scratchGroups.front().dist;
            if (dMin >= 0.0 && dMin < octTol * 1e4)
                return kCarTolerance;
        }
        if (Inside(p) != kOutside)
            return kCarTolerance;
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

// ====================================================================
// CT-TRIM: UV v-band trim model (build + classify)
// ====================================================================
//
// ---- Crossing-number point-in-ring test (a horizontal ray in +u at fixed v).
// Counts ring edges the ray crosses; XORs winding. Sets nearEdge if the query is
// within `band` (in u, at the crossing v) of a crossed edge → ambiguous.
// The ring vertices are a closed polyline in the (u,v) plane.
static inline int CrossingNumberRing(const std::vector<double>& ru,
                                     const std::vector<double>& rv,
                                     double uu, double v, double band,
                                     bool& nearEdge) {
    const size_t n = ru.size();
    int wind = 0;
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        double ui = ru[i], vi = rv[i], uj = ru[j], vj = rv[j];
        if ((vi > v) != (vj > v)) {
            double ut = uj + (v - vj) / (vi - vj) * (ui - uj);
            if (uu < ut) wind ^= 1;
            if (std::abs(uu - ut) < band) nearEdge = true;
        }
    }
    return wind;
}

// ClassifyUVTrim — classify a UV hit (u,v) against a validated trim model.
//   +1 inside face, 0 outside face, -1 ambiguous (within boundary band → use OCCT).
// u is expected in the face period; it is wrapped into [u0,u1) for full-periodic faces.
//   Mode::Band  → bot(u) ≤ v ≤ top(u) minus holes (the original CT-TRIM model).
//   Mode::Poly  → seam-aware crossing-number: inside the even-odd union of the outer
//                 rings AND not inside any hole. Used for faces (part_1's full-2π
//                 cylinders) whose outer trim carries a seam-crossing notch so a v-band
//                 envelope is wrong near the seam.
int G4StepSolid::ClassifyUVTrim(const AlgoCache::UVTrim& t, double u, double v) {
    if (!t.valid) return -1;
    double uu = u;
    if (t.periodic) {
        // Wrap the (atan2-derived) hit angle into the face's u-domain [u0,u1).
        const double period = 2.0 * M_PI;
        while (uu <  t.u0)        uu += period;
        while (uu >= t.u0+period) uu -= period;
    } else {
        // Partial-u face: a hit outside [u0,u1] is off the face. atan2 may return the
        // hit shifted by ±2π; bring it near the domain before the range check.
        while (uu < t.u0 - M_PI) uu += 2.0*M_PI;
        while (uu > t.u1 + M_PI) uu -= 2.0*M_PI;
        if (uu < t.u0 - t.band || uu > t.u1 + t.band) return 0;  // off the face in u
        if (uu < t.u0 + t.band || uu > t.u1 - t.band) return -1; // near u-edge → defer
    }

    if (t.mode == AlgoCache::UVTrim::Mode::Poly) {
        // v outside the axial extent (with margin) → definitely off the face.
        if (v < t.v0 - t.band || v > t.v1 + t.band) return 0;
        // For a full-periodic Poly face the outer rings are closed in [u0,u1]×v (seam
        // edges included). A horizontal +u ray at fixed v from uu would, for a periodic
        // ring, need to wrap; instead we test uu directly against rings that already
        // include the seam segments at u0/u1, so the ray stays within [u0,u1].
        bool nearEdge = false;
        int inOuter = 0;
        for (size_t r = 0; r < t.outU.size(); ++r)
            inOuter ^= CrossingNumberRing(t.outU[r], t.outV[r], uu, v, t.band, nearEdge);
        if (nearEdge) return -1;        // near the outer boundary → defer to OCCT
        if (!inOuter) return 0;         // outside the outer rings → off the face
        for (size_t h = 0; h < t.holeU.size(); ++h) {
            bool he = false;
            int wind = CrossingNumberRing(t.holeU[h], t.holeV[h], uu, v, t.band, he);
            if (he)   return -1;        // near a hole boundary → defer
            if (wind) return 0;         // inside a hole → off the face
        }
        return 1;                       // inside outer, outside all holes → in face
    }

    // --- Band model ---
    int b = (int)((uu - t.u0) / t.du);
    if (b < 0) b = 0;
    if (b >= t.NB) b = t.NB - 1;
    const double top = t.top[b];
    const double bot = t.bot[b];
    // Outside the v-band (with band margin) → clearly outside / clearly inside edge.
    if (v < bot - t.band || v > top + t.band) return 0;
    if (v < bot + t.band || v > top - t.band) return -1;  // near top/bot envelope → defer
    // Inside the band core → check holes. A hit inside a hole loop is OUTSIDE the face.
    for (size_t h = 0; h < t.holeU.size(); ++h) {
        bool nearEdge = false;
        int wind = CrossingNumberRing(t.holeU[h], t.holeV[h], uu, v, t.band, nearEdge);
        if (nearEdge) return -1;       // near a hole boundary → defer to OCCT
        if (wind)     return 0;        // inside a hole → outside the face
    }
    return 1;                          // inside the band, outside all holes → in face
}

// BuildUVTrim — discretise a full-periodic analytic cylinder face's real trim into a
// v-band model (top(u)/bot(u) envelopes + interior hole loops), then SELF-VALIDATE it
// against BRepTopAdaptor_FClass2d on a dense UV grid. .valid is set true only when the
// band reproduces OCCT's in/out classification with ZERO mismatch. Conservative by
// design: any face that does not validate keeps .valid=false and is never accelerated.
G4StepSolid::AlgoCache::UVTrim
G4StepSolid::BuildUVTrim(const FaceEntry& fe, G4double octTol) const {
    AlgoCache::UVTrim t;
    // Only cylinder faces with a non-trivial (non-trim-safe) trim qualify.
    if (fe.surfType != FaceEntry::SurfType::Cylinder) return t;
    if (fe.analyticTrimSafe) return t;   // trim-safe faces already take the fast path
    const double u0 = fe.u0, u1 = fe.u1, v0 = fe.v0, v1 = fe.v1;
    const bool fullU = (u1 - u0 >= 2.0*M_PI - 1e-7);
    if (v1 <= v0 || u1 <= u0) return t;
    const double uspan = u1 - u0;
    const TopoDS_Face& face = fe.face;
    const bool dbg = G4StepConfig::Get().uvTrimDebug;

    // Runtime boundary band (UV units). v maps 1:1 to mm (axial). Any hit within this of
    // an envelope / polygon / hole edge is deferred to the exact OCCT intersector.
    double band = G4StepConfig::Get().uvTrimBand;
    t.band = band;
    t.u0 = u0; t.u1 = u1; t.v0 = v0; t.v1 = v1;
    t.periodic = fullU;

    // --- Self-validation against BRepTopAdaptor_FClass2d (ground truth) ----------
    // Dense random UV cloud over [u0,u1]×[v0,v1]. Points the model would DEFER (cls==-1,
    // boundary band) are skipped — those take the exact OCCT path and cannot be wrong. We
    // require ZERO disagreement on the points the fast path actually CLASSIFIES
    // (cls∈{0,1}). A systematic model error misclassifies a large contiguous region, so a
    // sparse random sample reliably hits it. This is the safety net: a model that does not
    // validate keeps .valid=false and is never accelerated → correctness can't regress.
    BRepTopAdaptor_FClass2d fclass(face, 1e-9);
    auto selfValidate = [&](AlgoCache::UVTrim& cand, const char* tag) -> int {
        std::mt19937_64 rng(0x5UL * (uint64_t)(uintptr_t)&fe + 0x9E3779B97F4A7C15ULL);
        std::uniform_real_distribution<double> ru(u0, u1), rv(v0, v1);
        const int valN = dbg ? 20000 : 2000;
        int mism = 0, dbgShown = 0;
        for (int k = 0; k < valN && (dbg || mism == 0); ++k) {
            double u = ru(rng), v = rv(rng);
            int cls = ClassifyUVTrim(cand, u, v);
            if (cls < 0) continue;                  // deferred → never wrong
            TopAbs_State st = fclass.Perform(gp_Pnt2d(u, v));
            if (st == TopAbs_ON) continue;          // OCCT on-boundary → skip
            bool myIn = (cls == 1), occtIn = (st == TopAbs_IN);
            if (myIn != occtIn) {
                ++mism;
                if (dbg && dbgShown < 8) { ++dbgShown;
                    G4cout << "[UVTRIM-DBG]     " << tag << " MISM u=" << u << " v=" << v
                           << " myIn=" << myIn << " occtIn=" << occtIn << G4endl; }
            }
        }
        if (dbg) G4cout << "[UVTRIM-DBG]   " << tag << " validationMism(of " << valN
                        << ")=" << mism << G4endl;
        return mism;
    };

    // ============================================================================
    // ATTEMPT 1 — v-band model (the CT-TRIM model). Covers faces whose outer trim is
    // a single bot(u)≤v≤top(u) envelope (parts 0/2/5/6/7/8/9). Cheapest classify.
    // ============================================================================
    {
        const int NB = 2048;
        const double du = (u1 - u0) / NB;
        std::vector<double> top(NB, -1e300), bot(NB, 1e300);
        std::vector<std::vector<double>> holeU, holeV;
        bool sawOuter = false;
        for (TopExp_Explorer ew(face, TopAbs_WIRE); ew.More(); ew.Next()) {
            TopoDS_Wire wire = TopoDS::Wire(ew.Current());
            std::vector<std::vector<std::pair<double,double>>> edgePolys;
            double umin = 1e300, umax = -1e300;
            for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
                BRepAdaptor_Curve2d c2(we.Current(), face);
                double t0 = c2.FirstParameter(), t1 = c2.LastParameter();
                gp_Pnt2d a = c2.Value(t0), b = c2.Value(t1);
                bool seam = (std::abs(a.X() - b.X()) < 1e-6 &&
                             (std::abs(a.X() - u0) < 1e-3 || std::abs(a.X() - u1) < 1e-3));
                if (seam) continue;
                const int n = 512;
                std::vector<std::pair<double,double>> poly; poly.reserve(n + 1);
                for (int i = 0; i <= n; ++i) {
                    double tt = t0 + (t1 - t0) * ((double)i / n);
                    gp_Pnt2d q = c2.Value(tt);
                    poly.emplace_back(q.X(), q.Y());
                    umin = std::min(umin, q.X()); umax = std::max(umax, q.X());
                }
                edgePolys.push_back(std::move(poly));
            }
            if (edgePolys.empty()) continue;
            bool outer = (umax - umin >= uspan - std::max(1e-2, 1e-2*uspan));
            if (outer) {
                sawOuter = true;
                for (const auto& poly : edgePolys)
                    for (size_t i = 0; i + 1 < poly.size(); ++i) {
                        double ua = poly[i].first,  va = poly[i].second;
                        double ub = poly[i+1].first, vb = poly[i+1].second;
                        if (ua > ub) { std::swap(ua, ub); std::swap(va, vb); }
                        int ba = std::max(0, std::min(NB-1, (int)((ua - u0) / du)));
                        int bb = std::max(0, std::min(NB-1, (int)((ub - u0) / du)));
                        for (int b = ba; b <= bb; ++b) {
                            double uc = u0 + (b + 0.5) * du;
                            double vv = (ub > ua) ? va + (vb - va)*((uc - ua)/(ub - ua)) : va;
                            if (uc < ua) vv = va;
                            if (uc > ub) vv = vb;
                            top[b] = std::max(top[b], vv);
                            bot[b] = std::min(bot[b], vv);
                        }
                    }
            } else {
                std::vector<double> hu, hv;
                for (const auto& poly : edgePolys)
                    for (const auto& q : poly) { hu.push_back(q.first); hv.push_back(q.second); }
                if (hu.size() >= 3) { holeU.push_back(std::move(hu)); holeV.push_back(std::move(hv)); }
            }
        }
        bool degenerate = !sawOuter;
        if (!degenerate) {
            for (int b = 1;    b < NB; ++b) if (top[b] < -1e299) { top[b]=top[b-1]; bot[b]=bot[b-1]; }
            for (int b = NB-2; b >= 0; --b) if (top[b] < -1e299) { top[b]=top[b+1]; bot[b]=bot[b+1]; }
            for (int b = 0; b < NB; ++b) if (top[b] < -1e299) { degenerate = true; break; }
        }
        if (!degenerate) {
            AlgoCache::UVTrim band_t = t;
            band_t.mode = AlgoCache::UVTrim::Mode::Band;
            band_t.du = du; band_t.NB = NB;
            band_t.top = top; band_t.bot = bot;
            band_t.holeU = holeU; band_t.holeV = holeV;
            band_t.valid = true;
            if (dbg) G4cout << "[UVTRIM-DBG] cyl R=" << fe.cylR << " u=[" << u0 << "," << u1
                            << "] v=[" << v0 << "," << v1 << "] fullU=" << fullU
                            << " : try BAND nHoles=" << holeU.size() << G4endl;
            if (selfValidate(band_t, "BAND") == 0) return band_t;   // band model is exact
        }
    }

    // ============================================================================
    // ATTEMPT 2 — general seam-aware polygon model (CT-TRIM-2). For faces whose outer
    // trim is NOT a clean v-band (part_1's full-2π cylinders with a seam-crossing
    // notch). Reconstruct every wire (seam edges INCLUDED) as a closed UV ring in the
    // [u0,u1]×v plane by endpoint chaining, then classify by crossing-number:
    // inside the even-odd union of the large rings (outer) AND not inside any small
    // ring (hole). The seam is handled because the rings carry the vertical seam
    // segments at u0/u1 explicitly — no unwrapping, no ±2π query shifting needed.
    // ============================================================================
    if (!G4StepConfig::Get().uvTrimNoPoly) {
        // Per-wire: build closed UV rings by chaining oriented edge polylines.
        std::vector<std::vector<double>> outU, outV, holeU, holeV;
        const double chainTol = 1e-4;              // endpoint match tolerance in UV
        bool buildFail = false;
        for (TopExp_Explorer ew(face, TopAbs_WIRE); ew.More(); ew.Next() ) {
            TopoDS_Wire wire = TopoDS::Wire(ew.Current());
            // Collect oriented edge polylines (apply edge orientation so each runs
            // start→end along the wire's traversal direction).
            struct Seg { std::vector<std::pair<double,double>> pts; };
            std::vector<Seg> segs;
            double wvmin = 1e300, wvmax = -1e300, wumin = 1e300, wumax = -1e300;
            for (BRepTools_WireExplorer we(wire, face); we.More(); we.Next()) {
                BRepAdaptor_Curve2d c2(we.Current(), face);
                double t0 = c2.FirstParameter(), t1 = c2.LastParameter();
                const int n = 256;
                Seg s; s.pts.reserve(n + 1);
                for (int i = 0; i <= n; ++i) {
                    double tt = t0 + (t1 - t0) * ((double)i / n);
                    gp_Pnt2d q = c2.Value(tt);
                    s.pts.emplace_back(q.X(), q.Y());
                    wumin = std::min(wumin, q.X()); wumax = std::max(wumax, q.X());
                    wvmin = std::min(wvmin, q.Y()); wvmax = std::max(wvmax, q.Y());
                }
                if (we.Current().Orientation() == TopAbs_REVERSED)
                    std::reverse(s.pts.begin(), s.pts.end());
                segs.push_back(std::move(s));
            }
            if (segs.empty()) continue;
            // Greedy chain: start from seg 0, repeatedly attach the unused seg whose
            // start (or end, reversing) matches the current chain end within chainTol.
            // A periodic seam is fine: a seam edge at u0 and one at u1 are distinct
            // segments whose endpoints match their neighbours in the [u0,u1] plane.
            std::vector<char> used(segs.size(), 0);
            std::vector<std::pair<double,double>> ring;
            auto appendSeg = [&](std::vector<std::pair<double,double>>& dst,
                                 const std::vector<std::pair<double,double>>& src,
                                 bool skipFirst) {
                for (size_t i = skipFirst ? 1 : 0; i < src.size(); ++i) dst.push_back(src[i]);
            };
            appendSeg(ring, segs[0].pts, false);
            used[0] = 1;
            size_t attached = 1;
            while (attached < segs.size()) {
                double ex = ring.back().first, ey = ring.back().second;
                int best = -1; bool bestRev = false; double bestD = chainTol;
                for (size_t s = 0; s < segs.size(); ++s) {
                    if (used[s]) continue;
                    double sx = segs[s].pts.front().first, sy = segs[s].pts.front().second;
                    double tx = segs[s].pts.back().first,  ty = segs[s].pts.back().second;
                    double d0 = std::hypot(ex - sx, ey - sy);
                    double d1 = std::hypot(ex - tx, ey - ty);
                    if (d0 <= bestD) { bestD = d0; best = (int)s; bestRev = false; }
                    if (d1 <  bestD) { bestD = d1; best = (int)s; bestRev = true;  }
                }
                if (best < 0) { buildFail = true; break; }   // chain broke → bail to OCCT
                std::vector<std::pair<double,double>> sp = segs[best].pts;
                if (bestRev) std::reverse(sp.begin(), sp.end());
                appendSeg(ring, sp, true);
                used[best] = 1; ++attached;
            }
            if (buildFail) break;
            // Ring must close (last ≈ first within a looser tol; the seam may leave a
            // small gap). If it does not close, the chaining is unreliable → bail.
            double cx = ring.front().first - ring.back().first;
            double cy = ring.front().second - ring.back().second;
            if (std::hypot(cx, cy) > 1e-2) { buildFail = true; break; }
            std::vector<double> ru_, rv_;
            ru_.reserve(ring.size()); rv_.reserve(ring.size());
            for (auto& q : ring) { ru_.push_back(q.first); rv_.push_back(q.second); }
            // Classify ring as outer vs hole by its u-extent: an outer ring of a
            // full-2π face spans (nearly) the whole period; a hole is local. For a
            // partial-u face the outer ring spans the face's u-extent.
            bool isOuter = (wumax - wumin >= uspan - std::max(1e-2, 1e-2*uspan));
            if (isOuter) { outU.push_back(std::move(ru_)); outV.push_back(std::move(rv_)); }
            else         { holeU.push_back(std::move(ru_)); holeV.push_back(std::move(rv_)); }
        }
        if (!buildFail && !outU.empty()) {
            AlgoCache::UVTrim poly_t = t;
            poly_t.mode = AlgoCache::UVTrim::Mode::Poly;
            poly_t.outU = outU; poly_t.outV = outV;
            poly_t.holeU = holeU; poly_t.holeV = holeV;
            poly_t.valid = true;
            if (dbg) G4cout << "[UVTRIM-DBG]   try POLY nOuter=" << outU.size()
                            << " nHoles=" << holeU.size() << G4endl;
            if (selfValidate(poly_t, "POLY") == 0) return poly_t;  // polygon model exact
        }
    }

    // Neither model validated → keep .valid=false (OCCT intersector path, no regression).
    if (dbg) G4cout << "[UVTRIM-DBG]   -> NO VALID MODEL (keep OCCT)" << G4endl;
    return t;
}

// ====================================================================
// CT-IN: analytic point-in-solid via ray parity
// ====================================================================
//
// IntersectFaceForParity — append analytic ray-face hits for ONE face along the
// ray (p, dir). `dir` MUST be unit length so all w-parameters are true distances
// (keeps cylinder/sphere analytic roots consistent with the gp_Lin-based OCCT
// intersector used for non-rectangular planes). Returns false only for a
// non-analytic face type (Cone/Torus/Other) — the caller pre-screens solids so
// this should not fire, but it guards against a mismatch between the coverage
// flag and the per-face math.
//
// This mirrors the EXACT transition/UV math of RayCastToBoundary so that Inside()
// and DistanceToIn/Out agree on whether a point is on/in/out of the boundary.
bool G4StepSolid::IntersectFaceForParity(const FaceEntry& fe, const G4ThreeVector& p,
                                         const G4ThreeVector& dir, G4double octTol,
                                         std::vector<RayHit>& hits) const
{

    // Unhandled surface type (BSpline/Bezier/…) → caller must not be here (solid
    // pre-screened via analyticInsideOK). Cone/Torus ARE handled: they are always
    // analyticTrimSafe=false (Phase A) and fall to the trim-exact OCCT per-face
    // intersector path below, so ray parity works for cone/torus solids.
    if (fe.surfType == FaceEntry::SurfType::Other)
        return false;

    // Trim-exact path: any face whose UV bounding rectangle is NOT its exact trim
    // (non-rect plane, cylinder/sphere with cutouts or curved/partial trims).
    if (!fe.analyticTrimSafe) {
        AlgoCache* algo = GetAlgo();
        int fi = (int)(&fe - &fFaces[0]);   // fe ∈ fFaces → recover global index

        // ── CT-TRIM fast path: validated UV v-band trim model ──────────────────
        // For full-periodic cylinder faces with a validated band model, compute the
        // analytic ray-cylinder intersection (identical math to the trim-safe path)
        // and classify each candidate root's UV hit against the band. If every root
        // classifies cleanly (inside/outside, none in the boundary band), emit the
        // analytic hits — no OCCT intersector. If ANY root is ambiguous (within the
        // boundary band), discard all analytic results for this face and defer the
        // whole face to the OCCT trim-exact intersector (preserves correctness).
        const AlgoCache::UVTrim& tm =
            (fi < (int)algo->faceUVTrim.size()) ? algo->faceUVTrim[fi]
                                                : AlgoCache::UVTrim{};
        if (tm.valid && fe.surfType == FaceEntry::SurfType::Cylinder) {
            bool fwd = (fe.face.Orientation() != TopAbs_REVERSED);
            RaySurfCand cand[2];
            int nc = RayIntersectCylinder(fe.pos, fe.cylR, fe.u0, fe.v0, fe.v1, fwd,
                                          p.x(),p.y(),p.z(), dir.x(),dir.y(),dir.z(), octTol, cand);
            // Classify each candidate (in root order) against the validated v-band
            // trim; commit only if none is ambiguous. Stop at the first band hit
            // (mirrors the old deferOCCT short-circuit). A/disc failure → nc==0 →
            // empty accept (uvTrimAccept++), matching the old behaviour.
            bool deferOCCT = false;
            RaySurfCand staged[2];
            int  nStaged = 0;
            for (int c = 0; c < nc; ++c) {
                int cls = ClassifyUVTrim(tm, cand[c].u, cand[c].v);
                if (cls < 0) { deferOCCT = true; break; }  // boundary band → OCCT
                if (cls == 0) continue;                     // off the real face
                staged[nStaged++] = cand[c];
            }
            if (!deferOCCT) {
                for (int s = 0; s < nStaged; ++s)
                    hits.emplace_back(staged[s].t, staged[s].trans, fe.face,
                                      staged[s].u, staged[s].v);
                if (fTimingEnabled) ++algo->uvTrimAccept;
                return true;
            }
            if (fTimingEnabled) ++algo->uvTrimBandFallback;
            // fall through to OCCT for this face (a root was in the boundary band)
        }

        // Trim-exact OCCT path (faces without a validated band, or band-ambiguous hits).
        // The per-face OCCT IntCurvesFace_Intersector respects the true trimmed boundary.
        // Mandatory for ray parity — a single spurious hit on an over-covered UV
        // rectangle flips inside/outside. Still ~150× cheaper than
        // BRepClass3d_SolidClassifier::Perform (the pathology CT-IN replaced).
        gp_Lin ray(G4toOCCT(p), gp_Dir(dir.x(), dir.y(), dir.z()));
        IntCurvesFace_Intersector& inter = *algo->faceIntersectors[fi];
        inter.Perform(ray, -octTol, kInfinity);
        for (int j = 1; j <= inter.NbPnt(); ++j)
            hits.emplace_back(inter.WParameter(j), inter.Transition(j),
                              fe.face, inter.UParameter(j), inter.VParameter(j));
        return true;
    }

    // ─── Plane (rectangular trim → UV box exact) ───────────────────────────
    if (fe.surfType == FaceEntry::SurfType::Plane) {
        bool fwd = (fe.face.Orientation() != TopAbs_REVERSED);
        RaySurfCand cand[1];
        int nc = RayIntersectPlane(fe.pos, fe.u0, fe.u1, fe.v0, fe.v1, fwd,
                                   p.x(),p.y(),p.z(), dir.x(),dir.y(),dir.z(), octTol, cand);
        for (int c = 0; c < nc; ++c)
            hits.emplace_back(cand[c].t, cand[c].trans, fe.face, cand[c].u, cand[c].v);
        return true;
    }

    // ─── Cylinder (full periodic, rectangular v-trim → UV box exact) ───────
    if (fe.surfType == FaceEntry::SurfType::Cylinder) {
        bool fwd     = (fe.face.Orientation() != TopAbs_REVERSED);
        bool fullAng = (fe.u1 - fe.u0 >= 2.0*M_PI - octTol);
        RaySurfCand cand[2];
        int nc = RayIntersectCylinder(fe.pos, fe.cylR, fe.u0, fe.v0, fe.v1, fwd,
                                      p.x(),p.y(),p.z(), dir.x(),dir.y(),dir.z(), octTol, cand);
        for (int c = 0; c < nc; ++c) {
            if (!fullAng && (cand[c].u < fe.u0 - octTol || cand[c].u > fe.u1 + octTol)) continue;
            hits.emplace_back(cand[c].t, cand[c].trans, fe.face, cand[c].u, cand[c].v);
        }
        return true;
    }

    // ─── Sphere ───────────────────────────────────────────────────────────
    if (fe.surfType == FaceEntry::SurfType::Sphere) {
        bool fwd = (fe.face.Orientation() != TopAbs_REVERSED);
        RaySurfCand cand[2];
        int nc = RayIntersectSphere(fe.pos, fe.sphR, fe.u0, fe.u1, fe.v0, fe.v1, fwd,
                                    p.x(),p.y(),p.z(), dir.x(),dir.y(),dir.z(), octTol, cand);
        for (int c = 0; c < nc; ++c)
            hits.emplace_back(cand[c].t, cand[c].trans, fe.face, cand[c].u, cand[c].v);
        return true;
    }

    return false;  // non-analytic face type
}

// InsideSolidAnalytic — classify p against analytic solid `si` by ray parity.
// Casts a deterministic ray, builds hit groups via GroupHits, and counts genuine
// transversal crossings in front of p (w > 0). Odd ⇒ inside; even ⇒ outside.
// If the nearest crossing group is within the solid band ⇒ kSurface. Degenerate
// rays (grazing edges/vertices producing only Tangent groups near the parity
// boundary, or an odd number of net-zero-cancelling groups) are retried with a
// deterministically perturbed direction; `ok=false` if still ambiguous.
EInside G4StepSolid::InsideSolidAnalytic(int si, const G4ThreeVector& p,
                                         AlgoCache* algo, bool& ok) const
{
    ok = true;
    const std::vector<int>& faceIdx = algo->solidFaceIdx[si];
    const G4double band = algo->SolidBand(si);
    // octTol is the global OCCT search/grouping window — reuse for hit grouping
    // so coincident hits at shared edges merge exactly as in the ray path.
    const G4double octTol = algo->occtTol;

    // Deterministic ray directions: a primary axis-tilted dir, then perturbations.
    // Tilted away from axes to avoid systematically grazing axis-aligned planes.
    static const G4ThreeVector kDirs[4] = {
        G4ThreeVector(0.5773502691896258, 0.5773502691896258, 0.5773502691896258),
        G4ThreeVector(0.7745966692414834, 0.5163977794943222, 0.3651483716701107),
        G4ThreeVector(-0.4264014327112209, 0.6396021490668313, 0.6396021490668313),
        G4ThreeVector(0.3015113445777636, -0.9045340337332909, 0.3015113445777636)
    };

    std::vector<RayHit>& hits = algo->scratchInsideHits;

    // Per-solid parity directions chosen at construction to avoid the OCCT
    // intersector's direction-dependent pathology on BSpline-trimmed cylinders
    // (a fixed diagonal can be 50–250× slower than a good direction). Fall back
    // to the global kDirs if none were selected for this solid.
    const std::vector<G4ThreeVector>* solidDirs =
        (si < (int)algo->solidParityDirs.size() && !algo->solidParityDirs[si].empty())
        ? &algo->solidParityDirs[si] : nullptr;
    // Cap attempts: on these BSpline-trimmed solids each ray is expensive, so retry
    // only a couple of perturbations before deferring to the OCCT classifier — fewer
    // repeated slow rays trims the heavy tail without changing the result (the
    // classifier is the authoritative fallback anyway).
    const int maxAvail  = solidDirs ? (int)solidDirs->size() : 4;
    const int nAttempts = std::min(2, maxAvail);

    for (int attempt = 0; attempt < nAttempts; ++attempt) {
        const G4ThreeVector& dir = solidDirs ? (*solidDirs)[attempt] : kDirs[attempt];
        hits.clear();
        bool allAnalytic = true;
        for (int fi : faceIdx) {
            if (!IntersectFaceForParity(fFaces[fi], p, dir, octTol, hits)) {
                allAnalytic = false;
                break;
            }
        }
        if (!allAnalytic) { ok = false; return kOutside; }

        GroupHits(hits, algo->scratchGroups, octTol);

        // Classify groups. A "crossing" is a group with a net transition
        // (nIn != nOut); these are the genuine boundary penetrations. Tangent-only
        // groups (nIn==nOut==0) were already dropped by GroupHits. A group that
        // carries hits but nets to zero (nIn==nOut>0) is a grazing coincidence
        // (e.g. ray tangent to a shared edge) → flag ambiguous and retry.
        //
        // Closed-manifold invariant: a non-degenerate ray crosses a closed solid
        // boundary an EVEN number of times total (every entry has a matching exit).
        // If totalCross is ODD, the ray grazed/clipped a boundary and the parity is
        // unreliable → perturb and retry; after all retries → OCCT classifier
        // fallback. This is the guard against the spurious-hit "explosion" family.
        int  parity      = 0;   // forward (dist > band) crossings → inside test
        int  totalCross  = 0;   // all crossings (any dist) → even-parity invariant
        bool nearSurface = false;
        bool ambiguous   = false;
        for (const auto& g : algo->scratchGroups) {
            bool crossing = (g.nIn != g.nOut);
            bool graze    = (!crossing && (g.nIn + g.nOut) > 0);
            if (std::abs(g.dist) <= band) {
                // Group essentially AT the point → on the surface.
                if (crossing || graze) nearSurface = true;
                continue;
            }
            if (crossing) {
                ++totalCross;
                if (g.dist > band) ++parity;
            } else if (graze) {
                ambiguous = true;
            }
        }

        if (nearSurface) return kSurface;
        // Odd total crossings ⇒ grazing/clipping artifact ⇒ unreliable parity.
        if (totalCross & 1) ambiguous = true;
        if (ambiguous) {
            if (attempt < nAttempts - 1) continue;  // perturb ray and recount
            ok = false; return kOutside;            // give up → caller uses OCCT classifier
        }
        return (parity & 1) ? kInside : kOutside;
    }
    ok = false;
    return kOutside;
}
