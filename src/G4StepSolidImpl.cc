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

    const size_t nFaces = ownerSolid->fFaces.size();
    scratchHits.reserve(2 * nFaces);
    scratchGroups.reserve(nFaces);

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

    for (int i = 0; i < (int)fFaces.size(); ++i) {
        const FaceEntry& fe = fFaces[i];
        if (BBoxPointDist(fe.bbox, pt) >= result.dist) continue;
        if (fTimingEnabled) ++algo->timing.nfFacesVisited;

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

    // Inline dot helper
    auto dot3 = [](double ax_, double ay_, double az_,
                   double bx_, double by_, double bz_) {
        return ax_*bx_ + ay_*by_ + az_*bz_;
    };

    for (int fi = 0; fi < (int)fFaces.size(); ++fi) {
        const FaceEntry& fe = fFaces[fi];
        if (!rayHitsBox(ray.Location(), invDir, fe.bbox, octTol)) continue;
        if (fTimingEnabled) ++algo->timing.rayFacesVisited;

        bool analytic = false;

        // ─── Plane (rectangular only; circle/annulus trim falls back to OCCT) ──
        if (fe.surfType == FaceEntry::SurfType::Plane && fe.isRectPlane) {
            const gp_Ax3& ax  = fe.pos;
            const gp_Dir& nm  = ax.Direction();
            const gp_Pnt& loc = ax.Location();
            double nv = dot3(nm.X(),nm.Y(),nm.Z(), v.x(),v.y(),v.z());
            if (std::abs(nv) >= 1e-15) {
                double np = dot3(nm.X(),nm.Y(),nm.Z(),
                                 p.x()-loc.X(), p.y()-loc.Y(), p.z()-loc.Z());
                double t = -np / nv;
                if (t >= -octTol) {
                    double hx = p.x()+t*v.x()-loc.X();
                    double hy = p.y()+t*v.y()-loc.Y();
                    double hz = p.z()+t*v.z()-loc.Z();
                    const gp_Dir& xu = ax.XDirection();
                    const gp_Dir& yv = ax.YDirection();
                    double uf = dot3(xu.X(),xu.Y(),xu.Z(), hx,hy,hz);
                    double vf = dot3(yv.X(),yv.Y(),yv.Z(), hx,hy,hz);
                    if (uf >= fe.u0-octTol && uf <= fe.u1+octTol &&
                        vf >= fe.v0-octTol && vf <= fe.v1+octTol) {
                        bool fwd = (fe.face.Orientation() != TopAbs_REVERSED);
                        double n_out_v = fwd ? nv : -nv;
                        IntCurveSurface_TransitionOnCurve trans;
                        if      (n_out_v < -octTol) trans = IntCurveSurface_In;
                        else if (n_out_v >  octTol) trans = IntCurveSurface_Out;
                        else                        trans = IntCurveSurface_Tangent;
                        rawHits.emplace_back(t, trans, fe.face, uf, vf);
                        if (fTimingEnabled) algo->timing.rayHitsTotal += 1;
                    }
                }
            }
            analytic = true;
            if (fTimingEnabled) ++algo->timing.rayAnalyticCount;

        // ─── Cylinder ────────────────────────────────────────────────────
        } else if (fe.surfType == FaceEntry::SurfType::Cylinder) {
            const gp_Ax3& ax    = fe.pos;
            const gp_Dir& axDir = ax.Direction();
            const gp_Pnt& axLoc = ax.Location();
            const double   R    = fe.cylR;

            double dpx = p.x()-axLoc.X(), dpy = p.y()-axLoc.Y(), dpz = p.z()-axLoc.Z();
            double da = dot3(axDir.X(),axDir.Y(),axDir.Z(), dpx,dpy,dpz);
            double va_ = dot3(axDir.X(),axDir.Y(),axDir.Z(), v.x(),v.y(),v.z());
            double ppx = dpx - da*axDir.X(), ppy = dpy - da*axDir.Y(), ppz = dpz - da*axDir.Z();
            double vpx = v.x()-va_*axDir.X(), vpy = v.y()-va_*axDir.Y(), vpz = v.z()-va_*axDir.Z();

            double A = vpx*vpx + vpy*vpy + vpz*vpz;
            if (A >= 1e-30) {
                double B    = ppx*vpx + ppy*vpy + ppz*vpz;
                double C    = ppx*ppx + ppy*ppy + ppz*ppz - R*R;
                double disc = B*B - A*C;

                if (disc >= 0) {
                    double sqrtD  = std::sqrt(std::max(disc, 0.0));
                    bool   tang   = (sqrtD < octTol * std::sqrt(A));
                    bool   fullAng = (fe.u1 - fe.u0 >= 2.0*M_PI - octTol);
                    bool   fwd    = (fe.face.Orientation() != TopAbs_REVERSED);

                    auto tryRoot = [&](double t) {
                        if (t < -octTol) return;
                        double v_ax = da + t * va_;
                        if (v_ax < fe.v0 - octTol || v_ax > fe.v1 + octTol) return;
                        double qx = ppx+t*vpx, qy = ppy+t*vpy, qz = ppz+t*vpz;
                        double r  = std::sqrt(qx*qx + qy*qy + qz*qz);
                        if (r < 1e-15) return;
                        // Always compute uf (needed for correct normal in calcNorm path)
                        const gp_Dir& xu_c = ax.XDirection();
                        const gp_Dir& yv_c = ax.YDirection();
                        double uf = std::atan2(dot3(yv_c.X(),yv_c.Y(),yv_c.Z(), qx,qy,qz),
                                               dot3(xu_c.X(),xu_c.Y(),xu_c.Z(), qx,qy,qz));
                        // Normalize to [u0, u0+2π]
                        if (uf < fe.u0 - M_PI) uf += 2.0*M_PI;
                        if (!fullAng && (uf < fe.u0 - octTol || uf > fe.u1 + octTol)) return;
                        IntCurveSurface_TransitionOnCurve trans;
                        if (tang) {
                            trans = IntCurveSurface_Tangent;
                        } else {
                            double n_dot_v = dot3(qx,qy,qz, v.x(),v.y(),v.z()) / r;
                            double n_out_v = fwd ? n_dot_v : -n_dot_v;
                            if      (n_out_v < -octTol) trans = IntCurveSurface_In;
                            else if (n_out_v >  octTol) trans = IntCurveSurface_Out;
                            else                        trans = IntCurveSurface_Tangent;
                        }
                        rawHits.emplace_back(t, trans, fe.face, uf, v_ax);
                        if (fTimingEnabled) algo->timing.rayHitsTotal += 1;
                    };

                    if (tang) {
                        tryRoot(-B / A);
                    } else {
                        tryRoot((-B - sqrtD) / A);
                        tryRoot((-B + sqrtD) / A);
                    }
                }
            }
            analytic = true;
            if (fTimingEnabled) ++algo->timing.rayAnalyticCount;

        // ─── Sphere ──────────────────────────────────────────────────────
        } else if (fe.surfType == FaceEntry::SurfType::Sphere) {
            const gp_Pnt& cen = fe.pos.Location();
            const double   R  = fe.sphR;

            double dpx = p.x()-cen.X(), dpy = p.y()-cen.Y(), dpz = p.z()-cen.Z();
            double A    = v.x()*v.x() + v.y()*v.y() + v.z()*v.z();  // ≈ 1 for unit v
            double B    = dpx*v.x() + dpy*v.y() + dpz*v.z();
            double C    = dpx*dpx + dpy*dpy + dpz*dpz - R*R;
            double disc = B*B - A*C;

            if (disc >= 0) {
                double sqrtD     = std::sqrt(std::max(disc, 0.0));
                bool   tang      = (sqrtD < octTol * std::sqrt(A));
                bool   fullSph   = (fe.u1-fe.u0 >= 2.0*M_PI-octTol &&
                                    fe.v1-fe.v0 >= M_PI-octTol);
                bool   fwd       = (fe.face.Orientation() != TopAbs_REVERSED);

                auto tryRoot = [&](double t) {
                    if (t < -octTol) return;
                    double qx = dpx+t*v.x(), qy = dpy+t*v.y(), qz = dpz+t*v.z();
                    double r  = std::sqrt(qx*qx + qy*qy + qz*qz);
                    if (r < 1e-15) return;
                    double uf = 0.0, vf = 0.0;
                    if (!fullSph) {
                        const gp_Dir& xu   = fe.pos.XDirection();
                        const gp_Dir& yv_  = fe.pos.YDirection();
                        const gp_Dir& zv_  = fe.pos.Direction();
                        double xl = dot3(xu.X(),xu.Y(),xu.Z(), qx,qy,qz);
                        double yl = dot3(yv_.X(),yv_.Y(),yv_.Z(), qx,qy,qz);
                        double zl = dot3(zv_.X(),zv_.Y(),zv_.Z(), qx,qy,qz);
                        uf = std::atan2(yl, xl);
                        vf = std::asin(std::max(-1.0, std::min(1.0, zl/r)));
                        if (uf < fe.u0 - octTol) uf += 2.0*M_PI;
                        if (uf < fe.u0 - octTol || uf > fe.u1 + octTol) return;
                        if (vf < fe.v0 - octTol || vf > fe.v1 + octTol) return;
                    }
                    IntCurveSurface_TransitionOnCurve trans;
                    if (tang) {
                        trans = IntCurveSurface_Tangent;
                    } else {
                        double n_dot_v = dot3(qx,qy,qz, v.x(),v.y(),v.z()) / r;
                        double n_out_v = fwd ? n_dot_v : -n_dot_v;
                        if      (n_out_v < -octTol) trans = IntCurveSurface_In;
                        else if (n_out_v >  octTol) trans = IntCurveSurface_Out;
                        else                        trans = IntCurveSurface_Tangent;
                    }
                    rawHits.emplace_back(t, trans, fe.face, uf, vf);
                    if (fTimingEnabled) algo->timing.rayHitsTotal += 1;
                };

                if (tang) {
                    tryRoot(-B / A);
                } else {
                    tryRoot((-B - sqrtD) / A);
                    tryRoot((-B + sqrtD) / A);
                }
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

    GroupHits(rawHits, algo->scratchGroups, octTol);
    for (const auto& g : algo->scratchGroups) {
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
