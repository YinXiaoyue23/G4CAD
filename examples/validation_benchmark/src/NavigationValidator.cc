#include "NavigationValidator.hh"
#include "CLHEP/Random/RanluxEngine.h"
#include "CLHEP/Random/RandFlat.h"
#include "CLHEP/Units/PhysicalConstants.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

NavigationValidator::NavigationValidator(int n_points, int n_rays,
                                          double tol, bool dump)
    : fNPoints(n_points), fNRays(n_rays), fTol(tol), fDump(dump) {}

EInside NavigationValidator::InsideUnion(const std::vector<G4VSolid*>& solids,
                                          const G4ThreeVector& p) const {
    EInside result = kOutside;
    for (auto* s : solids) {
        EInside r = s->Inside(p);
        if (r == kInside)  return kInside;
        if (r == kSurface) result = kSurface;
    }
    return result;
}

double NavigationValidator::DistToInMin(const std::vector<G4VSolid*>& solids,
                                         const G4ThreeVector& p,
                                         const G4ThreeVector& v) const {
    double dmin = kInfinity;
    for (auto* s : solids) {
        double d = s->DistanceToIn(p, v);
        if (d < dmin) dmin = d;
    }
    return dmin;
}

double NavigationValidator::DistToOutContaining(const std::vector<G4VSolid*>& solids,
                                                  const G4ThreeVector& p,
                                                  const G4ThreeVector& v) const {
    for (auto* s : solids) {
        if (s->Inside(p) == kInside)
            return s->DistanceToOut(p, v);
    }
    return kInfinity;
}

double NavigationValidator::Median(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    std::size_t n = v.size();
    return (n % 2 == 0) ? 0.5*(v[n/2-1] + v[n/2]) : v[n/2];
}

NavValidationResult NavigationValidator::Run(
    const std::vector<G4VSolid*>& native_solids,
    const std::vector<G4VSolid*>& step_solids,
    const G4ThreeVector& bbox_min,
    const G4ThreeVector& bbox_max,
    const std::string& geometry_name,
    long seed)
{
    NavValidationResult result;
    result.geometry_name = geometry_name;
    result.n_points      = fNPoints;
    result.n_rays        = fNRays;

    // Expand bbox by 10%
    G4ThreeVector span = bbox_max - bbox_min;
    G4ThreeVector lo   = bbox_min - 0.1 * span;
    G4ThreeVector hi   = bbox_max + 0.1 * span;

    CLHEP::RanluxEngine eng(seed);

    // -------- Point classification test --------
    int mismatch = 0;
    for (int i = 0; i < fNPoints; ++i) {
        G4ThreeVector p(
            lo.x() + CLHEP::RandFlat::shoot(&eng) * (hi.x() - lo.x()),
            lo.y() + CLHEP::RandFlat::shoot(&eng) * (hi.y() - lo.y()),
            lo.z() + CLHEP::RandFlat::shoot(&eng) * (hi.z() - lo.z())
        );
        EInside r_native = InsideUnion(native_solids, p);
        EInside r_step   = InsideUnion(step_solids,   p);
        if (r_native != r_step) {
            ++mismatch;
            if (fDump) result.dump_points.push_back(p);
        }
    }
    result.mismatch_rate = static_cast<double>(mismatch) / fNPoints;

    // -------- Ray distance test --------
    std::vector<double> devs_in, devs_out;
    devs_in.reserve(fNRays);
    devs_out.reserve(fNRays);

    // Larger region to ensure rays start outside
    G4ThreeVector lo2 = bbox_min - 0.3 * span;
    G4ThreeVector hi2 = bbox_max + 0.3 * span;

    for (int i = 0; i < fNRays; ++i) {
        // Random point outside geometry (in expanded bbox)
        G4ThreeVector p(
            lo2.x() + CLHEP::RandFlat::shoot(&eng) * (hi2.x() - lo2.x()),
            lo2.y() + CLHEP::RandFlat::shoot(&eng) * (hi2.y() - lo2.y()),
            lo2.z() + CLHEP::RandFlat::shoot(&eng) * (hi2.z() - lo2.z())
        );
        // Skip if inside
        if (InsideUnion(native_solids, p) != kOutside) continue;

        // Random unit direction
        double phi   = CLHEP::RandFlat::shoot(&eng) * CLHEP::twopi;
        double costh = 2.0 * CLHEP::RandFlat::shoot(&eng) - 1.0;
        double sinth = std::sqrt(1.0 - costh*costh);
        G4ThreeVector v(sinth*std::cos(phi), sinth*std::sin(phi), costh);

        double d_native = DistToInMin(native_solids, p, v);
        double d_step   = DistToInMin(step_solids,   p, v);

        if (d_native < kInfinity && d_step < kInfinity) {
            double dev = std::abs(d_native - d_step);
            devs_in.push_back(dev);
            if (dev > fTol) ++result.n_problematic_in;

            // DistanceToOut test: step inside along native ray
            G4ThreeVector p_in = p + (d_native * 1.001) * v;
            if (InsideUnion(native_solids, p_in) == kInside) {
                double dout_native = DistToOutContaining(native_solids, p_in, v);
                double dout_step   = DistToOutContaining(step_solids,   p_in, v);
                if (dout_native < kInfinity && dout_step < kInfinity) {
                    double devout = std::abs(dout_native - dout_step);
                    devs_out.push_back(devout);
                    if (devout > fTol) ++result.n_problematic_out;
                }
            }
        }
    }

    // Statistics for DistanceToIn
    if (!devs_in.empty()) {
        result.mean_dev_in   = std::accumulate(devs_in.begin(), devs_in.end(), 0.0) / devs_in.size();
        result.median_dev_in = Median(devs_in);
        result.max_dev_in    = *std::max_element(devs_in.begin(), devs_in.end());
    }

    // Statistics for DistanceToOut
    if (!devs_out.empty()) {
        result.mean_dev_out   = std::accumulate(devs_out.begin(), devs_out.end(), 0.0) / devs_out.size();
        result.median_dev_out = Median(devs_out);
        result.max_dev_out    = *std::max_element(devs_out.begin(), devs_out.end());
    }

    std::cout << "[NavigationValidator] " << geometry_name
              << "  mismatch=" << result.mismatch_rate*100 << "%"
              << "  max_dev_in=" << result.max_dev_in << " mm"
              << "  max_dev_out=" << result.max_dev_out << " mm\n";

    return result;
}
