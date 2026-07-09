#include "SafetyDiagnostic.hh"
#include "CLHEP/Random/RanluxEngine.h"
#include "CLHEP/Random/RandFlat.h"
#include "G4PhysicalConstants.hh"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <iostream>

SafetyDiagnostic::SafetyDiagnostic(int nRandom, int nPath)
    : fNRandom(nRandom), fNPath(nPath) {}

int SafetyDiagnostic::FindContainingSolid(const std::vector<G4VSolid*>& solids,
                                           const G4ThreeVector& p) {
    for (int i = 0; i < (int)solids.size(); ++i)
        if (solids[i]->Inside(p) == kInside) return i;
    return -1;
}

void SafetyDiagnostic::Run(
    const std::vector<G4VSolid*>& native_solids,
    const std::vector<G4VSolid*>& step_solids,
    const G4ThreeVector& bbox_min,
    const G4ThreeVector& bbox_max,
    const std::string& geometry_name,
    const std::string& mode,
    long seed,
    OutputWriter& writer)
{
    CLHEP::RanluxEngine eng(seed + 999);  // different seed from navigation test

    std::vector<SafetyDiagnosticPoint> pts;
    pts.reserve(fNRandom + fNPath);

    // Primary gun ray: (30,0,-200) + t*(0,0,1)
    const G4ThreeVector gun_pos(30.0, 0.0, -200.0);
    const G4ThreeVector gun_dir(0.0,  0.0,   1.0);
    const double PATH_LENGTH = 800.0;  // mm

    // ---- Random interior points ----
    int n_random_got = 0;
    const int MAX_ATTEMPTS_RANDOM = fNRandom * 25;
    for (int attempt = 0; attempt < MAX_ATTEMPTS_RANDOM && n_random_got < fNRandom; ++attempt) {
        G4ThreeVector p(
            bbox_min.x() + CLHEP::RandFlat::shoot(&eng) * (bbox_max.x() - bbox_min.x()),
            bbox_min.y() + CLHEP::RandFlat::shoot(&eng) * (bbox_max.y() - bbox_min.y()),
            bbox_min.z() + CLHEP::RandFlat::shoot(&eng) * (bbox_max.z() - bbox_min.z())
        );
        int ni = FindContainingSolid(native_solids, p);
        if (ni < 0) continue;
        int si = FindContainingSolid(step_solids, p);
        if (si < 0) continue;

        G4double d_native = native_solids[ni]->DistanceToOut(p);
        G4double d_step   = step_solids[si]->DistanceToOut(p);

        // Only skip if native cannot be determined (avoids division by zero).
        // Step safety of 0 is recorded (it IS the underestimation we want to measure).
        if (!std::isfinite(d_native) || d_native <= 0.0) continue;
        if (!std::isfinite(d_step))                      continue;

        SafetyDiagnosticPoint pt;
        pt.x        = (float)p.x();
        pt.y        = (float)p.y();
        pt.z        = (float)p.z();
        pt.source   = 0;
        pt.native_mm = (float)d_native;
        pt.step_mm   = (float)d_step;
        pt.ratio     = (float)(d_step / d_native);
        pts.push_back(pt);
        ++n_random_got;
    }

    // ---- Path-following points ----
    int n_path_got = 0;
    const int MAX_ATTEMPTS_PATH = fNPath * 20;
    for (int attempt = 0; attempt < MAX_ATTEMPTS_PATH && n_path_got < fNPath; ++attempt) {
        double t = CLHEP::RandFlat::shoot(&eng) * PATH_LENGTH;
        G4ThreeVector p = gun_pos + t * gun_dir;

        int ni = FindContainingSolid(native_solids, p);
        if (ni < 0) continue;
        int si = FindContainingSolid(step_solids, p);
        if (si < 0) continue;

        G4double d_native = native_solids[ni]->DistanceToOut(p);
        G4double d_step   = step_solids[si]->DistanceToOut(p);

        if (!std::isfinite(d_native) || d_native <= 0.0) continue;
        if (!std::isfinite(d_step))                      continue;

        SafetyDiagnosticPoint pt;
        pt.x        = (float)p.x();
        pt.y        = (float)p.y();
        pt.z        = (float)p.z();
        pt.source   = 1;
        pt.native_mm = (float)d_native;
        pt.step_mm   = (float)d_step;
        pt.ratio     = (float)(d_step / d_native);
        pts.push_back(pt);
        ++n_path_got;
    }

    writer.WriteSafetyDiagnostic(pts, geometry_name, mode);

    if (pts.empty()) {
        std::cout << "[SafetyDiagnostic] " << geometry_name
                  << " (" << mode << ") no valid points found\n";
        return;
    }

    // ---- Compute summary statistics ----
    std::vector<double> ratios;
    ratios.reserve(pts.size());
    double sum_native = 0.0, sum_step = 0.0;
    int n_random_used = 0, n_path_used = 0;
    for (const auto& pt : pts) {
        ratios.push_back((double)pt.ratio);
        sum_native += pt.native_mm;
        sum_step   += pt.step_mm;
        if (pt.source == 0) ++n_random_used;
        else                ++n_path_used;
    }

    std::sort(ratios.begin(), ratios.end());
    const int N = (int)ratios.size();

    double ratio_mean   = std::accumulate(ratios.begin(), ratios.end(), 0.0) / N;
    double ratio_median = (N % 2 == 0) ? 0.5*(ratios[N/2-1] + ratios[N/2]) : ratios[N/2];
    double ratio_p05    = ratios[std::max(0, (int)(0.05 * N))];
    double ratio_p95    = ratios[std::min(N-1, (int)(0.95 * N))];

    int lt1 = 0, lt01 = 0, lt001 = 0;
    for (double r : ratios) {
        if (r < 0.1)   ++lt1;
        if (r < 0.01)  ++lt01;
        if (r < 0.001) ++lt001;
    }

    SafetySummaryRow row;
    row.geometry              = geometry_name;
    row.mode                  = mode;
    row.n_points_random       = n_random_used;
    row.n_points_path         = n_path_used;
    row.n_points_used         = N;
    row.ratio_mean            = ratio_mean;
    row.ratio_median          = ratio_median;
    row.ratio_p05             = ratio_p05;
    row.ratio_p95             = ratio_p95;
    row.frac_lt_0p1           = (double)lt1  / N;
    row.frac_lt_0p01          = (double)lt01 / N;
    row.frac_lt_0p001         = (double)lt001 / N;
    row.native_safety_mean_mm = sum_native / N;
    row.step_safety_mean_mm   = sum_step   / N;
    writer.WriteSafetySummary(row);

    std::cout << "[SafetyDiagnostic] " << geometry_name << " (" << mode << ")"
              << "  n=" << N
              << "  ratio_median=" << ratio_median
              << "  ratio_p05=" << ratio_p05
              << "  ratio_p95=" << ratio_p95
              << "  frac_lt_0.1=" << row.frac_lt_0p1
              << "\n";
}
