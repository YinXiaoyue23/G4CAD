#include "OutputWriter.hh"
#include <iomanip>
#include <stdexcept>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <numeric>

static std::string DirOf(const std::string& prefix) {
    auto pos = prefix.rfind('/');
    return (pos == std::string::npos) ? "." : prefix.substr(0, pos);
}

OutputWriter::OutputWriter(const std::string& prefix, bool enable_root)
    : fPrefix(prefix), fEnableRoot(enable_root)
{
    std::string dir = DirOf(prefix);
    if (dir != ".") std::filesystem::create_directories(dir);
}

OutputWriter::~OutputWriter() { Flush(); }

void OutputWriter::EnsureOpen(std::ofstream& f,
                               const std::string& filename,
                               const std::string& header) {
    if (!f.is_open()) {
        bool exists = std::filesystem::exists(filename);
        f.open(filename, std::ios::app);
        if (!f) throw std::runtime_error("Cannot open: " + filename);
        if (!exists) f << header << "\n";
    }
}

void OutputWriter::WriteGeometrySummary(const GeometrySummaryRow& r) {
    EnsureOpen(fGeomFile, fPrefix + "_geometry_summary.csv",
        "geometry,mode,volume_mm3,analytical_volume_mm3,volume_rel_error,"
        "bbox_xmin_mm,bbox_xmax_mm,bbox_ymin_mm,bbox_ymax_mm,bbox_zmin_mm,bbox_zmax_mm,"
        "n_solids,tolerance_mm");
    fGeomFile << std::setprecision(8)
              << r.geometry << "," << r.mode << ","
              << r.volume_mm3 << "," << r.analytical_volume_mm3 << ","
              << r.volume_rel_error << ","
              << r.bbox_xmin << "," << r.bbox_xmax << ","
              << r.bbox_ymin << "," << r.bbox_ymax << ","
              << r.bbox_zmin << "," << r.bbox_zmax << ","
              << r.n_solids << "," << r.tolerance_mm << "\n";
}

void OutputWriter::WriteNavigationTest(const NavigationTestRow& r) {
    EnsureOpen(fNavFile, fPrefix + "_navigation_test.csv",
        "geometry,mode_pair,n_points,n_rays,mismatch_rate,"
        "mean_dev_in_mm,median_dev_in_mm,max_dev_in_mm,"
        "mean_dev_out_mm,median_dev_out_mm,max_dev_out_mm,"
        "n_problematic_in,n_problematic_out");
    fNavFile << std::setprecision(8)
             << r.geometry << "," << r.mode_pair << ","
             << r.n_points << "," << r.n_rays << ","
             << r.mismatch_rate << ","
             << r.mean_dev_in << "," << r.median_dev_in << "," << r.max_dev_in << ","
             << r.mean_dev_out << "," << r.median_dev_out << "," << r.max_dev_out << ","
             << r.n_problematic_in << "," << r.n_problematic_out << "\n";
}

void OutputWriter::WriteEventSummary(const EventSummaryRow& r) {
    EnsureOpen(fEventFile, fPrefix + "_event_summary.csv",
        "geometry,mode,event_id,edep_MeV,track_length_mm,n_steps,n_boundary_crossings");
    fEventFile << std::setprecision(8)
               << r.geometry << "," << r.mode << "," << r.event_id << ","
               << r.edep_MeV << "," << r.track_length_mm << ","
               << r.n_steps << "," << r.n_boundary_crossings << "\n";
}

void OutputWriter::WriteEventSummaryThreadSafe(const EventSummaryRow& r) {
    std::lock_guard<std::mutex> lock(fEventMutex);
    WriteEventSummary(r);
    // Update running stats for this geometry-mode combination
    EventRunStats* stats = nullptr;
    for (auto& s : fRunStats)
        if (s.geometry == r.geometry && s.mode == r.mode) { stats = &s; break; }
    if (!stats) {
        fRunStats.push_back({r.geometry, r.mode});
        stats = &fRunStats.back();
    }
    stats->count++;
    stats->sum_edep   += r.edep_MeV;
    stats->sum2_edep  += r.edep_MeV * r.edep_MeV;
    stats->sum_len    += r.track_length_mm;
    stats->sum2_len   += r.track_length_mm * r.track_length_mm;
    stats->sum_steps  += r.n_steps;
    stats->sum2_steps += (double)r.n_steps * r.n_steps;
    stats->sum_bcx    += r.n_boundary_crossings;
    stats->sum2_bcx   += (double)r.n_boundary_crossings * r.n_boundary_crossings;
}

void OutputWriter::WritePhysicsSummary(const PhysicsSummaryRow& r) {
    EnsureOpen(fPhysicsFile, fPrefix + "_physics_summary.csv",
        "geometry,mode,n_events,"
        "edep_mean_MeV,edep_std_MeV,edep_se_MeV,"
        "track_len_mean_mm,track_len_std_mm,track_len_se_mm,"
        "steps_mean,steps_std,steps_se,"
        "bcx_mean,bcx_std,bcx_se");
    fPhysicsFile << std::setprecision(8)
                 << r.geometry << "," << r.mode << "," << r.n_events << ","
                 << r.edep_mean_MeV     << "," << r.edep_std_MeV     << "," << r.edep_se_MeV     << ","
                 << r.track_len_mean_mm << "," << r.track_len_std_mm << "," << r.track_len_se_mm << ","
                 << r.steps_mean        << "," << r.steps_std        << "," << r.steps_se        << ","
                 << r.bcx_mean          << "," << r.bcx_std          << "," << r.bcx_se          << "\n";
}

void OutputWriter::WriteScalingSummary(const ScalingSummaryRow& r) {
    EnsureOpen(fScaleFile, fPrefix + "_scaling_summary.csv",
        "geometry,mode,n_threads,n_events,wall_time_s,events_per_sec,"
        "speedup,efficiency_pct,mem_rss_mb,nav_warnings");
    fScaleFile << std::setprecision(8)
               << r.geometry << "," << r.mode << ","
               << r.n_threads << "," << r.n_events << ","
               << r.wall_time_s << "," << r.events_per_sec << ","
               << r.speedup << "," << r.efficiency_pct << ","
               << r.mem_rss_mb << "," << r.nav_warnings << "\n";
}

void OutputWriter::WriteCrossSection(const std::string& geometry,
                                      const std::string& mode,
                                      const std::string& plane,
                                      const std::vector<CrossSectionPoint>& points) {
    std::string filename = fPrefix + "_cross_section_" + geometry + "_" + mode + "_" + plane + ".csv";
    std::ofstream f(filename);
    if (!f) throw std::runtime_error("Cannot open: " + filename);
    f << "x_mm,y_mm,z_mm,classification\n";
    f << std::setprecision(5);
    for (const auto& p : points) {
        f << p.x << "," << p.y << "," << p.z << "," << (int)p.classification << "\n";
    }
}

void OutputWriter::WriteSafetyDiagnostic(const std::vector<SafetyDiagnosticPoint>& points,
                                          const std::string& geometry,
                                          const std::string& mode) {
    std::string filename = fPrefix + "_safety_diagnostic_" + geometry + "_" + mode + ".csv";
    std::ofstream f(filename);
    if (!f) throw std::runtime_error("Cannot open: " + filename);
    f << "x_mm,y_mm,z_mm,source,native_mm,step_mm,ratio\n";
    f << std::setprecision(6);
    for (const auto& p : points) {
        f << p.x << "," << p.y << "," << p.z << ","
          << (int)p.source << ","
          << p.native_mm << "," << p.step_mm << "," << p.ratio << "\n";
    }
}

void OutputWriter::WriteSafetySummary(const SafetySummaryRow& r) {
    EnsureOpen(fSafetySummaryFile, fPrefix + "_safety_summary.csv",
        "geometry,mode,n_points_random,n_points_path,n_points_used,"
        "ratio_mean,ratio_median,ratio_p05,ratio_p95,"
        "frac_lt_0p1,frac_lt_0p01,frac_lt_0p001,"
        "native_safety_mean_mm,step_safety_mean_mm");
    fSafetySummaryFile << std::setprecision(8)
                       << r.geometry << "," << r.mode << ","
                       << r.n_points_random << "," << r.n_points_path << "," << r.n_points_used << ","
                       << r.ratio_mean    << "," << r.ratio_median << ","
                       << r.ratio_p05     << "," << r.ratio_p95    << ","
                       << r.frac_lt_0p1   << "," << r.frac_lt_0p01  << "," << r.frac_lt_0p001 << ","
                       << r.native_safety_mean_mm << "," << r.step_safety_mean_mm << "\n";
}

void OutputWriter::WriteComplexCaseSummary(const ComplexCaseSummaryRow& r) {
    EnsureOpen(fComplexFile, fPrefix + "_complex_case.csv",
        "step_file,n_solids,load_time_s,mem_before_mb,mem_after_mb,"
        "bbox_xmin_mm,bbox_xmax_mm,bbox_ymin_mm,bbox_ymax_mm,bbox_zmin_mm,bbox_zmax_mm,"
        "volume_mm3,wall_time_s,events_per_sec,n_events");
    fComplexFile << std::setprecision(8)
                 << r.step_file << "," << r.n_solids << ","
                 << r.load_time_s << ","
                 << r.mem_before_mb << "," << r.mem_after_mb << ","
                 << r.bbox_xmin << "," << r.bbox_xmax << ","
                 << r.bbox_ymin << "," << r.bbox_ymax << ","
                 << r.bbox_zmin << "," << r.bbox_zmax << ","
                 << r.volume_mm3 << ","
                 << r.wall_time_s << "," << r.events_per_sec << "," << r.n_events << "\n";
}

static double SafeStd(double sum, double sum2, int n) {
    if (n < 2) return 0.0;
    double mean = sum / n;
    double var  = sum2 / n - mean * mean;
    return (var > 0.0) ? std::sqrt(var) : 0.0;
}

void OutputWriter::Flush() {
    // Write physics summary rows from accumulated stats
    for (const auto& s : fRunStats) {
        if (s.count == 0) continue;
        PhysicsSummaryRow row;
        row.geometry = s.geometry;
        row.mode     = s.mode;
        row.n_events = s.count;
        double n = s.count;
        row.edep_mean_MeV     = s.sum_edep  / n;
        row.edep_std_MeV      = SafeStd(s.sum_edep,  s.sum2_edep,  s.count);
        row.edep_se_MeV       = row.edep_std_MeV / std::sqrt(n);
        row.track_len_mean_mm = s.sum_len   / n;
        row.track_len_std_mm  = SafeStd(s.sum_len,   s.sum2_len,   s.count);
        row.track_len_se_mm   = row.track_len_std_mm / std::sqrt(n);
        row.steps_mean        = s.sum_steps / n;
        row.steps_std         = SafeStd(s.sum_steps, s.sum2_steps, s.count);
        row.steps_se          = row.steps_std / std::sqrt(n);
        row.bcx_mean          = s.sum_bcx   / n;
        row.bcx_std           = SafeStd(s.sum_bcx,   s.sum2_bcx,   s.count);
        row.bcx_se            = row.bcx_std / std::sqrt(n);
        WritePhysicsSummary(row);
    }
    fRunStats.clear();

    if (fGeomFile.is_open())         { fGeomFile.flush();         fGeomFile.close(); }
    if (fNavFile.is_open())          { fNavFile.flush();           fNavFile.close(); }
    if (fEventFile.is_open())        { fEventFile.flush();         fEventFile.close(); }
    if (fPhysicsFile.is_open())      { fPhysicsFile.flush();       fPhysicsFile.close(); }
    if (fScaleFile.is_open())        { fScaleFile.flush();         fScaleFile.close(); }
    if (fSafetySummaryFile.is_open()){ fSafetySummaryFile.flush(); fSafetySummaryFile.close(); }
    if (fComplexFile.is_open())      { fComplexFile.flush();       fComplexFile.close(); }
}
