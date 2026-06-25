#include "OutputWriter.hh"
#include <iomanip>
#include <stdexcept>
#include <filesystem>

static std::string DirOf(const std::string& prefix) {
    auto pos = prefix.rfind('/');
    return (pos == std::string::npos) ? "." : prefix.substr(0, pos);
}

OutputWriter::OutputWriter(const std::string& prefix, bool enable_root)
    : fPrefix(prefix), fEnableRoot(enable_root)
{
    // Create output directory if needed
    std::string dir = DirOf(prefix);
    if (dir != ".") std::filesystem::create_directories(dir);
}

OutputWriter::~OutputWriter() { Flush(); }

void OutputWriter::EnsureOpen(std::ofstream& f,
                               const std::string& filename,
                               const std::string& header) {
    if (!f.is_open()) {
        f.open(filename);
        if (!f) throw std::runtime_error("Cannot open: " + filename);
        f << header << "\n";
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

void OutputWriter::Flush() {
    if (fGeomFile.is_open())  { fGeomFile.flush();  fGeomFile.close(); }
    if (fNavFile.is_open())   { fNavFile.flush();   fNavFile.close(); }
    if (fEventFile.is_open()) { fEventFile.flush(); fEventFile.close(); }
    if (fScaleFile.is_open()) { fScaleFile.flush(); fScaleFile.close(); }
}
