#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <mutex>

struct GeometrySummaryRow {
    std::string geometry, mode;
    double volume_mm3, analytical_volume_mm3, volume_rel_error;
    double bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax, bbox_zmin, bbox_zmax;
    int    n_solids;
    double tolerance_mm;
};

struct NavigationTestRow {
    std::string geometry, mode_pair;
    int    n_points, n_rays;
    double mismatch_rate;
    double mean_dev_in, median_dev_in, max_dev_in;
    double mean_dev_out, median_dev_out, max_dev_out;
    int    n_problematic_in, n_problematic_out;
};

struct EventSummaryRow {
    std::string geometry, mode;
    int    event_id;
    double edep_MeV, track_length_mm;
    int    n_steps, n_boundary_crossings;
};

struct ScalingSummaryRow {
    std::string geometry, mode;
    int    n_threads, n_events;
    double wall_time_s, events_per_sec, speedup, efficiency_pct;
    double mem_rss_mb;
    int    nav_warnings;
};

struct CrossSectionPoint {
    float  x, y, z;
    int8_t classification;  // 0=Outside, 1=Surface, 2=Inside
};

class OutputWriter {
public:
    OutputWriter(const std::string& prefix, bool enable_root = false);
    ~OutputWriter();

    void WriteGeometrySummary(const GeometrySummaryRow& row);
    void WriteNavigationTest(const NavigationTestRow& row);
    void WriteEventSummary(const EventSummaryRow& row);
    void WriteScalingSummary(const ScalingSummaryRow& row);
    void WriteCrossSection(const std::string& geometry,
                           const std::string& mode,
                           const std::string& plane,
                           const std::vector<CrossSectionPoint>& points);

    void Flush();

    // Thread-safe variant for MT physics runs
    void WriteEventSummaryThreadSafe(const EventSummaryRow& row);

private:
    std::string   fPrefix;
    bool          fEnableRoot;

    std::ofstream fGeomFile;
    std::ofstream fNavFile;
    std::ofstream fEventFile;
    std::ofstream fScaleFile;
    std::mutex    fEventMutex;

    void EnsureOpen(std::ofstream& f, const std::string& filename, const std::string& header);
};
