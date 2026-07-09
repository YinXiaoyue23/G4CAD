#pragma once
#include <string>
#include <fstream>
#include <vector>
#include <mutex>
#include <cstdint>

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

// Aggregate statistics for one geometry-mode physics run (Feature 1)
struct PhysicsSummaryRow {
    std::string geometry, mode;
    int    n_events;
    double edep_mean_MeV,     edep_std_MeV,     edep_se_MeV;
    double track_len_mean_mm, track_len_std_mm, track_len_se_mm;
    double steps_mean,        steps_std,        steps_se;
    double bcx_mean,          bcx_std,          bcx_se;
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
    int8_t classification;  // 0=Outside, 1=Surface, 2=Inside (or 0=match, 1=mismatch)
};

// Per-point output for safety diagnostic (Feature 2)
struct SafetyDiagnosticPoint {
    float  x, y, z;
    int8_t source;      // 0=random interior, 1=along primary ray
    float  native_mm;
    float  step_mm;
    float  ratio;       // step_mm / native_mm
};

// Aggregate statistics for safety diagnostic (Feature 2)
struct SafetySummaryRow {
    std::string geometry, mode;
    int    n_points_random, n_points_path, n_points_used;
    double ratio_mean, ratio_median, ratio_p05, ratio_p95;
    double frac_lt_0p1, frac_lt_0p01, frac_lt_0p001;
    double native_safety_mean_mm, step_safety_mean_mm;
};

// Complex STEP application case (Feature 6)
struct ComplexCaseSummaryRow {
    std::string step_file;
    int    n_solids;
    double load_time_s;
    double mem_before_mb, mem_after_mb;
    double bbox_xmin, bbox_xmax, bbox_ymin, bbox_ymax, bbox_zmin, bbox_zmax;
    double volume_mm3;
    double wall_time_s;
    double events_per_sec;
    int    n_events;
};

class OutputWriter {
public:
    OutputWriter(const std::string& prefix, bool enable_root = false);
    ~OutputWriter();

    void WriteGeometrySummary(const GeometrySummaryRow& row);
    void WriteNavigationTest(const NavigationTestRow& row);
    void WriteEventSummary(const EventSummaryRow& row);
    void WritePhysicsSummary(const PhysicsSummaryRow& row);
    void WriteScalingSummary(const ScalingSummaryRow& row);
    void WriteCrossSection(const std::string& geometry,
                           const std::string& mode,
                           const std::string& plane,
                           const std::vector<CrossSectionPoint>& points);
    void WriteSafetyDiagnostic(const std::vector<SafetyDiagnosticPoint>& points,
                                const std::string& geometry,
                                const std::string& mode);
    void WriteSafetySummary(const SafetySummaryRow& row);
    void WriteComplexCaseSummary(const ComplexCaseSummaryRow& row);

    void Flush();

    // Thread-safe variant for MT physics runs; also updates running stats.
    void WriteEventSummaryThreadSafe(const EventSummaryRow& row);

private:
    std::string   fPrefix;
    bool          fEnableRoot;

    std::ofstream fGeomFile;
    std::ofstream fNavFile;
    std::ofstream fEventFile;
    std::ofstream fPhysicsFile;
    std::ofstream fScaleFile;
    std::ofstream fSafetySummaryFile;
    std::ofstream fComplexFile;
    std::mutex    fEventMutex;

    // Running stats for physics summary — updated in WriteEventSummaryThreadSafe
    struct EventRunStats {
        std::string geometry, mode;
        int    count    = 0;
        double sum_edep = 0, sum2_edep = 0;
        double sum_len  = 0, sum2_len  = 0;
        double sum_steps = 0, sum2_steps = 0;
        double sum_bcx   = 0, sum2_bcx  = 0;
    };
    std::vector<EventRunStats> fRunStats;  // protected by fEventMutex

    void EnsureOpen(std::ofstream& f, const std::string& filename, const std::string& header);
};
