#include "BenchmarkRunner.hh"
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>

BenchmarkRunner::BenchmarkRunner(OutputWriter* writer,
                                   const PhysicsConfig& baseCfg,
                                   std::vector<G4VSolid*> solids)
    : fWriter(writer), fBaseCfg(baseCfg), fSolids(std::move(solids)) {}

double BenchmarkRunner::ReadMemRSS_MB() {
#ifdef __linux__
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("VmRSS:", 0) == 0) {
            std::istringstream ss(line.substr(6));
            double kb = 0;
            ss >> kb;
            return kb / 1024.0;
        }
    }
#endif
    return -1.0;
}

void BenchmarkRunner::Run() {
    const std::vector<int> threadCounts = {1, 2, 4, 8, 16};
    int maxThreads = static_cast<int>(std::thread::hardware_concurrency());

    double baseline_eps = -1.0;

    for (int nt : threadCounts) {
        if (nt > maxThreads) {
            std::cout << "[BenchmarkRunner] Skipping " << nt
                      << " threads (max=" << maxThreads << ")\n";
            continue;
        }

        PhysicsConfig cfg = fBaseCfg;
        cfg.n_threads     = nt;

        PhysicsRunner runner(fWriter, cfg);
        double wallTime = runner.Run(fSolids);

        double eps     = (wallTime > 0) ? cfg.n_events / wallTime : 0.0;
        if (baseline_eps < 0) baseline_eps = eps;

        double speedup    = (baseline_eps > 0) ? eps / baseline_eps : 1.0;
        double efficiency = (nt > 0) ? speedup / nt * 100.0 : 0.0;

        ScalingSummaryRow row;
        row.geometry      = cfg.geometry_name;
        row.mode          = cfg.mode;
        row.n_threads     = nt;
        row.n_events      = cfg.n_events;
        row.wall_time_s   = wallTime;
        row.events_per_sec = eps;
        row.speedup       = speedup;
        row.efficiency_pct = efficiency;
        row.mem_rss_mb    = ReadMemRSS_MB();
        row.nav_warnings  = 0;  // future: hook G4cerr counter

        fWriter->WriteScalingSummary(row);

        std::cout << "[BenchmarkRunner] threads=" << nt
                  << "  eps=" << eps
                  << "  speedup=" << speedup
                  << "  efficiency=" << efficiency << "%\n";
    }
}
