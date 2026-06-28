#include "BenchmarkRunner.hh"
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <unistd.h>
#include <limits.h>

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

static std::string SelfExePath() {
    char buf[PATH_MAX];
    ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (len < 0) return "./g4cad_bench";
    buf[len] = '\0';
    return std::string(buf);
}

void BenchmarkRunner::Run() {
    const std::vector<int> threadCounts = {1, 2, 4, 8, 16};
    int maxThreads = static_cast<int>(std::thread::hardware_concurrency());
    std::string exe = SelfExePath();

    double baseline_eps = -1.0;

    for (int nt : threadCounts) {
        if (nt > maxThreads) {
            std::cout << "[BenchmarkRunner] Skipping " << nt
                      << " threads (max=" << maxThreads << ")\n";
            continue;
        }

        // Build subprocess command — redirects stderr to /dev/null to suppress G4 verbosity
        std::ostringstream cmd;
        cmd << exe
            << " --geometry "    << fBaseCfg.geometry_name
            << " --mode "        << fBaseCfg.mode
            << " --events "      << fBaseCfg.n_events
            << " --threads "     << nt
            << " --seed "        << fBaseCfg.seed
            << " --physics-list "<< fBaseCfg.physics_list
            << " --material "    << fBaseCfg.material
            << " --output /tmp/g4bench_t" << nt;
        if (!fBaseCfg.step_file.empty())
            cmd << " --step-file " << fBaseCfg.step_file;
        cmd << " 2>/dev/null";

        std::cout << "[BenchmarkRunner] threads=" << nt << " starting...\n";

        auto t0 = std::chrono::steady_clock::now();
        int ret = std::system(cmd.str().c_str());
        auto t1 = std::chrono::steady_clock::now();

        if (ret != 0) {
            std::cout << "[BenchmarkRunner] threads=" << nt << " subprocess failed (code=" << ret << "), skipping\n";
            continue;
        }

        double wallTime = std::chrono::duration<double>(t1 - t0).count();
        double eps      = (wallTime > 0) ? fBaseCfg.n_events / wallTime : 0.0;
        if (baseline_eps < 0) baseline_eps = eps;

        double speedup    = (baseline_eps > 0) ? eps / baseline_eps : 1.0;
        double efficiency = (nt > 0) ? speedup / nt * 100.0 : 0.0;

        ScalingSummaryRow row;
        row.geometry       = fBaseCfg.geometry_name;
        row.mode           = fBaseCfg.mode;
        row.n_threads      = nt;
        row.n_events       = fBaseCfg.n_events;
        row.wall_time_s    = wallTime;
        row.events_per_sec = eps;
        row.speedup        = speedup;
        row.efficiency_pct = efficiency;
        row.mem_rss_mb     = ReadMemRSS_MB();
        row.nav_warnings   = 0;

        fWriter->WriteScalingSummary(row);

        std::cout << "[BenchmarkRunner] threads=" << nt
                  << "  wall=" << wallTime << "s"
                  << "  eps=" << eps
                  << "  speedup=" << speedup
                  << "  eff=" << efficiency << "%\n";
    }
}
