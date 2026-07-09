#include "EnvironmentRecord.hh"
#include "G4Version.hh"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <thread>

// OCCT version string supplied via CMake compile definition
#ifndef OCCT_VERSION_STR
#define OCCT_VERSION_STR "unknown"
#endif

// G4CAD version supplied via CMake compile definition
#ifndef G4CAD_VERSION_STR
#define G4CAD_VERSION_STR "unknown"
#endif

#ifndef BUILD_TYPE_STR
#define BUILD_TYPE_STR "unknown"
#endif

static std::string ReadProcLine(const std::string& file, const std::string& key) {
    std::ifstream f(file);
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind(key, 0) == 0) {
            auto colon = line.find(':');
            if (colon != std::string::npos) {
                std::string val = line.substr(colon + 1);
                // trim leading whitespace and trailing newline
                auto start = val.find_first_not_of(" \t");
                if (start != std::string::npos) val = val.substr(start);
                while (!val.empty() && (val.back() == '\n' || val.back() == '\r'))
                    val.pop_back();
                return val;
            }
        }
    }
    return "unknown";
}

static int CountProcLines(const std::string& file, const std::string& key) {
    std::ifstream f(file);
    std::string line;
    int count = 0;
    while (std::getline(f, line))
        if (line.rfind(key, 0) == 0) ++count;
    return count;
}

static std::string OsName() {
    // Try /etc/os-release first (Linux)
    std::ifstream f("/etc/os-release");
    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("PRETTY_NAME=", 0) == 0) {
            std::string val = line.substr(12);
            if (!val.empty() && val.front() == '"') val = val.substr(1);
            if (!val.empty() && val.back() == '"')  val.pop_back();
            return val;
        }
    }
    // Fallback: uname -r via popen
    FILE* p = popen("uname -sr", "r");
    if (p) {
        char buf[256] = {};
        if (fgets(buf, sizeof(buf), p)) {
            std::string s(buf);
            while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
            pclose(p);
            return s;
        }
        pclose(p);
    }
    return "unknown";
}

static std::string CompilerString() {
#if defined(__clang__)
    return std::string("clang ") + __clang_version__;
#elif defined(__GNUC__)
    return std::string("gcc ") + __VERSION__;
#else
    return "unknown";
#endif
}

void EnvironmentRecord::Write(const std::string& filename,
                               const std::string& physics_list,
                               long seed) {
    std::ofstream f(filename);
    if (!f) return;

    // Timestamp
    std::time_t now = std::time(nullptr);
    char tsbuf[64] = {};
    std::strftime(tsbuf, sizeof(tsbuf), "%Y-%m-%dT%H:%M:%S", std::localtime(&now));

    // Hardware concurrency
    unsigned int hw_threads = std::thread::hardware_concurrency();

    // CPU info (Linux)
    std::string cpu_model = ReadProcLine("/proc/cpuinfo", "model name");
    int cpu_cores = CountProcLines("/proc/cpuinfo", "processor");

    // RAM
    std::string mem_total_kb = ReadProcLine("/proc/meminfo", "MemTotal");
    double ram_gb = -1.0;
    try { ram_gb = std::stod(mem_total_kb) / (1024.0 * 1024.0); } catch (...) {}

    f << "timestamp           = " << tsbuf           << "\n";
    f << "geant4_version      = " << G4Version        << "\n";
    f << "g4cad_version       = " << G4CAD_VERSION_STR << "\n";
    f << "occt_version        = " << OCCT_VERSION_STR  << "\n";
    f << "compiler            = " << CompilerString()  << "\n";
    f << "build_type          = " << BUILD_TYPE_STR    << "\n";
    f << "os                  = " << OsName()          << "\n";
    f << "cpu_model           = " << cpu_model         << "\n";
    f << "cpu_cores_logical   = " << cpu_cores         << "\n";
    f << "cpu_hw_concurrency  = " << hw_threads        << "\n";
    if (ram_gb > 0)
        f << "ram_total_gb        = " << ram_gb         << "\n";
    else
        f << "ram_total_gb        = unknown\n";
    f << "physics_list        = " << physics_list      << "\n";
    f << "seed                = " << seed              << "\n";
    f << "production_cuts     = default\n";
}
