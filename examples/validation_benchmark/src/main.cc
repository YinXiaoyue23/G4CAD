#include "GeometryRegistry.hh"
#include "OutputWriter.hh"
#include "NavigationValidator.hh"
#include "CrossSectionSampler.hh"
#include "PhysicsRunner.hh"
#include "BenchmarkRunner.hh"
#include "SafetyDiagnostic.hh"
#include "EnvironmentRecord.hh"
#include "G4SystemOfUnits.hh"

#include <getopt.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <stdexcept>
#include <cmath>
#include <chrono>
#include <sstream>
#include <vector>
#include <unistd.h>
#include <limits.h>

// ---------- helpers ----------

static double ReadMemRSS_MB() {
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

static void PrintUsage(const char* prog) {
    std::cout <<
"Usage: " << prog << " [OPTIONS]\n"
"\n"
"Geometry:\n"
"  --geometry  box|sphere|cylinder|box_hole|touching_boxes\n"
"  --mode      native|step\n"
"  --step-file <path>      (required when mode=step or --navigation-test/--cross-section)\n"
"\n"
"Simulation:\n"
"  --events    <N>         (default 1000; 0 = skip physics run)\n"
"  --threads   <N>         (default 1)\n"
"  --particle  gamma|e-|proton|geantino|chargedgeantino  (default e-)\n"
"  --energy    <MeV>       (default 10.0)\n"
"  --seed      <value>     (default 42)\n"
"  --physics-list FTFP_BERT|QBBC  (default FTFP_BERT)\n"
"  --material  <G4 material>      (default G4_Si)\n"
"\n"
"Validation:\n"
"  --navigation-test       Compare native vs STEP solid navigation (requires --step-file)\n"
"  --enable-navigation-dump  Write mismatch points to CSV for FreeCAD\n"
"  --nav-points <N>        Random points for navigation test (default 100000)\n"
"  --nav-rays   <N>        Random rays for navigation test (default 50000)\n"
"\n"
"Cross-section:\n"
"  --cross-section         Sample grid and classify points (requires --step-file)\n"
"  --cross-section-compare Compare native vs step and write mismatch maps\n"
"  --cross-section-plane   xy|xz|yz|all  (default all)\n"
"  --cross-section-resolution <N>  (default 500)\n"
"\n"
"Safety diagnostic:\n"
"  --safety-diagnostic     Compare scalar DistanceToOut(p) between native and step\n"
"  --safety-points <N>     Random interior points (default 10000)\n"
"  --safety-path-points <N> Points along primary ray path (default 1000)\n"
"\n"
"Geantino control run:\n"
"  --geantino-test         Run additional geantino simulation (same geometry/mode)\n"
"\n"
"Benchmarking:\n"
"  --benchmark             Run scaling test over thread counts 1,2,4,8,16\n"
"  --benchmark-geometries <g1,g2,...>  Run scaling for comma-separated geometry list\n"
"\n"
"Complex STEP case:\n"
"  --complex-case <path>   Load and run arbitrary STEP file (no native comparison)\n"
"  --skip-volume           Skip GetCubicVolume() in complex-case (avoids MC hang for 100+ face solids)\n"
"\n"
"Output:\n"
"  --output    <prefix>    Output file prefix (default \"output\")\n"
"  --enable-root           Write ROOT files in addition to CSV\n"
"\n";
}

static std::vector<std::string> SplitComma(const std::string& s) {
    std::vector<std::string> out;
    std::istringstream ss(s);
    std::string tok;
    while (std::getline(ss, tok, ','))
        if (!tok.empty()) out.push_back(tok);
    return out;
}

// ---------- main ----------

int main(int argc, char** argv) {
    // ---- defaults ----
    std::string geometry_str   = "box";
    std::string mode_str       = "native";
    std::string step_file;
    int    n_events            = 1000;
    int    n_threads           = 1;
    std::string particle       = "e-";
    double energy_MeV          = 10.0;
    long   seed                = 42;
    std::string output_prefix  = "output";
    std::string physics_list   = "FTFP_BERT";
    std::string material       = "G4_Si";
    bool   do_nav_test         = false;
    bool   nav_dump            = false;
    int    nav_points          = 100000;
    int    nav_rays            = 50000;
    bool   do_cross_section    = false;
    bool   do_cs_compare       = false;
    std::string cs_plane_str   = "all";
    int    cs_resolution       = 500;
    bool   do_benchmark        = false;
    std::string bench_geometries;
    bool   do_safety_diag      = false;
    int    safety_points       = 10000;
    int    safety_path_points  = 1000;
    bool   do_geantino_test    = false;
    std::string complex_case_file;
    bool   enable_root         = false;
    bool   skip_volume         = false;

    // ---- option parsing ----
    static option long_opts[] = {
        {"geometry",                 required_argument, nullptr, 'g'},
        {"mode",                     required_argument, nullptr, 'm'},
        {"step-file",                required_argument, nullptr, 'f'},
        {"events",                   required_argument, nullptr, 'e'},
        {"threads",                  required_argument, nullptr, 't'},
        {"particle",                 required_argument, nullptr, 'p'},
        {"energy",                   required_argument, nullptr, 'E'},
        {"seed",                     required_argument, nullptr, 's'},
        {"output",                   required_argument, nullptr, 'o'},
        {"physics-list",             required_argument, nullptr, 'P'},
        {"material",                 required_argument, nullptr, 'M'},
        {"navigation-test",          no_argument,       nullptr, 'n'},
        {"enable-navigation-dump",   no_argument,       nullptr, 'd'},
        {"nav-points",               required_argument, nullptr, 1001},
        {"nav-rays",                 required_argument, nullptr, 1002},
        {"cross-section",            no_argument,       nullptr, 'c'},
        {"cross-section-compare",    no_argument,       nullptr, 2005},
        {"cross-section-plane",      required_argument, nullptr, 1003},
        {"cross-section-resolution", required_argument, nullptr, 1004},
        {"benchmark",                no_argument,       nullptr, 'b'},
        {"benchmark-geometries",     required_argument, nullptr, 2007},
        {"safety-diagnostic",        no_argument,       nullptr, 2001},
        {"safety-points",            required_argument, nullptr, 2002},
        {"safety-path-points",       required_argument, nullptr, 2003},
        {"geantino-test",            no_argument,       nullptr, 2004},
        {"complex-case",             required_argument, nullptr, 2006},
        {"skip-volume",              no_argument,       nullptr, 2008},
        {"enable-root",              no_argument,       nullptr, 'r'},
        {"help",                     no_argument,       nullptr, 'h'},
        {nullptr, 0, nullptr, 0}
    };

    int opt, idx;
    while ((opt = getopt_long(argc, argv, "g:m:f:e:t:p:E:s:o:P:M:ndcbrh", long_opts, &idx)) != -1) {
        switch (opt) {
            case 'g': geometry_str  = optarg; break;
            case 'm': mode_str      = optarg; break;
            case 'f': step_file     = optarg; break;
            case 'e': n_events      = std::atoi(optarg); break;
            case 't': n_threads     = std::atoi(optarg); break;
            case 'p': particle      = optarg; break;
            case 'E': energy_MeV    = std::atof(optarg); break;
            case 's': seed          = std::atol(optarg); break;
            case 'o': output_prefix = optarg; break;
            case 'P': physics_list  = optarg; break;
            case 'M': material      = optarg; break;
            case 'n': do_nav_test   = true;   break;
            case 'd': nav_dump      = true;   break;
            case 1001: nav_points   = std::atoi(optarg); break;
            case 1002: nav_rays     = std::atoi(optarg); break;
            case 'c': do_cross_section = true; break;
            case 2005: do_cs_compare = true; break;
            case 1003: cs_plane_str = optarg; break;
            case 1004: cs_resolution = std::atoi(optarg); break;
            case 'b': do_benchmark  = true;   break;
            case 2007: bench_geometries = optarg; break;
            case 2001: do_safety_diag = true; break;
            case 2002: safety_points = std::atoi(optarg); break;
            case 2003: safety_path_points = std::atoi(optarg); break;
            case 2004: do_geantino_test = true; break;
            case 2006: complex_case_file = optarg; break;
            case 2008: skip_volume  = true;   break;
            case 'r': enable_root   = true;   break;
            case 'h': PrintUsage(argv[0]); return 0;
            default:  PrintUsage(argv[0]); return 1;
        }
    }

    // ---- validate ----
    if ((do_nav_test || do_cross_section || do_cs_compare || do_safety_diag)
        && step_file.empty() && mode_str != "native") {
        std::cerr << "Error: --navigation-test, --cross-section(-compare), "
                     "--safety-diagnostic require --step-file when mode=step\n";
        return 1;
    }
    if (mode_str == "step" && step_file.empty() && complex_case_file.empty()) {
        std::cerr << "Error: --mode step requires --step-file\n";
        return 1;
    }

    try {
        // Write environment record first (always)
        EnvironmentRecord::Write(output_prefix + "_environment.txt", physics_list, seed);

        // ---- complex STEP case (independent path) ----
        if (!complex_case_file.empty()) {
            std::cout << "[ComplexCase] Loading " << complex_case_file << " ...\n";
            double mem_before = ReadMemRSS_MB();
            auto t_load0 = std::chrono::steady_clock::now();
            auto solids = GeometryRegistry::CreateFromStep(complex_case_file, 1e-7);
            auto t_load1 = std::chrono::steady_clock::now();
            double mem_after  = ReadMemRSS_MB();
            double load_time  = std::chrono::duration<double>(t_load1 - t_load0).count();

            // Compute aggregate bbox and (optionally) volume
            // --skip-volume: skip GetCubicVolume() which uses slow MC for complex solids
            double xmin=1e30, xmax=-1e30, ymin=1e30, ymax=-1e30, zmin=1e30, zmax=-1e30;
            double total_vol = 0.0;
            for (auto* s : solids) {
                G4ThreeVector pmin, pmax;
                s->BoundingLimits(pmin, pmax);
                xmin=std::min(xmin,pmin.x()); xmax=std::max(xmax,pmax.x());
                ymin=std::min(ymin,pmin.y()); ymax=std::max(ymax,pmax.y());
                zmin=std::min(zmin,pmin.z()); zmax=std::max(zmax,pmax.z());
                if (!skip_volume) total_vol += s->GetCubicVolume();
            }
            if (skip_volume)
                std::cout << "[ComplexCase] n_solids=" << solids.size()
                          << "  load_time=" << load_time << "s"
                          << "  volume=SKIPPED(--skip-volume)"
                          << "  mem_delta=" << (mem_after - mem_before) << " MB\n";
            else
                std::cout << "[ComplexCase] n_solids=" << solids.size()
                          << "  load_time=" << load_time << "s"
                          << "  volume=" << total_vol << " mm3"
                          << "  mem_delta=" << (mem_after - mem_before) << " MB\n";

            OutputWriter cwriter(output_prefix, enable_root);

            double wall_time = 0.0, eps = 0.0;
            if (n_events > 0) {
                // Auto-position gun below the geometry
                double world_half = std::max({std::abs(xmax), std::abs(xmin),
                                              std::abs(ymax), std::abs(ymin),
                                              std::abs(zmax), std::abs(zmin)}) * 1.5 + 300.0;
                PhysicsConfig pcfg;
                pcfg.geometry_name = "complex_step";
                pcfg.mode          = "step";
                pcfg.particle      = particle;
                pcfg.energy_MeV    = energy_MeV;
                pcfg.n_events      = n_events;
                pcfg.n_threads     = n_threads;
                pcfg.seed          = seed;
                pcfg.physics_list  = physics_list;
                pcfg.material      = material;
                pcfg.world_half_mm = world_half;
                PhysicsRunner runner(&cwriter, pcfg);
                wall_time = runner.Run(solids);
                eps = (wall_time > 0) ? n_events / wall_time : 0.0;
            }

            ComplexCaseSummaryRow crow;
            crow.step_file     = complex_case_file;
            crow.n_solids      = (int)solids.size();
            crow.load_time_s   = load_time;
            crow.mem_before_mb = mem_before;
            crow.mem_after_mb  = mem_after;
            crow.bbox_xmin     = xmin; crow.bbox_xmax = xmax;
            crow.bbox_ymin     = ymin; crow.bbox_ymax = ymax;
            crow.bbox_zmin     = zmin; crow.bbox_zmax = zmax;
            crow.volume_mm3    = total_vol;
            crow.wall_time_s   = wall_time;
            crow.events_per_sec = eps;
            crow.n_events      = n_events;
            cwriter.WriteComplexCaseSummary(crow);
            cwriter.Flush();
            std::cout << "[Done] Complex case output prefix: " << output_prefix << "\n";
            return 0;
        }

        GeometryType gtype = GeometryRegistry::Parse(geometry_str);
        GeometryInfo ginfo = GeometryRegistry::GetInfo(gtype);

        OutputWriter writer(output_prefix, enable_root);

        // ---- build solids ----
        std::vector<G4VSolid*> solids;
        if (mode_str == "native") {
            solids = GeometryRegistry::CreateNative(gtype);
        } else {
            solids = GeometryRegistry::CreateFromStep(step_file, ginfo.tolerance_mm);
        }

        // ---- geometry summary ----
        {
            double vol = 0.0;
            for (auto* s : solids) vol += s->GetCubicVolume();
            double rel_err = (ginfo.analytical_volume_mm3 > 0)
                ? std::abs(vol - ginfo.analytical_volume_mm3) / ginfo.analytical_volume_mm3
                : 0.0;

            GeometrySummaryRow row;
            row.geometry              = geometry_str;
            row.mode                  = mode_str;
            row.volume_mm3            = vol;
            row.analytical_volume_mm3 = ginfo.analytical_volume_mm3;
            row.volume_rel_error      = rel_err;
            row.bbox_xmin             = ginfo.bbox_min.x();
            row.bbox_xmax             = ginfo.bbox_max.x();
            row.bbox_ymin             = ginfo.bbox_min.y();
            row.bbox_ymax             = ginfo.bbox_max.y();
            row.bbox_zmin             = ginfo.bbox_min.z();
            row.bbox_zmax             = ginfo.bbox_max.z();
            row.n_solids              = ginfo.n_solids;
            row.tolerance_mm          = ginfo.tolerance_mm;
            writer.WriteGeometrySummary(row);

            std::cout << "[Geometry] " << geometry_str << " (" << mode_str << ")"
                      << "  volume=" << vol << " mm3"
                      << "  rel_err=" << rel_err*100 << "%\n";
        }

        // ---- navigation test ----
        if (do_nav_test) {
            auto native_solids = GeometryRegistry::CreateNative(gtype);
            auto step_solids   = GeometryRegistry::CreateFromStep(step_file, ginfo.tolerance_mm);

            NavigationValidator nav(nav_points, nav_rays, 1e-3, nav_dump);
            auto result = nav.Run(native_solids, step_solids,
                                  ginfo.bbox_min, ginfo.bbox_max,
                                  geometry_str, seed);

            NavigationTestRow row;
            row.geometry          = geometry_str;
            row.mode_pair         = "native_vs_step";
            row.n_points          = result.n_points;
            row.n_rays            = result.n_rays;
            row.mismatch_rate     = result.mismatch_rate;
            row.mean_dev_in       = result.mean_dev_in;
            row.median_dev_in     = result.median_dev_in;
            row.max_dev_in        = result.max_dev_in;
            row.mean_dev_out      = result.mean_dev_out;
            row.median_dev_out    = result.median_dev_out;
            row.max_dev_out       = result.max_dev_out;
            row.n_problematic_in  = result.n_problematic_in;
            row.n_problematic_out = result.n_problematic_out;
            writer.WriteNavigationTest(row);

            if (nav_dump && !result.dump_points.empty()) {
                std::ofstream df(output_prefix + "_nav_dump_" + geometry_str + ".csv");
                df << "x_mm,y_mm,z_mm\n";
                for (const auto& p : result.dump_points)
                    df << p.x() << "," << p.y() << "," << p.z() << "\n";
            }
        }

        // ---- cross-section (single-mode) ----
        if (do_cross_section) {
            CrossSectionConfig cscfg;
            cscfg.resolution = cs_resolution;
            if (cs_plane_str == "all") {
                cscfg.planes = { SlicePlane::XY, SlicePlane::XZ, SlicePlane::YZ };
            } else {
                cscfg.planes = { CrossSectionSampler::ParsePlane(cs_plane_str) };
            }

            auto native_solids = GeometryRegistry::CreateNative(gtype);
            CrossSectionSampler::RunAll(native_solids, ginfo.bbox_min, ginfo.bbox_max,
                                        cscfg, geometry_str, "native", writer);

            auto step_solids = GeometryRegistry::CreateFromStep(step_file, ginfo.tolerance_mm);
            CrossSectionSampler::RunAll(step_solids, ginfo.bbox_min, ginfo.bbox_max,
                                        cscfg, geometry_str, "step", writer);
        }

        // ---- cross-section compare (native vs step with mismatch map) ----
        if (do_cs_compare) {
            if (step_file.empty()) {
                std::cerr << "Error: --cross-section-compare requires --step-file\n";
                return 1;
            }
            CrossSectionConfig cscfg;
            cscfg.resolution = cs_resolution;
            if (cs_plane_str == "all") {
                cscfg.planes = { SlicePlane::XY, SlicePlane::XZ, SlicePlane::YZ };
            } else {
                cscfg.planes = { CrossSectionSampler::ParsePlane(cs_plane_str) };
            }

            auto native_solids = GeometryRegistry::CreateNative(gtype);
            auto step_solids   = GeometryRegistry::CreateFromStep(step_file, ginfo.tolerance_mm);
            CrossSectionSampler::RunCompare(native_solids, step_solids,
                                            ginfo.bbox_min, ginfo.bbox_max,
                                            cscfg, geometry_str, writer);
        }

        // ---- safety diagnostic ----
        if (do_safety_diag) {
            if (step_file.empty()) {
                std::cerr << "Error: --safety-diagnostic requires --step-file\n";
                return 1;
            }
            auto native_solids = GeometryRegistry::CreateNative(gtype);
            auto step_solids   = GeometryRegistry::CreateFromStep(step_file, ginfo.tolerance_mm);
            SafetyDiagnostic sd(safety_points, safety_path_points);
            sd.Run(native_solids, step_solids,
                   ginfo.bbox_min, ginfo.bbox_max,
                   geometry_str, mode_str, seed, writer);
        }

        // ---- physics run ----
        if (n_events > 0 && !do_benchmark) {
            PhysicsConfig pcfg;
            pcfg.geometry_name = geometry_str;
            pcfg.mode          = mode_str;
            pcfg.particle      = particle;
            pcfg.energy_MeV    = energy_MeV;
            pcfg.n_events      = n_events;
            pcfg.n_threads     = n_threads;
            pcfg.seed          = seed;
            pcfg.physics_list  = physics_list;
            pcfg.material      = material;

            PhysicsRunner runner(&writer, pcfg);
            runner.Run(solids);
        }

        // ---- geantino control run ----
        // Runs as a subprocess because G4RunManager is a singleton that cannot
        // be re-created after the first physics run in the same process.
        if (do_geantino_test && n_events > 0 && !do_benchmark) {
            char exe_buf[PATH_MAX];
            ssize_t exe_len = readlink("/proc/self/exe", exe_buf, sizeof(exe_buf) - 1);
            if (exe_len > 0) {
                exe_buf[exe_len] = '\0';
                std::ostringstream cmd;
                cmd << exe_buf
                    << " --geometry "    << geometry_str
                    << " --mode "        << mode_str
                    << " --particle geantino"
                    << " --events "      << n_events
                    << " --seed "        << (seed + 1)
                    << " --physics-list "<< physics_list
                    << " --material "    << material
                    << " --output "      << (output_prefix + "_geantino");
                if (!step_file.empty())
                    cmd << " --step-file " << step_file;
                cmd << " 2>/dev/null";
                std::cout << "[GeantinoTest] Running geantino subprocess...\n";
                int ret = std::system(cmd.str().c_str());
                if (ret != 0)
                    std::cerr << "[GeantinoTest] subprocess failed (code=" << ret << ")\n";
            } else {
                std::cerr << "[GeantinoTest] cannot determine self exe path, skipping\n";
            }
        }

        // ---- benchmark: single geometry ----
        if (do_benchmark && bench_geometries.empty()) {
            PhysicsConfig pcfg;
            pcfg.geometry_name = geometry_str;
            pcfg.mode          = mode_str;
            pcfg.step_file     = step_file;
            pcfg.particle      = particle;
            pcfg.energy_MeV    = energy_MeV;
            pcfg.n_events      = (n_events > 0) ? n_events : 1000;
            pcfg.seed          = seed;
            pcfg.physics_list  = physics_list;
            pcfg.material      = material;

            BenchmarkRunner bench(&writer, pcfg, solids);
            bench.Run();
        }

        // ---- benchmark: multi-geometry ----
        if (!bench_geometries.empty()) {
            auto geom_list = SplitComma(bench_geometries);
            for (const auto& gname : geom_list) {
                GeometryType gt = GeometryRegistry::Parse(gname);
                GeometryInfo gi = GeometryRegistry::GetInfo(gt);

                std::vector<G4VSolid*> gs;
                if (mode_str == "native") {
                    gs = GeometryRegistry::CreateNative(gt);
                } else {
                    if (step_file.empty()) {
                        std::cerr << "[BenchmarkGeometries] --step-file required for mode=step, skipping " << gname << "\n";
                        continue;
                    }
                    gs = GeometryRegistry::CreateFromStep(step_file, gi.tolerance_mm);
                }

                PhysicsConfig pcfg;
                pcfg.geometry_name = gname;
                pcfg.mode          = mode_str;
                pcfg.step_file     = step_file;
                pcfg.particle      = particle;
                pcfg.energy_MeV    = energy_MeV;
                pcfg.n_events      = (n_events > 0) ? n_events : 1000;
                pcfg.seed          = seed;
                pcfg.physics_list  = physics_list;
                pcfg.material      = material;

                BenchmarkRunner bench(&writer, pcfg, gs);
                bench.Run();
            }
        }

        writer.Flush();
        std::cout << "[Done] Output prefix: " << output_prefix << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
