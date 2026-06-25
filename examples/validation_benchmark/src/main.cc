#include "GeometryRegistry.hh"
#include "OutputWriter.hh"
#include "NavigationValidator.hh"
#include "CrossSectionSampler.hh"
#include "PhysicsRunner.hh"
#include "BenchmarkRunner.hh"
#include "G4SystemOfUnits.hh"

#include <getopt.h>
#include <iostream>
#include <string>
#include <cstdlib>
#include <stdexcept>
#include <cmath>

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
"  --particle  gamma|e-|proton  (default e-)\n"
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
"  --cross-section-plane   xy|xz|yz|all  (default all)\n"
"  --cross-section-resolution <N>  (default 500)\n"
"\n"
"Benchmarking:\n"
"  --benchmark             Run scaling test over thread counts 1,2,4,8,16\n"
"\n"
"Output:\n"
"  --output    <prefix>    Output file prefix (default \"output\")\n"
"  --enable-root           Write ROOT files in addition to CSV\n"
"\n";
}

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
    std::string cs_plane_str   = "all";
    int    cs_resolution       = 500;
    bool   do_benchmark        = false;
    bool   enable_root         = false;

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
        {"cross-section-plane",      required_argument, nullptr, 1003},
        {"cross-section-resolution", required_argument, nullptr, 1004},
        {"benchmark",                no_argument,       nullptr, 'b'},
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
            case 1003: cs_plane_str = optarg; break;
            case 1004: cs_resolution = std::atoi(optarg); break;
            case 'b': do_benchmark  = true;   break;
            case 'r': enable_root   = true;   break;
            case 'h': PrintUsage(argv[0]); return 0;
            default:  PrintUsage(argv[0]); return 1;
        }
    }

    // ---- validate ----
    if (geometry_str.empty()) { std::cerr << "Error: --geometry required\n"; return 1; }
    if ((do_nav_test || do_cross_section) && step_file.empty()) {
        std::cerr << "Error: --navigation-test and --cross-section require --step-file\n";
        return 1;
    }
    if (mode_str == "step" && step_file.empty()) {
        std::cerr << "Error: --mode step requires --step-file\n";
        return 1;
    }

    try {
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

        // ---- cross-section ----
        if (do_cross_section) {
            CrossSectionConfig cscfg;
            cscfg.resolution = cs_resolution;
            if (cs_plane_str == "all") {
                cscfg.planes = { SlicePlane::XY, SlicePlane::XZ, SlicePlane::YZ };
            } else {
                cscfg.planes = { CrossSectionSampler::ParsePlane(cs_plane_str) };
            }

            // native
            auto native_solids = GeometryRegistry::CreateNative(gtype);
            CrossSectionSampler::RunAll(native_solids, ginfo.bbox_min, ginfo.bbox_max,
                                        cscfg, geometry_str, "native", writer);

            // step
            auto step_solids = GeometryRegistry::CreateFromStep(step_file, ginfo.tolerance_mm);
            CrossSectionSampler::RunAll(step_solids, ginfo.bbox_min, ginfo.bbox_max,
                                        cscfg, geometry_str, "step", writer);
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

        // ---- benchmark ----
        if (do_benchmark) {
            PhysicsConfig pcfg;
            pcfg.geometry_name = geometry_str;
            pcfg.mode          = mode_str;
            pcfg.particle      = particle;
            pcfg.energy_MeV    = energy_MeV;
            pcfg.n_events      = (n_events > 0) ? n_events : 1000;
            pcfg.seed          = seed;
            pcfg.physics_list  = physics_list;
            pcfg.material      = material;

            BenchmarkRunner bench(&writer, pcfg, solids);
            bench.Run();
        }

        writer.Flush();
        std::cout << "[Done] Output prefix: " << output_prefix << "\n";

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
