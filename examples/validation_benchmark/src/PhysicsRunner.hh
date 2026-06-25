#pragma once
#include "OutputWriter.hh"
#include "G4VSolid.hh"
#include <string>
#include <vector>

struct PhysicsConfig {
    std::string geometry_name;
    std::string mode;           // "native" or "step"
    std::string particle    = "e-";
    double      energy_MeV  = 10.0;
    int         n_events    = 1000;
    int         n_threads   = 1;
    long        seed        = 42;
    std::string physics_list = "FTFP_BERT";
    std::string material     = "G4_Si";
    double      world_half_mm = 300.0;
};

class PhysicsRunner {
public:
    PhysicsRunner(OutputWriter* writer, PhysicsConfig cfg);

    // Run simulation with the given solids. Returns wall time in seconds.
    double Run(std::vector<G4VSolid*> solids);

private:
    OutputWriter* fWriter;
    PhysicsConfig fCfg;
};
