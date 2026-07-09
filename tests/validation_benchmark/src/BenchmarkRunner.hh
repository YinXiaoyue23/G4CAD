#pragma once
#include "PhysicsRunner.hh"
#include "OutputWriter.hh"
#include <vector>

class BenchmarkRunner {
public:
    BenchmarkRunner(OutputWriter*           writer,
                    const PhysicsConfig&    baseCfg,
                    std::vector<G4VSolid*>  solids);

    void Run();

private:
    OutputWriter*          fWriter;
    PhysicsConfig          fBaseCfg;
    std::vector<G4VSolid*> fSolids;

    static double ReadMemRSS_MB();
};
