#pragma once
#include "G4VSolid.hh"
#include "G4ThreeVector.hh"
#include "OutputWriter.hh"
#include <string>
#include <vector>

class SafetyDiagnostic {
public:
    SafetyDiagnostic(int nRandom = 10000, int nPath = 1000);

    // Compare scalar DistanceToOut(p) between native and step solids.
    // Writes per-point CSV and aggregate summary row via writer.
    void Run(
        const std::vector<G4VSolid*>& native_solids,
        const std::vector<G4VSolid*>& step_solids,
        const G4ThreeVector& bbox_min,
        const G4ThreeVector& bbox_max,
        const std::string& geometry_name,
        const std::string& mode,
        long seed,
        OutputWriter& writer);

private:
    int fNRandom;
    int fNPath;

    // Returns index of first solid with kInside, or -1 if none.
    static int FindContainingSolid(const std::vector<G4VSolid*>& solids,
                                    const G4ThreeVector& p);
};
