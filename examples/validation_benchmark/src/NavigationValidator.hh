#pragma once
#include <string>
#include <vector>
#include "G4VSolid.hh"
#include "G4ThreeVector.hh"

struct NavValidationResult {
    std::string geometry_name;
    int    n_points        = 0;
    int    n_rays          = 0;
    double mismatch_rate   = 0.0;
    double mean_dev_in     = 0.0;
    double median_dev_in   = 0.0;
    double max_dev_in      = 0.0;
    double mean_dev_out    = 0.0;
    double median_dev_out  = 0.0;
    double max_dev_out     = 0.0;
    int    n_problematic_in  = 0;
    int    n_problematic_out = 0;
    std::vector<G4ThreeVector> dump_points;
};

class NavigationValidator {
public:
    NavigationValidator(int n_points  = 100000,
                        int n_rays    = 50000,
                        double tol    = 1e-3,
                        bool   dump   = false);

    NavValidationResult Run(
        const std::vector<G4VSolid*>& native_solids,
        const std::vector<G4VSolid*>& step_solids,
        const G4ThreeVector& bbox_min,
        const G4ThreeVector& bbox_max,
        const std::string&   geometry_name,
        long seed = 42);

private:
    int    fNPoints;
    int    fNRays;
    double fTol;
    bool   fDump;

    // Union Inside() over a list of solids (for touching_boxes)
    EInside InsideUnion(const std::vector<G4VSolid*>& solids,
                        const G4ThreeVector& p) const;

    // Min DistanceToIn over a list (point is outside all)
    double DistToInMin(const std::vector<G4VSolid*>& solids,
                       const G4ThreeVector& p,
                       const G4ThreeVector& v) const;

    // DistanceToOut for the solid that contains p
    double DistToOutContaining(const std::vector<G4VSolid*>& solids,
                               const G4ThreeVector& p,
                               const G4ThreeVector& v) const;

    static double Median(std::vector<double>& v);
};
