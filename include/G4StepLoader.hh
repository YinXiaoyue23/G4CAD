#ifndef G4StepLoader_HH
#define G4StepLoader_HH

#include <vector>
#include <string>
// Expose the OCCT header directly because the user environment already provides OpenCASCADE.
#include <TopoDS_Shape.hxx> 

class G4StepLoader {
public:
    // Read a STEP file and return a processed list of shapes using deep-copied solids.
    static std::vector<TopoDS_Shape> Load(const std::string& filename, bool verbose = true);

private:
    static void CollectSolids(const TopoDS_Shape& shape, std::vector<TopoDS_Shape>& solids);
};

#endif