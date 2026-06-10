#include "G4StepLoader.hh"
#include "G4StepSolid.hh"
#include "globals.hh"

#include <STEPControl_Reader.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopAbs.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <IFSelect_ReturnStatus.hxx>

void G4StepLoader::CollectSolids(const TopoDS_Shape& shape, std::vector<TopoDS_Shape>& solids) {
    if (shape.IsNull()) return;
    if (shape.ShapeType() == TopAbs_SOLID) {
        solids.push_back(shape);
    } else if (shape.ShapeType() == TopAbs_COMPOUND) {
        TopoDS_Iterator it(shape);
        for (; it.More(); it.Next()) {
            CollectSolids(it.Value(), solids);
        }
    }
}

std::vector<TopoDS_Shape> G4StepLoader::Load(const std::string& filename, bool verbose) {
    std::vector<TopoDS_Shape> safeList;
    
    if (verbose) G4cout << ">>> G4StepLoader: Reading " << filename << "..." << G4endl;

    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filename.c_str());
    
    if (status != IFSelect_RetDone) {
        G4cerr << ">>> G4StepLoader Error: Cannot read file." << G4endl;
        return safeList;
    }

    reader.TransferRoots();
    TopoDS_Shape result = reader.OneShape();

    if (result.IsNull()) return safeList;

    std::vector<TopoDS_Shape> rawList;
    CollectSolids(result, rawList);

    // Deep-copy the solids so they are detached from the original STEP document state.
    for (const auto& s : rawList) {
        BRepBuilderAPI_Copy copier(s);
        if (copier.IsDone()) {
            safeList.push_back(copier.Shape());
        }
    }

    if (verbose) G4cout << ">>> G4StepLoader: Loaded " << safeList.size() << " solids." << G4endl;
    return safeList;
}