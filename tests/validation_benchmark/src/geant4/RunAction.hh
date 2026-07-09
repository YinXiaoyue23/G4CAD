#pragma once
#include "G4UserRunAction.hh"
#include "OutputWriter.hh"
#include <string>
#include <atomic>

class RunAction : public G4UserRunAction {
public:
    RunAction(OutputWriter*  writer,
              std::string    geometryName,
              std::string    mode);

    void BeginOfRunAction(const G4Run*) override;
    void EndOfRunAction(const G4Run*) override;

    void RecordEvent(int eventId, double edepMeV, double trackLenMm,
                     int nSteps, int nBoundary);

private:
    OutputWriter* fWriter;
    std::string   fGeometryName;
    std::string   fMode;
};
