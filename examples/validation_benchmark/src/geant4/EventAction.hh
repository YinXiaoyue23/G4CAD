#pragma once
#include "G4UserEventAction.hh"

class RunAction;

class EventAction : public G4UserEventAction {
public:
    explicit EventAction(RunAction* runAction);

    void BeginOfEventAction(const G4Event*) override;
    void EndOfEventAction(const G4Event* event) override;

    void AccumulateStep(double edepMeV, double lengthMm, bool isBoundary);

private:
    RunAction* fRunAction;
    double fEdep        = 0.0;
    double fTrackLength = 0.0;
    int    fNSteps      = 0;
    int    fNBoundary   = 0;
};
