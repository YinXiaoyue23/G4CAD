#include "EventAction.hh"
#include "RunAction.hh"
#include "G4Event.hh"

EventAction::EventAction(RunAction* runAction)
    : fRunAction(runAction) {}

void EventAction::BeginOfEventAction(const G4Event*) {
    fEdep        = 0.0;
    fTrackLength = 0.0;
    fNSteps      = 0;
    fNBoundary   = 0;
}

void EventAction::AccumulateStep(double edepMeV, double lengthMm, bool isBoundary) {
    fEdep        += edepMeV;
    fTrackLength += lengthMm;
    ++fNSteps;
    if (isBoundary) ++fNBoundary;
}

void EventAction::EndOfEventAction(const G4Event* event) {
    fRunAction->RecordEvent(event->GetEventID(),
                            fEdep, fTrackLength, fNSteps, fNBoundary);
}
