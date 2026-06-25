#include "SteppingAction.hh"
#include "EventAction.hh"
#include "G4Step.hh"
#include "G4StepPoint.hh"
#include "G4SystemOfUnits.hh"

SteppingAction::SteppingAction(EventAction* ea) : fEventAction(ea) {}

void SteppingAction::UserSteppingAction(const G4Step* step) {
    double edep   = step->GetTotalEnergyDeposit() / MeV;
    double length = step->GetStepLength() / mm;
    bool   isBnd  = (step->GetPostStepPoint()->GetStepStatus() == fGeomBoundary);
    fEventAction->AccumulateStep(edep, length, isBnd);
}
