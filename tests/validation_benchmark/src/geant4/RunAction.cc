#include "RunAction.hh"

RunAction::RunAction(OutputWriter* writer, std::string geometryName, std::string mode)
    : fWriter(writer), fGeometryName(std::move(geometryName)), fMode(std::move(mode)) {}

void RunAction::BeginOfRunAction(const G4Run*) {}
void RunAction::EndOfRunAction(const G4Run*)  {}

void RunAction::RecordEvent(int eventId, double edepMeV, double trackLenMm,
                             int nSteps, int nBoundary) {
    EventSummaryRow row;
    row.geometry            = fGeometryName;
    row.mode                = fMode;
    row.event_id            = eventId;
    row.edep_MeV            = edepMeV;
    row.track_length_mm     = trackLenMm;
    row.n_steps             = nSteps;
    row.n_boundary_crossings = nBoundary;
    fWriter->WriteEventSummaryThreadSafe(row);
}
