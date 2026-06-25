#include "ActionInitialization.hh"
#include "RunAction.hh"
#include "EventAction.hh"
#include "SteppingAction.hh"
#include "PrimaryGeneratorAction.hh"

ActionInitialization::ActionInitialization(OutputWriter* writer,
                                            const PrimaryConfig& primaryCfg,
                                            std::string geometryName,
                                            std::string mode)
    : fWriter(writer), fPrimary(primaryCfg),
      fGeometryName(std::move(geometryName)), fMode(std::move(mode)) {}

void ActionInitialization::BuildForMaster() const {
    SetUserAction(new RunAction(fWriter, fGeometryName, fMode));
}

void ActionInitialization::Build() const {
    auto* runAction   = new RunAction(fWriter, fGeometryName, fMode);
    auto* eventAction = new EventAction(runAction);

    SetUserAction(runAction);
    SetUserAction(eventAction);
    SetUserAction(new SteppingAction(eventAction));
    SetUserAction(new PrimaryGeneratorAction(
        fPrimary.particle, fPrimary.energyMeV,
        fPrimary.direction, fPrimary.position));
}
