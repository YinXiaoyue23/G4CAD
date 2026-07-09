#include "PhysicsRunner.hh"
#include "geant4/DetectorConstruction.hh"
#include "geant4/ActionInitialization.hh"
#include "G4RunManagerFactory.hh"
#include "G4PhysListFactory.hh"
#include "Randomize.hh"
#include <chrono>
#include <iostream>
#include <stdexcept>

PhysicsRunner::PhysicsRunner(OutputWriter* writer, PhysicsConfig cfg)
    : fWriter(writer), fCfg(std::move(cfg)) {}

double PhysicsRunner::Run(std::vector<G4VSolid*> solids) {
    // Determine run manager type
    auto rmType = (fCfg.n_threads > 1)
        ? G4RunManagerType::MTOnly
        : G4RunManagerType::SerialOnly;

    auto* rm = G4RunManagerFactory::CreateRunManager(rmType);
    if (!rm) throw std::runtime_error("Failed to create G4RunManager");

#ifdef G4MULTITHREADED
    if (fCfg.n_threads > 1) {
        auto* mtRM = dynamic_cast<G4MTRunManager*>(rm);
        if (mtRM) mtRM->SetNumberOfThreads(fCfg.n_threads);
    }
#endif

    // Detector
    rm->SetUserInitialization(
        new DetectorConstruction(solids, fCfg.material, fCfg.world_half_mm));

    // Physics list
    G4PhysListFactory factory;
    auto* physList = factory.GetReferencePhysList(fCfg.physics_list);
    if (!physList) throw std::runtime_error("Unknown physics list: " + fCfg.physics_list);
    rm->SetUserInitialization(physList);

    // Actions
    PrimaryConfig primary;
    primary.particle  = fCfg.particle;
    primary.energyMeV = fCfg.energy_MeV;
    rm->SetUserInitialization(
        new ActionInitialization(fWriter, primary, fCfg.geometry_name, fCfg.mode));

    // Random seed
    CLHEP::HepRandom::setTheSeed(fCfg.seed);

    rm->Initialize();

    std::cout << "[PhysicsRunner] Running " << fCfg.n_events << " events"
              << "  geometry=" << fCfg.geometry_name
              << "  mode=" << fCfg.mode
              << "  threads=" << fCfg.n_threads << "\n";

    auto t0 = std::chrono::steady_clock::now();
    rm->BeamOn(fCfg.n_events);
    auto t1 = std::chrono::steady_clock::now();

    double wallTime = std::chrono::duration<double>(t1 - t0).count();
    std::cout << "[PhysicsRunner] Done. Wall time = " << wallTime << " s\n";

    delete rm;
    return wallTime;
}
