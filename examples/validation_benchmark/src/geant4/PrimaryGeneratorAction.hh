#pragma once
#include "G4VUserPrimaryGeneratorAction.hh"
#include "G4ParticleGun.hh"
#include "G4ThreeVector.hh"
#include <string>
#include <memory>

class PrimaryGeneratorAction : public G4VUserPrimaryGeneratorAction {
public:
    PrimaryGeneratorAction(std::string particle  = "e-",
                           double      energyMeV = 10.0,
                           G4ThreeVector direction = G4ThreeVector(0,0,1),
                           G4ThreeVector position  = G4ThreeVector(0,0,-200));
    ~PrimaryGeneratorAction() override;

    void GeneratePrimaries(G4Event* event) override;

private:
    std::unique_ptr<G4ParticleGun> fGun;
};
