#include "PrimaryGeneratorAction.hh"
#include "G4ParticleTable.hh"
#include "G4SystemOfUnits.hh"

PrimaryGeneratorAction::PrimaryGeneratorAction(std::string particle,
                                                double energyMeV,
                                                G4ThreeVector direction,
                                                G4ThreeVector position)
    : fGun(std::make_unique<G4ParticleGun>(1))
{
    auto* def = G4ParticleTable::GetParticleTable()->FindParticle(particle);
    if (!def) throw std::runtime_error("Unknown particle: " + particle);
    fGun->SetParticleDefinition(def);
    fGun->SetParticleEnergy(energyMeV * MeV);
    fGun->SetParticleMomentumDirection(direction);
    fGun->SetParticlePosition(position * mm);
}

PrimaryGeneratorAction::~PrimaryGeneratorAction() = default;

void PrimaryGeneratorAction::GeneratePrimaries(G4Event* event) {
    fGun->GeneratePrimaryVertex(event);
}
