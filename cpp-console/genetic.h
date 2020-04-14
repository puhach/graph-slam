#pragma once

#include "position.h"
#include "measurement.h"

#include <vector>
#include <utility>


class World;

class Genetic
{
public:
	//Genetic(int nLandmarks)
	//	: nLandmarks(nLandmarks) {}

	Genetic(const World &world)
		: world(world) { }

	// TEST!
	void giveRPositions(const Positions& rposv) { this->actualRPos = rposv; }

	std::pair<Positions, Positions> localize(const Measurements& measurements, const Displacements& displacements, double measurementNoise, double motionNoise, int epochs = 10000) const;

private:

	void crossOver(const Positions& parent1, const Positions& parent2, Positions &child1, Positions &child2) const;

	void mutate(Positions& posv) const;

	// TEST!
	Positions actualRPos;

	const World& world;
};	// Genetic