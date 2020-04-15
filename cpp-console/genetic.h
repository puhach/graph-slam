#pragma once

#include "position.h"
#include "measurement.h"

#include <vector>
#include <utility>


class World;


class Genetic
{
	using Position1D = decltype(std::declval<Position>().first);
	using Positions1D = std::vector<Position1D>;
	//using LkDistance1D = double;	
	using LkDistance1D = std::tuple_element_t<1, LandmarkDistance>;	// tuple<int, double, double>: 1 and 2 must be the same
	//using LkDistance1D = std::tuple_element<2, LandmarkDistance>;
	using Measurement1D = std::vector<std::pair<int, LkDistance1D>>;
	//using Measurement1D = std::vector<std::pair<int, double>>;
	using Measurements1D = std::vector<Measurement1D>;
	using Displacement1D = std::tuple_element_t<0, Displacement>;	// pair<double, double>: 0 and 1 must be the same
	//using Displacement1D = std::tuple_element<1, Displacement>;
	using Displacements1D = std::vector<Displacement1D>;


public:
	//Genetic(int nLandmarks)
	//	: nLandmarks(nLandmarks) {}

	Genetic(const World &world)
		: world(world) { }

	// TEST!
	void giveRPositions(const Positions& rposv) { this->actualRPos = rposv; }

	std::pair<Positions, Positions> localize(const Measurements& measurements, const Displacements& displacements, double measurementNoise, double motionNoise, int epochs = 1000) const;

private:

	std::pair<Positions1D, Positions1D> localize1D(const Measurements1D& measurements, const Displacements1D& displacements, double measurementNoise, double motionNoise
		, Position1D minPos, Position1D maxPos, int epochs) const;

	//void crossOver(const Positions& parent1, const Positions& parent2, Positions &child1, Positions &child2) const;
	//void crossOver(const XPositions& parent1, const XPositions& parent2, XPositions& child1, XPositions& child2) const;
	template <typename Seq>
	void crossOver(const Seq &parent1, const Seq &parent2, Seq &child1, Seq &child2) const;

	//void mutate(Positions& posv) const;
	template <typename Seq>
	void mutate(Seq& posv, double min, double max) const;

	// TEST!
	Positions actualRPos;

	const World& world;
};	// Genetic