#pragma once

#include "position.h"
#include "measurement.h"

#include <vector>
#include <utility>
#include <random>
#include <stdexcept>

class World;


class Genetic
{
	using Position1D = decltype(std::declval<Position>().first);
	using Positions1D = std::vector<Position1D>;
	using LkDistance1D = std::tuple_element_t<1, LandmarkDistance>;	// tuple<int, double, double>: 1 and 2 must be the same
	using Measurement1D = std::vector<std::pair<int, LkDistance1D>>;
	using Measurements1D = std::vector<Measurement1D>;
	using Displacement1D = std::tuple_element_t<0, Displacement>;	// pair<double, double>: 0 and 1 must be the same
	using Displacements1D = std::vector<Displacement1D>;


public:
	Genetic(double minX, double maxX, double minY, double maxY, int nLandmarks, int nEpochs = 1000, int nCandidates = 2000, double mutationChance = 0.7, int printEvery = 0)
		: minX(minX)
		, maxX(maxX > minX ? maxX : throw std::invalid_argument("maxX must be greater than minX."))
		, minY(minY)
		, maxY(maxY > minY ? maxY : throw std::invalid_argument("maxY must be greater than minY."))
		, nLandmarks(nLandmarks > 0 ? nLandmarks : throw std::invalid_argument("The number of landmarks must be positive."))
		, nEpochs(nEpochs > 0 ? nEpochs : throw std::invalid_argument("The number of epochs must be positive."))
		, nCandidates(nCandidates > 0 ? nCandidates : throw std::invalid_argument("The number of candidates must be positive."))
		, mutationChance(mutationChance >= 0 && mutationChance <= 1 ? mutationChance : throw std::invalid_argument("mutationChance must be in range [0..1]."))
		, printEvery(printEvery)
		, randomEngine(std::random_device{}()) {}

	std::pair<Positions, Positions> localize(const Measurements& measurements, const Displacements& displacements, double measurementNoise, double motionNoise) const;

private:

	std::pair<Positions1D, Positions1D> localize1D(const Measurements1D& measurements, const Displacements1D& displacements
		, double measurementNoise, double motionNoise, Position1D minPos, Position1D maxPos) const;

	template <typename Seq>
	void crossOver(const Seq &parent1, const Seq &parent2, Seq &child1, Seq &child2) const;

	template <typename Seq>
	void mutate(Seq& posv, double min, double max) const;

	double minX, maxX, minY, maxY;
	int nLandmarks, nEpochs, nCandidates;
	double mutationChance;
	int printEvery;
	mutable std::mt19937 randomEngine;
};	// Genetic