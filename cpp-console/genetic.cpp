#include "genetic.h"

#include "world.h"

#include <algorithm>
#include <random>


// TODO: consider passing measurementNoise and motionNoise to the constructor
std::pair<Positions, Positions> Genetic::localize(const Measurements& measurements, const Displacements& displacements, double measurementNoise, double motionNoise, int epochs) const
{
	int nCandidates = 2000;
	double mutationChance = 0.8;
	int nTimesteps = measurements.size();

	// TODO: probably, pass world dimensions and random engine (or maybe use a standalone engine) to the constructor instead of world instance.
	//thread_local std::mt19937 randomEngine(std::random_device{}());
	auto& randomEngine = this->world.getRandomEngine();
	std::uniform_real_distribution<double> xDist(0, this->world.getWidth()), yDist(0, this->world.getHeight());


	// Initialize solution candidates for robot positions and landmark locations.

	std::vector<std::pair<Positions, Positions>> candidates(nCandidates);
	for (auto& [rposv, lkposv] : candidates)
	{
		// TEST!
		/*rposv = this->actualRPos;
		lkposv.resize(this->world.getLandmarkNum());
		for (int i = 0; i < this->world.getLandmarkNum(); ++i)
			lkposv[i] = this->world.getLandmark(i);*/

		rposv.resize(nTimesteps);
		std::generate(rposv.begin(), rposv.end(), [&randomEngine, &xDist, &yDist]() -> std::pair<double, double> {
				return { xDist(randomEngine), yDist(randomEngine) };
			});

		lkposv.resize(this->world.getLandmarkNum());
		std::generate(lkposv.begin(), lkposv.end(), [&randomEngine, &xDist, &yDist]() -> std::pair<double, double> {
				return { xDist(randomEngine), yDist(randomEngine) };
			});
	}

	// Evaluate each solution candidate according to the fitness functions.

	auto fitness = [&measurements, &displacements, measurementNoise, motionNoise](Positions rposv, Positions lkposv) -> double
	{
		double diff = 0;

		for (int t = 1; t < rposv.size(); ++t)
		{
			double candDx = rposv[t].first - rposv[t - 1].first,
					candDy = rposv[t].second - rposv[t - 1].second;

			double givenDx = displacements[t].first,
					givenDy = displacements[t].second;

			//diff += 1/motionNoise * (abs(candDx - givenDx) + abs(candDy - givenDy));
			//diff += (abs(candDx - givenDx) + abs(candDy - givenDy));
			
			double dx = abs(candDx - givenDx), dy = abs(candDy - givenDy);
			if (dx > motionNoise)
				diff += dx*dx;
				
			if (dy > motionNoise)
				diff += dy*dy;
		}


		//for (const auto& measurement : measurements)
		for (int t = 0; t < measurements.size(); ++t)
		{
			const auto& measurement = measurements[t];

			for (const auto& [lkIndex, lkDx, lkDy] : measurement)
			{
				double candDx = lkposv[lkIndex].first - rposv[t].first,
					candDy = lkposv[lkIndex].second - rposv[t].second;

				//diff += 1 / measurementNoise * (abs(candDx - lkDx) + abs(candDy - lkDy));
				//diff += (abs(candDx - lkDx) + abs(candDy - lkDy));
				double dx = abs(candDx - lkDx), dy = abs(candDy - lkDy);
				if (dx > measurementNoise)
					diff += dx*dx;

				if (dy > measurementNoise)
					diff += dy*dy;
			}	// landmark
		}

		// TODO: define eps constant
		return 1/(diff + 1e-8);
	};

	int bestCandIndex = 0;

	for (int epoch = 1; epoch <= epochs; ++epoch)
	{

		// For each candidate compute the probability to breed.
	
		bestCandIndex = 0;
		double maxFitness = 0;
		std::vector<double> probs(nCandidates);
		for (int i = 0; i < nCandidates; ++i)
		{
			double f = fitness(candidates[i].first, candidates[i].second);
			probs[i] = f;
			if (f > maxFitness)
			{
				maxFitness = bestCandIndex;
				maxFitness = f;

				//if (maxFitness > )
			}
		}

		// TEST!
		if (epoch % 100 == 0)
			std::cout << "genetic:" << epoch << " " << maxFitness << std::endl;

		std::discrete_distribution<int> breedDist(probs.begin(), probs.end());


		// Select parents.

		std::vector<std::pair<Positions, Positions>> parents(nCandidates);
		for (int i = 0; i < nCandidates; ++i)
		{
			int candIndex = breedDist(randomEngine);
			parents[i] = candidates[candIndex];
		}


		// Cross over to produce new candidates.

		for (int i = 0; i < nCandidates; i += 2)
		{
			crossOver(parents[i].first, parents[i + 1].first, candidates[i].first, candidates[i + 1].first);
			crossOver(parents[i].second, parents[i + 1].second, candidates[i].second, candidates[i + 1].second);
			//auto& [candidates[i].first, candidates[i + 1].first] = crossOver();
		}

		// Mutate.

		std::discrete_distribution<int> mutationDist({mutationChance, 1 - mutationChance});	// mutate/not mutate
		for (auto& cand : candidates)
		{
			if (mutationDist(randomEngine) == 0)	// mutate?
			{
				mutate(cand.first);
				mutate(cand.second);
			}	// mutate
		}	// cand

	}	// epoch

	return std::pair<Positions, Positions>(candidates[bestCandIndex].first, candidates[bestCandIndex].second);
}	// localize


void Genetic::crossOver(const Positions& parent1, const Positions& parent2, Positions& child1, Positions& child2) const
{
	if (parent1.size() != parent2.size())
		throw std::runtime_error("Parent size mismatch.");

	if (child1.size() != child2.size())
		throw std::runtime_error("Children size mismatch.");

	if (parent1.size() != child1.size())
		throw std::runtime_error("Parent/children size mismatch.");


	int n = parent1.size();
	if (n < 2)
		throw std::runtime_error("At least 2 elements are needed to perform a crossover.");


	auto& randomEngine = this->world.getRandomEngine();

	std::uniform_int_distribution<int> dist(1, n-1);	// [1, n-1]
	int splitIndex = dist(randomEngine);

	// child1 = parent1[0..splitIndex) + parent2[splitIndex..n)
	std::copy_n(parent1.begin(), splitIndex, child1.begin());	// par1 [0..splitIndex) -> child1 [0..splitIndex)
	std::copy_n(parent2.begin() + splitIndex, n - splitIndex, child1.begin() + splitIndex);	// par2 [splitIndex..n) -> child1[splitIndex..n)
	
	// child2 = parent2[0..splitIndex) + parent1[splitIndex..n)
	std::copy_n(parent2.begin(), splitIndex, child2.begin());	// par2 [0..splitIndex) -> child2 [0..splitIndex)
	std::copy_n(parent1.begin() + splitIndex, n - splitIndex, child2.begin() + splitIndex);	// par1 [splitIndex..n) -> child2 [splitIndex..n)

}	// crossOver

void Genetic::mutate(Positions& posv) const
{
	if (posv.size() < 1)
		throw std::runtime_error("Can't mutate an empty sequence.");

	auto& randomEngine = this->world.getRandomEngine();

	std::uniform_int_distribution<int> indexDist(0, posv.size()-1);	
	std::uniform_real_distribution<double> xDist(0, this->world.getWidth()), yDist(0, this->world.getHeight());
	int index = indexDist(randomEngine);
	posv[index].first = xDist(randomEngine);
	posv[index].second = yDist(randomEngine);
}
