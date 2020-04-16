#include "genetic.h"

#include "world.h"

#include <algorithm>
#include <random>
//#include <typeinfo>	//debug

//template <typename T>
//class ShowType;


// TODO: consider passing measurementNoise and motionNoise to the constructor
std::pair<Positions, Positions> Genetic::localize(const Measurements& measurements, const Displacements& displacements, double measurementNoise, double motionNoise) const
{
	static_assert(std::is_same<std::tuple_element_t<0, Position>, std::tuple_element_t<1, Position>>::value, "Position types must be the same.");
	static_assert(std::is_same<std::tuple_element_t<1, LandmarkDistance>, std::tuple_element_t<2, LandmarkDistance>>::value, "Landmark distance types must be the same.");
	static_assert(std::is_same<std::tuple_element_t<0, Displacement>, std::tuple_element_t<1, Displacement>>::value, "Displacement types must be the same.");

	Measurements1D measurementsX(measurements.size()), measurementsY(measurements.size());
	for (int t = 0; t < measurements.size(); ++t)
	{
		measurementsX[t].resize(measurements[t].size());
		measurementsY[t].resize(measurements[t].size());
		
		for (int i = 0; i < measurements[t].size(); ++i)
		{
			const auto& [lkIndex, dx, dy] = measurements[t][i];
			measurementsX[t][i].first = lkIndex;
			measurementsX[t][i].second = dx;
			measurementsY[t][i].first = lkIndex;
			measurementsY[t][i].second = dy;
		}	// measurement
	}	// t

	Displacements1D displacementsX(displacements.size()), displacementsY(displacements.size());
	for (int t = 0; t < displacements.size(); ++t)
	{
		displacementsX[t] = displacements[t].first;
		displacementsY[t] = displacements[t].second;
	}	// t


	if (this->printEvery)
		std::cout << "Locating X coordinates..." << std::endl;

	const auto& [rposxv, lkposxv] = this->localize1D(measurementsX, displacementsX, measurementNoise, motionNoise, this->minX, this->maxX);

	if (this->printEvery)
		std::cout << "Locating Y coordinates..." << std::endl;

	const auto& [rposyv, lkposyv] = this->localize1D(measurementsY, displacementsY, measurementNoise, motionNoise, this->minY, this->maxY);

	if (rposxv.size() != rposyv.size())
		throw std::runtime_error("Robot X and Y position vectors mismatch.");

	if (lkposxv.size() != lkposyv.size())
		throw std::runtime_error("Landmark X and Y position vectors mismatch.");

	Positions rposv(rposxv.size()), lkposv(lkposxv.size());
	
	for (int i = 0; i < rposv.size(); ++i)
	{
		rposv[i].first = rposxv[i];
		rposv[i].second = rposyv[i];
	}

	for (int i = 0; i < lkposv.size(); ++i)
	{
		lkposv[i].first = lkposxv[i];
		lkposv[i].second = lkposyv[i];
	}

	return { rposv, lkposv };

}	// localize



std::pair<Genetic::Positions1D, Genetic::Positions1D> Genetic::localize1D(const Genetic::Measurements1D& measurements, const Genetic::Displacements1D& displacements
	, double measurementNoise, double motionNoise, Genetic::Position1D minPos, Genetic::Position1D maxPos) const
{
	int nTimesteps = static_cast<int>(measurements.size());

	// Narrow down the range of possible starting positions.

	Genetic::Position1D negDev = 0, posDev = 0, curPos = 0;
	for (const auto& d : displacements)
	{
		curPos += d;
		negDev = std::min(curPos, negDev);
		posDev = std::max(curPos, posDev);
	}

	if (minPos - negDev > maxPos - posDev)
		throw std::runtime_error("Unable to find a starting point.");

	std::uniform_real_distribution<Position1D> posDist(minPos-negDev, maxPos-posDev), lkDist(minPos, maxPos);


	// Initialize solution candidates for robot positions and landmark locations.

	std::vector<std::pair<Positions1D, Positions1D>> candidates(this->nCandidates);
	for (auto& [rposv, lkposv] : candidates)
	{
		
		rposv.resize(nTimesteps);
		rposv[0] = posDist(this->randomEngine);
		for (int t = 1; t < nTimesteps; ++t)
			///rposv[t] = std::min(std::max(minPos, rposv[t-1] + displacements[t]), maxPos);
			rposv[t] = rposv[t - 1] + displacements[t];
		//track(rposv);


		lkposv.resize(this->nLandmarks);
		std::generate(lkposv.begin(), lkposv.end(), [this, &lkDist]() -> double {
				//return posDist(randomEngine);
				return lkDist(this->randomEngine);
			});

	}

	// Evaluate each solution candidate according to the fitness functions.


	//auto fitness = [&measurements, &displacements, measurementNoise, motionNoise](Positions1D rposv, Positions1D lkposv) -> double
	//{
	//	double diff = 0, maxD = 0;

	//	for (int t = 1; t < rposv.size(); ++t)
	//	{
	//		double candD = rposv[t] - rposv[t - 1];
	//		double givenD = displacements[t];
	//		//diff += 1/motionNoise * (abs(candDx - givenDx) + abs(candDy - givenDy));
	//		double d = abs(candD - givenD);
	//		maxD = std::max(d, maxD);
	//		if (d > motionNoise)
	//			diff += d;
	//	}

	//	for (int t = 0; t < measurements.size(); ++t)
	//	{
	//		const auto& measurement = measurements[t];
	//		for (const auto& [lkIndex, lkD] : measurement)
	//		{
	//			double candD = lkposv[lkIndex] - rposv[t];
	//			//diff += 1 / measurementNoise * (abs(candDx - lkDx) + abs(candDy - lkDy));
	//			double d = abs(candD - lkD);
	//			maxD = std::max(d, maxD);
	//			if (d > measurementNoise)
	//				diff += d;
	//		}	// landmark
	//	}	// for

	//	
	//	// TODO: define eps constant
	//	return 1 / (diff + maxD*maxD + 1e-8);
	//	//return 1 / (maxD * maxD + 1e-8);
	//};


	auto fitness = [&measurements, &displacements, measurementNoise, motionNoise](Positions1D rposv, Positions1D lkposv) -> double
	{
		double diff = 0;

		for (int t = 1; t < rposv.size(); ++t)
		{
			double candD = rposv[t] - rposv[t - 1];
			double givenD = displacements[t];
			double d = abs(candD - givenD);
			//if (d > motionNoise)
				//	diff += d * d;
			diff = std::max(diff, d*d);
		}


		//for (const auto& measurement : measurements)
		for (int t = 0; t < measurements.size(); ++t)
		{
			const auto& measurement = measurements[t];

			for (const auto& [lkIndex, lkD] : measurement)
			{
				double candD = lkposv[lkIndex] - rposv[t];
				double d = abs(candD - lkD);
				//if (d > measurementNoise)
					//	diff += d * d;
				diff = std::max(diff, d * d);
			}	// landmark
		}

		// TODO: define eps constant
		return 1 / (diff + 1e-8);
	};


	//int bestCandIndex = 0;
	std::pair<Genetic::Positions1D, Genetic::Positions1D> bestCand = candidates[0];
	double maxFitness = 0;

	for (int epoch = 1; epoch <= this->nEpochs; ++epoch)
	{

		// For each candidate compute the probability to breed.

		int bestCandIndex = -1;		
		std::vector<double> probs(candidates.size());
		for (int i = 0; i < candidates.size(); ++i)
		{
			const auto& [rposv, lkposv] = candidates[i];
			double f = fitness(rposv, lkposv);
			probs[i] = f;
			if (f > maxFitness)
			{
				bestCandIndex = i;
				maxFitness = f;
			}
		}	// i

		if (bestCandIndex >= 0)
			bestCand = candidates[bestCandIndex];

		if (this->printEvery && epoch % this->printEvery == 0)
			std::cout << "Epoch " << epoch << " Fitness: " << maxFitness << std::endl;

		
		// Select parents.

		std::discrete_distribution<int> breedDist(probs.begin(), probs.end());
		std::vector<std::pair<Positions1D, Positions1D>> parents(candidates.size());
		for (int i = 0; i < candidates.size(); ++i)
		{
			int candIndex = breedDist(this->randomEngine);
			parents[i] = candidates[candIndex];
		}	// i


		// Cross over to produce new candidates.

		for (int i = 0; i < candidates.size(); i += 2)
		{
			crossOver(parents[i].second, parents[i + 1].second, candidates[i].second, candidates[i + 1].second);
		}	//	i

		// Mutate.

		std::discrete_distribution<int> mutationDist({ this->mutationChance, 1 - this->mutationChance });	// mutate/not mutate
		for (auto& cand : candidates)
		{
			
			if (mutationDist(this->randomEngine) == 0)	// mutate?
			{				
				auto& [rposv, lkposv] = cand;

				//rposv[0] = posDist(randomEngine);
				if (bestCand.first[0] > rposv[0])	// move towards the upper boundary
					rposv[0] = (rposv[0] + posDist.b()) / 2;
				else	// move towards the lower boundary
					rposv[0] = (posDist.a() + rposv[0]) / 2;

				// Adjust the rest of positions accordingly.
				for (int t = 1; t < nTimesteps; ++t)
				{
					rposv[t] = rposv[t - 1] + displacements[t];
					/*if (rposv[t] < minPos || rposv[t] > maxPos)
					{
						std::cout << "out of range" << rposv[t] << std::endl;
						rposv[0] = posDist(randomEngine);
						t = 0;
						continue;
					}*/
				}	// t

				mutate(lkposv, minPos, maxPos);
			}	// mutate
			
		}	// cand

	}	// epoch


	return bestCand;
}	// localize1D


template <typename Seq>
void Genetic::crossOver(const Seq& parent1, const Seq& parent2, Seq& child1, Seq& child2) const
{
	if (parent1.size() != parent2.size())
		throw std::runtime_error("Parent size mismatch.");

	if (child1.size() != child2.size())
		throw std::runtime_error("Children size mismatch.");

	if (parent1.size() != child1.size())
		throw std::runtime_error("Parent/children size mismatch.");


	int n = static_cast<int>(parent1.size());
	if (n < 2)
		throw std::runtime_error("At least 2 elements are needed to perform a crossover.");


	std::uniform_int_distribution<int> dist(1, n-1);	// [1, n-1]
	int splitIndex = dist(this->randomEngine);

	// child1 = parent1[0..splitIndex) + parent2[splitIndex..n)
	std::copy_n(parent1.begin(), splitIndex, child1.begin());	// par1 [0..splitIndex) -> child1 [0..splitIndex)
	std::copy_n(parent2.begin() + splitIndex, n - splitIndex, child1.begin() + splitIndex);	// par2 [splitIndex..n) -> child1[splitIndex..n)
	
	// child2 = parent2[0..splitIndex) + parent1[splitIndex..n)
	std::copy_n(parent2.begin(), splitIndex, child2.begin());	// par2 [0..splitIndex) -> child2 [0..splitIndex)
	std::copy_n(parent1.begin() + splitIndex, n - splitIndex, child2.begin() + splitIndex);	// par1 [splitIndex..n) -> child2 [splitIndex..n)
}	// crossOver

template <typename Seq>
void Genetic::mutate(Seq &posv, double min, double max) const
{
	if (posv.size() < 1)
		throw std::runtime_error("Can't mutate an empty sequence.");

	std::uniform_int_distribution<int> indexDist(0, static_cast<int>(posv.size())-1);	
	//std::uniform_real_distribution<double> xDist(0, this->world.getWidth()), yDist(0, this->world.getHeight());
	std::uniform_real_distribution<double> posDist(min, max);
	int index = indexDist(this->randomEngine);
	posv[index] = posDist(this->randomEngine);
}	// mutate
