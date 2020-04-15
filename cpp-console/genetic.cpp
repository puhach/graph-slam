#include "genetic.h"

#include "world.h"

#include <algorithm>
#include <random>
#include <typeinfo>	//debug

//template <typename T>
//class ShowType;

// TEST!
bool stage = 0;

// TODO: consider passing measurementNoise and motionNoise to the constructor
std::pair<Positions, Positions> Genetic::localize(const Measurements& measurements, const Displacements& displacements, double measurementNoise, double motionNoise, int epochs) const
{
	static_assert(std::is_same<std::tuple_element_t<0, Position>, std::tuple_element_t<1, Position>>::value, "Position types must be the same.");
	static_assert(std::is_same<std::tuple_element_t<1, LandmarkDistance>, std::tuple_element_t<2, LandmarkDistance>>::value, "Landmark distance types must be the same.");
	static_assert(std::is_same<std::tuple_element_t<0, Displacement>, std::tuple_element_t<1, Displacement>>::value, "Displacement types must be the same.");

	//ShowType<typename Genetic::LkDistance1D> dummy;
	//static_assert(std::is_same<typename Genetic::LkDistance1D, double>::value, "LkDistance1D must be double.");

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

	stage = 0;
	const auto& [rposxv, lkposxv] = this->localize1D(measurementsX, displacementsX, measurementNoise, motionNoise, 0, this->world.getWidth(), epochs);
	stage = 1;
	const auto& [rposyv, lkposyv] = this->localize1D(measurementsY, displacementsY, measurementNoise, motionNoise, 0, this->world.getHeight(), epochs);

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
	, double measurementNoise, double motionNoise, Genetic::Position1D minPos, Genetic::Position1D maxPos, int epochs) const
{
	int nCandidates = 2000;
	double mutationChance = 0.8;
	int nTimesteps = measurements.size();

	// TODO: probably, pass world dimensions and random engine (or maybe use a standalone engine) to the constructor instead of world instance.
	//thread_local std::mt19937 randomEngine(std::random_device{}());
	auto& randomEngine = this->world.getRandomEngine();
	//std::uniform_real_distribution<double> xDist(0, this->world.getWidth()), yDist(0, this->world.getHeight());
	std::uniform_real_distribution<Position1D> posDist(minPos, maxPos);


	// Initialize solution candidates for robot positions and landmark locations.

	//std::vector<std::pair<Positions, Positions>> candidates(nCandidates);

	//std::vector<std::tuple<XPositions, YPositions, XPositions, YPositions>> candidates(nCandidates);
	std::vector<std::pair<Positions1D, Positions1D>> candidates(nCandidates);
	for (auto& [rposv, lkposv] : candidates)
	{
		// TEST!
		/*rposv.resize(this->actualRPos.size());
		for (int i = 0; i < rposv.size(); ++i)
			rposv[i] = stage == 0 ? this->actualRPos[i].first : this->actualRPos[i].second;

		lkposv.resize(this->world.getLandmarkNum());
		for (int i = 0; i < this->world.getLandmarkNum(); ++i)
			lkposv[i] = stage == 0 ? this->world.getLandmark(i).first : this->world.getLandmark(i).second;*/

		

		rposv.resize(nTimesteps);
		std::generate(rposv.begin(), rposv.end(), [&randomEngine, &posDist]() -> double {
				return posDist(randomEngine);
			});

		// TODO: try to get rid of this->world
		lkposv.resize(this->world.getLandmarkNum());
		std::generate(lkposv.begin(), lkposv.end(), [&randomEngine, &posDist]() -> double {
				return posDist(randomEngine);
			});

	}

	// Evaluate each solution candidate according to the fitness functions.

	//auto fitness = [&measurements, &displacements, measurementNoise, motionNoise](Positions rposv, Positions lkposv) -> double
	//{
	//	double diff = 0;

	//	for (int t = 1; t < rposv.size(); ++t)
	//	{
	//		double candDx = rposv[t].first - rposv[t - 1].first,
	//				candDy = rposv[t].second - rposv[t - 1].second;

	//		double givenDx = displacements[t].first,
	//				givenDy = displacements[t].second;

	//		//diff += 1/motionNoise * (abs(candDx - givenDx) + abs(candDy - givenDy));
	//		//diff += (abs(candDx - givenDx) + abs(candDy - givenDy));
	//		
	//		double dx = abs(candDx - givenDx), dy = abs(candDy - givenDy);
	//		if (dx > motionNoise)
	//			diff += dx*dx;
	//			
	//		if (dy > motionNoise)
	//			diff += dy*dy;
	//	}


	//	//for (const auto& measurement : measurements)
	//	for (int t = 0; t < measurements.size(); ++t)
	//	{
	//		const auto& measurement = measurements[t];

	//		for (const auto& [lkIndex, lkDx, lkDy] : measurement)
	//		{
	//			double candDx = lkposv[lkIndex].first - rposv[t].first,
	//				candDy = lkposv[lkIndex].second - rposv[t].second;

	//			//diff += 1 / measurementNoise * (abs(candDx - lkDx) + abs(candDy - lkDy));
	//			//diff += (abs(candDx - lkDx) + abs(candDy - lkDy));
	//			double dx = abs(candDx - lkDx), dy = abs(candDy - lkDy);
	//			if (dx > measurementNoise)
	//				diff += dx*dx;

	//			if (dy > measurementNoise)
	//				diff += dy*dy;
	//		}	// landmark
	//	}

	//	// TODO: define eps constant
	//	return 1/(diff + 1e-8);
	//};

	auto fitness = [&measurements, &displacements, measurementNoise, motionNoise](Positions1D rposv, Positions1D lkposv) -> double
	{
		double diff = 0;

		for (int t = 1; t < rposv.size(); ++t)
		{
			double candD = rposv[t] - rposv[t - 1];

			double givenD = displacements[t];

			//diff += 1/motionNoise * (abs(candDx - givenDx) + abs(candDy - givenDy));
			//diff += (abs(candDx - givenDx) + abs(candDy - givenDy));

			double d = abs(candD - givenD);
			if (d > motionNoise)
				diff += d * d;

		}


		//for (const auto& measurement : measurements)
		for (int t = 0; t < measurements.size(); ++t)
		{
			const auto& measurement = measurements[t];

			for (const auto& [lkIndex, lkD] : measurement)
			{
				double candD = lkposv[lkIndex] - rposv[t];

				//diff += 1 / measurementNoise * (abs(candDx - lkDx) + abs(candDy - lkDy));
				//diff += (abs(candDx - lkDx) + abs(candDy - lkDy));
				double d = abs(candD - lkD);
				if (d > measurementNoise)
					diff += d * d;
			}	// landmark
		}

		// TODO: define eps constant
		return 1 / (diff + 1e-8);
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
			const auto& [rposv, lkposv] = candidates[i];
			//double f = fitness(candidates[i].first, candidates[i].second);
			double f = fitness(rposv, lkposv);
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

		//std::vector<std::pair<Positions, Positions>> parents(nCandidates);
		std::vector<std::pair<Positions1D, Positions1D>> parents(nCandidates);
		for (int i = 0; i < nCandidates; ++i)
		{
			int candIndex = breedDist(randomEngine);
			parents[i] = candidates[candIndex];
		}


		// Cross over to produce new candidates.

		for (int i = 0; i < nCandidates; i += 2)
		{
			if (epoch % 2)
				crossOver(parents[i].first, parents[i + 1].first, candidates[i].first, candidates[i + 1].first);
			else
				crossOver(parents[i].second, parents[i + 1].second, candidates[i].second, candidates[i + 1].second);

			//auto& [rposxv1, rposyv1, lkposxv1, lkposyv1] = parents[i];
			//auto& [rposxv2, rposyv2, lkposyv2, lkposyv2] = parents[i+1];
			//crossOver(std::get<0>(parents[i]), std::get<0>(parents[i + 1]), std::get<0>(candidates[i]), std::get<0>(candidates[i + 1]));
			//crossOver(std::get<1>(parents[i]), std::get<1>(parents[i + 1]), std::get<1>(candidates[i]), std::get<1>(candidates[i + 1]));
			//crossOver(std::get<2>(parents[i]), std::get<2>(parents[i + 1]), std::get<2>(candidates[i]), std::get<2>(candidates[i + 1]));
			//crossOver(std::get<3>(parents[i]), std::get<3>(parents[i + 1]), std::get<3>(candidates[i]), std::get<3>(candidates[i + 1]));
		}

		// Mutate.

		std::discrete_distribution<int> mutationDist({ mutationChance, 1 - mutationChance });	// mutate/not mutate
		for (auto& cand : candidates)
		{
			if (mutationDist(randomEngine) == 0)	// mutate?
			{
				auto& [rposv, lkposv] = cand;
				if (epoch % 2)
					mutate(rposv, minPos, maxPos);
				else
					mutate(lkposv, minPos, maxPos);
			}	// mutate
		}	// cand

	}	// epoch


	////const auto& bestCand = candidates[bestCandIndex];
	//const auto& [rposv, lkposv] = candidates[bestCandIndex];

	//Positions rposv(nTimesteps);
	//for (int t = 0; t < nTimesteps; ++t)
	//{
	//	//rposv[i].first = std::get<0>(candidates[bestCandIndex]), std::get<1>(candidates[bestCandIndex]) };
	//	rposv[t].first = rposv[t];
	//	rposv[t].second = rposyv[t];
	//}

	//Positions lkposv(this->world.getLandmarkNum());
	//for (int i = 0; i < this->world.getLandmarkNum(); ++i)
	//{
	//	//lkposv[i] = { std::get<2>(candidates[bestCandIndex]), std::get<3>(candidates[bestCandIndex]) };
	//	lkposv[i].first = lkposxv[i];
	//	lkposv[i].second = lkposyv[i];
	//}

	//return std::pair<Positions, Positions>(candidates[bestCandIndex].first, candidates[bestCandIndex].second);
	//return std::pair<Positions, Positions>(rposv, lkposv);
	return candidates[bestCandIndex];
}	// localize1D


//void Genetic::crossOver(const Positions& parent1, const Positions& parent2, Positions& child1, Positions& child2) const
template <typename Seq>
void Genetic::crossOver(const Seq& parent1, const Seq& parent2, Seq& child1, Seq& child2) const
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

//void Genetic::mutate(Positions& posv) const
template <typename Seq>
void Genetic::mutate(Seq &posv, double min, double max) const
{
	if (posv.size() < 1)
		throw std::runtime_error("Can't mutate an empty sequence.");

	auto& randomEngine = this->world.getRandomEngine();

	std::uniform_int_distribution<int> indexDist(0, posv.size()-1);	
	//std::uniform_real_distribution<double> xDist(0, this->world.getWidth()), yDist(0, this->world.getHeight());
	std::uniform_real_distribution<double> posDist(min, max);
	int index = indexDist(randomEngine);
	posv[index] = posDist(randomEngine);
	//posv[index].first = xDist(randomEngine);
	//posv[index].second = yDist(randomEngine);
}
