#pragma once

#include <tuple>
#include <vector>
#include <random>

class World;

// TODO: perhaps, rename it to LandmarkDisplacement or LkDisplacement
using LandmarkDistance = std::tuple<int, double, double>;	// (landmark_index, horizontal distance to landmark, vertical distance to landmark)
using Measurement = std::vector<LandmarkDistance>;
using Measurements = std::vector<Measurement>;
//using Position = std::pair<double, double>;
//using Positions = std::vector<Position>;
using Displacement = std::pair<double, double>;	// (dx from previous x, dy from previous y)
using Displacements = std::vector<Displacement>;

class Robot
{
public:

	enum { MaxTimeSteps = 1000 };

	//Robot(int x, int y, World &world);
	Robot(double x, double y, World &world);

	std::pair<Measurements, Displacements> moveAndSense(int timesteps);

private:

	Measurement sense() const;
	Displacement wander();

	double x, y;	
	World& world;

	// As long as we are not using threads thread_local is not necessary here.
	static thread_local std::mt19937 randomEngine;
};	// Robot

