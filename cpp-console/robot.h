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
	Robot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise, World &world);

	// TODO: add getters and setters for the Robot's parameters

	std::pair<Measurements, Displacements> moveAndSense(int timesteps);

private:

	Measurement sense() const;	
	Displacement wander();
	bool move(double dx, double dy);
	void distortMotion(double& dx, double& dy);
	void distortMeasurement(double& dx, double& dy) const;

	double x, y, sensorRange, stepSize, measurementNoise, motionNoise;
	World& world;

	
	//static thread_local std::mt19937 randomEngine;
};	// Robot

