#pragma once

#include <tuple>
#include <vector>

class World;

using LandmarkDistance = std::tuple<int, double, double>;	// (landmark_index, horizontal distance to landmark, vertical distance to landmark)
using Measurement = std::vector<LandmarkDistance>;
using Measurements = std::vector<Measurement>;
using Position = std::pair<double, double>;
using Positions = std::vector<Position>;

class Robot
{
public:

	enum { MaxTimeSteps = 1000 };

	Robot(int x, int y, World &world);

	std::pair<Measurements, Positions> moveAndSense(int timesteps);

private:
	int x, y;	// TODO: change to double
	World& world;
};	// Robot

