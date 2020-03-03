#pragma once

#include <vector>

class World
{
public:
	enum { MinWorld = 3, MaxWorld = 1000, MinLandmarks = 1, MaxLandmarks = 1000};

	World(int width, int height, int nLandmarks);

private:
	int width, height;
	std::vector<std::tuple<int, int> > landmarks;
};	// World

