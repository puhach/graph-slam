#pragma once

#include <vector>

class World
{
public:
	World(int width, int height, int nLandmarks);

private:
	int width, height;
	std::vector<std::tuple<int, int> > landmarks;
};	// World

