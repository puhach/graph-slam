#pragma once

#include <vector>
#include <iostream>

class World
{
	friend std::ostream& operator << (std::ostream& ostream, const World& world);

public:
	enum { MinWorld = 3, MaxWorld = 1000, MinLandmarks = 0, MaxLandmarks = 1000};

	World(int width, int height, int nLandmarks);

private:
	int width, height;
	std::vector<std::pair<int, int> > landmarks;
};	// World


std::ostream& operator << (std::ostream& ostream, const World& world);