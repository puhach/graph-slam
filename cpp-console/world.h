#pragma once

#include <vector>
#include <iostream>

class World
{
	friend std::ostream& operator << (std::ostream& ostream, const World& world);

public:
	enum { MinWorld = 3, MaxWorld = 1000, MinLandmarks = 0, MaxLandmarks = 1000};

	World(int width, int height, int nLandmarks);

	std::pair<int, int> getWorldSize() const { return { this->width, this->height }; }

	void getWorldSize(int& width, int& height) const noexcept;

	int getWidth() const noexcept { return this->width; }

	int getHeight() const noexcept { return this->height; }

private:
	int width, height;
	std::vector<std::pair<double, double> > landmarks;	
};	// World


std::ostream& operator << (std::ostream& ostream, const World& world);