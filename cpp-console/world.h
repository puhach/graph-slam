#pragma once

#include <vector>
#include <iostream>
#include <random>

class World
{
	friend std::ostream& operator << (std::ostream& ostream, const World& world);


public:
	// TODO: consider using constexpr instead
	enum { MinWorld = 3, MaxWorld = 1000, MinLandmarks = 0, MaxLandmarks = 1000};

	World(int width, int height, int nLandmarks);

	std::pair<int, int> getWorldSize() const { return { this->width, this->height }; }

	void getWorldSize(int& width, int& height) const noexcept;

	int getWidth() const noexcept { return this->width; }

	int getHeight() const noexcept { return this->height; }

	std::size_t getLandmarkNum() const { return this->landmarks.size(); }

	std::pair<double, double> getLandmark(int lkIndex) const;

	Robot& createRobot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise);

	static std::mt19937& getRandomEngine();

private:
	int width, height;
	std::vector<std::pair<double, double> > landmarks;	
	RobotWrapper robot;
	double robotX, robotY;
};	// World


std::ostream& operator << (std::ostream& ostream, const World& world);