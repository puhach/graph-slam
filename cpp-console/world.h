#pragma once

#include "lkdistance.h"
#include "measurement.h"

#include <vector>
#include <iostream>
#include <random>
#include <functional>

class Robot;


class World
{
	friend std::ostream& operator << (std::ostream& ostream, const World& world);

	class RobotWrapper;

public:
	static constexpr int MinWorld = 3;
	static constexpr int MaxWorld = 1000;
	static constexpr int MinLandmarks = 0;
	static constexpr int MaxLandmarks = 1000;

	World(int width, int height, int nLandmarks);
	
	World(const World& other) = delete;
	
	World(World&& other) = default;

	~World() noexcept;
	
	World& operator = (const World& other) = delete;

	World& operator = (World&& other) = default;

	std::pair<int, int> getWorldSize() const { return { this->width, this->height }; }

	void getWorldSize(int& width, int& height) const noexcept;

	int getWidth() const noexcept { return this->width; }

	int getHeight() const noexcept { return this->height; }

	std::size_t getLandmarkNum() const { return this->landmarks.size(); }

	std::pair<double, double> getLandmark(int lkIndex) const;

	Robot& createRobot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise);

	double getRobotX() const { return this->robotX; }

	double getRobotY() const { return this->robotY; }

	static std::mt19937& getRandomEngine();

private:

	bool moveRobot(double dx, double dy, double noise);
	Measurement revealLandmarks(double range, double noise) const;

	int width, height;
	std::vector<std::pair<double, double> > landmarks;	
	double robotX, robotY;
	// Robot object was replaced by a smart pointer, otherwise we would have to deal with a Robot instance in invalid state.
	std::unique_ptr<RobotWrapper> robot;
	//RobotWrapper robot;
};	// World


std::ostream& operator << (std::ostream& ostream, const World& world);