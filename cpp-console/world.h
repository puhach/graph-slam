#pragma once


#include <vector>
#include <iostream>
#include <random>
#include <functional>

class Robot;

using LandmarkDistance = std::tuple<int, double, double>;	// (landmark_index, horizontal distance to landmark, vertical distance to landmark)
using Measurement = std::vector<LandmarkDistance>;

class World
{
	friend std::ostream& operator << (std::ostream& ostream, const World& world);

	class RobotWrapper;

public:
	// TODO: consider using constexpr instead
	enum { MinWorld = 3, MaxWorld = 1000, MinLandmarks = 0, MaxLandmarks = 1000};

	World(int width, int height, int nLandmarks);
	~World() noexcept;
	

	std::pair<int, int> getWorldSize() const { return { this->width, this->height }; }

	void getWorldSize(int& width, int& height) const noexcept;

	int getWidth() const noexcept { return this->width; }

	int getHeight() const noexcept { return this->height; }

	// TODO: we, probably, don't need these methods anymore
	std::size_t getLandmarkNum() const { return this->landmarks.size(); }
	std::pair<double, double> getLandmark(int lkIndex) const;

	Robot& createRobot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise);

	double getRobotX() const { return this->robotX; }

	double getRobotY() const { return this->robotY; }

	static std::mt19937& getRandomEngine();

private:

	// TODO: add noise parameter to this function
	bool moveRobot(double dx, double dy, double noise);
	Measurement revealLandmarks(double range, double noise) const;

	int width, height;
	std::vector<std::pair<double, double> > landmarks;	
	//RobotWrapper robot;
	double robotX, robotY;
	std::unique_ptr<RobotWrapper> robot;
};	// World


std::ostream& operator << (std::ostream& ostream, const World& world);