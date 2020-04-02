#pragma once

#include "position.h"
#include "measurement.h"

#include <tuple>
#include <vector>
#include <random>
#include <functional>

//#include <Eigen/Dense>

class World;


class Robot
{
public:

	// TODO: consider using constexpr instead
	enum { MaxTimeSteps = 1000 };

	//Robot(int x, int y, World &world);
	//Robot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise, World &world);
	//virtual ~Robot() noexcept = default;

	double getSensorRange() const { return this->sensorRange; }

	double getStepSize() const { return this->stepSize; }

	double getMeasurementNoise() const { return this->measurementNoise; }

	double getMotionNoise() const { return this->motionNoise; }

	void sense();
	void roamAndSense();

	//std::pair<Positions, Positions> localize(double x0, double y0) const;
	template <class Localizer>
	std::pair<Positions, Positions> localize(const Localizer &localizer) const;

	void moveAndSense(int timesteps);



	//std::pair<Measurements, Displacements> moveAndSense(int timesteps);
	//std::pair<Positions, Positions> localize();

	//Measurements getMeasurements() const { return this->measurements; }
	//Displacements getDisplacements() const { return this->displacements; }

	//std::pair<Positions, Positions> localize() const;

protected:
	Robot(double sensorRange, double stepSize, double measurementNoise, double motionNoise, World &world
		, std::function<bool (World&, double dx, double dy, double noise)> move
		, std::function<Measurement (const World&, double range, double noise)> senseLandmarks);

	virtual ~Robot() noexcept = default;

private:

	//Measurement sense() const;	
	//Displacement wander();
	//bool move(double dx, double dy);
	
	//void distortMotion(double& dx, double& dy) const;
	//void distortMeasurement(double& dx, double& dy) const;

	

	double sensorRange, stepSize, measurementNoise, motionNoise;
	World& world;
	std::function<bool(World&, double dx, double dy, double noise)> move;
	std::function<Measurement(const World&, double range, double noise)> senseLandmarks;

	Measurements measurements;
	Displacements displacements;
};	// Robot

