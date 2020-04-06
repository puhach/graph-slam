#pragma once

#include "position.h"
#include "measurement.h"

#include <tuple>
#include <vector>
#include <random>
#include <functional>


class World;


class Robot
{
public:

	// TODO: consider using constexpr instead
	enum { MaxTimeSteps = 1000 };

	// TODO: add copy/move constructors and assignment operators

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


protected:
	Robot(double sensorRange, double stepSize, double measurementNoise, double motionNoise, World &world
		, std::function<bool (World&, double dx, double dy, double noise)> move
		, std::function<Measurement (const World&, double range, double noise)> senseLandmarks);

	virtual ~Robot() noexcept = default;

private:

	double sensorRange, stepSize, measurementNoise, motionNoise;
	World& world;
	std::function<bool(World&, double dx, double dy, double noise)> move;
	std::function<Measurement(const World&, double range, double noise)> senseLandmarks;

	Measurements measurements;
	Displacements displacements;
};	// Robot

