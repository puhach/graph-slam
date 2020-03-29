#pragma once

#include <tuple>
#include <vector>
#include <random>
#include <functional>

//#include <Eigen/Dense>

class World;

// TODO: perhaps, rename it to LandmarkDisplacement or LkDisplacement
using LandmarkDistance = std::tuple<int, double, double>;	// (landmark_index, horizontal distance to landmark, vertical distance to landmark)
using Measurement = std::vector<LandmarkDistance>;
using Measurements = std::vector<Measurement>;
//using Position = std::pair<double, double>;
//using Positions = std::vector<Position>;
using Displacement = std::pair<double, double>;	// (dx from previous x, dy from previous y)
using Displacements = std::vector<Displacement>;
//using RobotPosition = std::pair<double, double>;
//using LandmarkPosition = std::tuple<int, double, double>;
using Position = std::pair<double, double>;	// Landmark or Robot position
using Positions = std::vector<Position>;

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

	std::pair<Positions, Positions> localize(double x0, double y0) const;

	void moveAndSense(int timesteps);



	//std::pair<Measurements, Displacements> moveAndSense(int timesteps);
	//std::pair<Positions, Positions> localize();

	//Measurements getMeasurements() const { return this->measurements; }
	//Displacements getDisplacements() const { return this->displacements; }

	//std::pair<Positions, Positions> localize() const;

protected:
	Robot(double sensorRange, double stepSize, double measurementNoise, double motionNoise, World &world
		, std::function<bool (World&, double, double)> move
		, std::function<Measurement (const World&)> senseLandmarks);

	virtual ~Robot() noexcept = default;

private:

	//Measurement sense() const;	
	//Displacement wander();
	//bool move(double dx, double dy);
	
	//void distortMotion(double& dx, double& dy) const;
	//void distortMeasurement(double& dx, double& dy) const;

	//static void addConstraints(Eigen::MatrixXd &omega, Eigen::VectorXd &xi, int i, int j, double d, double noise);

	double x, y, sensorRange, stepSize, measurementNoise, motionNoise;
	World& world;
	std::function<bool(World&, double, double)> move;
	std::function<Measurement(const World&)> senseLandmarks;

	Measurements measurements;
	Displacements displacements;
};	// Robot

