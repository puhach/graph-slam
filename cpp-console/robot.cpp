#include "robot.h"
#include "world.h"
#include "graphslam.h"

#include <exception>
#include <cmath>
//#include <numbers>


template std::pair<Positions, Positions>  Robot::localize<GraphSlam>(const GraphSlam &method) const;



// TODO: perhaps, we need a Sensor class to group several parameters
Robot::Robot(double sensorRange, double stepSize, double measurementNoise, double motionNoise, World &world
			, std::function<bool (World&, double dx, double dy, double noise)> move
			, std::function<Measurement (const World&, double range, double noise)> senseLandmarks)
	: sensorRange(sensorRange > 0 ? sensorRange : throw std::invalid_argument("Robot's sensor range is invalid."))
	, stepSize(stepSize > 0 && stepSize < world.getHeight() && stepSize < world.getWidth() ? stepSize : throw std::invalid_argument("Robot's step size is invalid."))
	, measurementNoise(measurementNoise > 0 && measurementNoise < sensorRange ? measurementNoise : throw std::invalid_argument("Robot's measurement noise must be in range (0, sensorRange)."))
	, motionNoise(motionNoise > 0 && motionNoise < stepSize ? motionNoise : throw std::invalid_argument("Robot's motion noise must be in range (0, stepSize)."))
	, world(world)
	, move(std::move(move))
	, senseLandmarks(std::move(senseLandmarks))
{

}


// TODO: implement moving by the specified distance
void Robot::moveAndSense(int timesteps)
{
	if (timesteps < 1 || timesteps > MaxTimeSteps)
		throw std::invalid_argument("The number of time steps is invalid.");

	
}	// moveAndSense


//std::pair<Positions, Positions> Robot::localize(double x0, double y0) const
template <class Localizer>
std::pair<Positions, Positions> Robot::localize(const Localizer &localizer) const
{
	if (this->measurements.empty())
		throw std::runtime_error("No measurement data.");

	if (this->measurements.size() != this->displacements.size())
		throw std::runtime_error("Measurement data is inconsistent with robot displacements.");

	return localizer.localize(this->measurements, this->displacements, this->measurementNoise, this->motionNoise);
}	// localize

// TODO: perhaps, rename it to spotSense()
void Robot::sense()
{
	this->measurements.emplace_back(this->senseLandmarks(this->world, this->sensorRange, this->measurementNoise));
	this->displacements.emplace_back(0, 0);
}	// sense 

void Robot::roamAndSense()
{
	//throw std::runtime_error("Not implemented");
	thread_local std::uniform_real_distribution<double> orientDist(0, 2 * std::acos(-1));
		
	double dx = 0, dy = 0;
	
	do	// try to move the robot until we succeed
	{
		//double orientation = orientDist(Robot::randomEngine);
		double orientation = orientDist(World::getRandomEngine());
	
		dx = this->stepSize * std::cos(orientation);
		dy = this->stepSize * std::sin(orientation);
	
		//distortMotion(dx, dy);
	
	} while (!this->move(this->world, dx, dy, this->motionNoise));
	
	//return Displacement(dx, dy);
	this->measurements.emplace_back(this->senseLandmarks(this->world, this->sensorRange, this->measurementNoise));
	this->displacements.emplace_back(dx, dy);
}	// roamAndSense



