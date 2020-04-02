#include "robot.h"
#include "world.h"
#include "graphslam.h"

#include <exception>
#include <cmath>
//#include <numbers>


template std::pair<Positions, Positions>  Robot::localize<GraphSlam>(const GraphSlam &method) const;

//thread_local std::mt19937 Robot::randomEngine(std::random_device{}());


///Robot::Robot(int x, int y, World& world)
//Robot::Robot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise, World& world)
//	: x(x >= 0 && x < world.getWidth() ? x : throw std::invalid_argument("Robot's X coordinate is out of the world."))
//	, y(y >= 0 && y < world.getHeight() ? y : throw std::invalid_argument("Robot's Y coordinate is out of the world."))
//	, sensorRange(sensorRange > 0 ? sensorRange : throw std::invalid_argument("Robot's sensor range is invalid."))
//	, stepSize(stepSize > 0 && stepSize < world.getHeight() && stepSize < world.getWidth() ? stepSize : throw std::invalid_argument("Robot's step size is invalid."))
//	, measurementNoise(measurementNoise > 0 && measurementNoise < sensorRange ? measurementNoise : throw std::invalid_argument("Robot's measurement noise must be in range (0, sensorRange)."))
//	, motionNoise(motionNoise > 0 && motionNoise < stepSize ? motionNoise : throw std::invalid_argument("Robot's motion noise must be in range (0, stepSize)."))
//	, world(world)
//{
//
//}

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


//std::pair<Measurements, Displacements> Robot::moveAndSense(int timesteps)
void Robot::moveAndSense(int timesteps)
{
	if (timesteps < 1 || timesteps > MaxTimeSteps)
		throw std::invalid_argument("The number of time steps is invalid.");

	//// There is an additional measurement and displacement for the initial position.
	//this->measurements.resize(static_cast<std::size_t>(timesteps) + 1);
	//this->displacements.resize(static_cast<std::size_t>(timesteps) + 1);
	////Measurements measurements(timesteps+1);
	////Displacements displacements(timesteps+1);

	//// The initial position is represented as a displacement from the world origin (0; 0).
	//// TODO: try not to use the initial position, because in practice it may be unknown.
	//// Probably, we should not even have (x, y) as Robot's members, because, strictly speaking,
	//// the robot doesn't know it's real position.	
	//this->displacements[0] = Displacement(this->x, this->y);
	////double x0 = 0, y0 = 0;
	//this->measurements[0] = std::move(this->sense());

	//for (int t = 1; t <= timesteps; ++t)
	//{		
	//	this->displacements[t] = this->wander();	// returns the displacement from the previous position
	//	this->measurements[t] = std::move(this->sense());
	//}

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

//Measurement Robot::sense() const
//{
//	Measurement measurement;
//	measurement.reserve(this->world.getLandmarkNum());
//
//	for (int i = 0; i < this->world.getLandmarkNum(); ++i)
//	{		
//		auto [lkx, lky] = world.getLandmark(i);
//
//		// Get the distance to the landmark.
//		double dx = lkx - this->x;
//		double dy = lky - this->y;
//
//		// Distort the measurement.		
//		distortMeasurement(dx, dy);
//		
//		if (dx * dx + dy * dy <= this->sensorRange*this->sensorRange)
//			measurement.emplace_back(i, dx, dy);
//		//if (() + ())
//	}
//
//	return measurement;
//}	// sense


//Displacement Robot::wander()
//{
//	// As long as we are not using threads, static can be used instead of thread_local.
//	// The Standard Library doesn't define pi, hence std::acos(-1) is used to calculate it.
//	thread_local std::uniform_real_distribution<double> orientDist(0, 2*std::acos(-1));
//	
//	double dx = 0, dy = 0;
//
//	do	// try to move the robot until we succeed
//	{
//		//double orientation = orientDist(Robot::randomEngine);
//		double orientation = orientDist(World::getRandomEngine());
//
//		dx = this->stepSize * std::cos(orientation);
//		dy = this->stepSize * std::sin(orientation);
//
//		distortMotion(dx, dy);
//
//	} while (!this->move(dx, dy));
//
//	return Displacement(dx, dy);
//}	// wander
//
//
//bool Robot::move(double dx, double dy)
//{
//	double newX = this->x + dx, newY = this->y + dy;
//
//	if (newX < 0 || newX >= this->world.getWidth() || newY < 0 || newY >= this->world.getHeight())
//		return false;
//
//	this->x = newX;
//	this->y = newY;
//
//	return true;
//}	// move


//void Robot::distortMotion(double& dx, double& dy) const
//{
//	// TODO: remove thread_local because this->motionNoise may be changed
//	thread_local std::uniform_real_distribution<double> motionDist(-this->motionNoise, this->motionNoise);
//
//	dx += motionDist(World::getRandomEngine());
//	dy += motionDist(World::getRandomEngine());
//}	// distortMotion
//
//void Robot::distortMeasurement(double& dx, double& dy)	const
//{
//	// TODO: remove thread_local because this->measurementNoise may be changed
//	thread_local std::uniform_real_distribution<double> measurementDist(-this->measurementNoise, this->measurementNoise);
//
//	// TODO: try using something Gaussian kernel to distort farther landmark distances more
//	//double factorX = 1 - std::exp(-dx * dx), factorY = 1 - std::exp(-dy * dy);
//	//dx += rand()*measurementNoise*factorX
//	//dy += rand()*measurementNoise*factorY
//	dx += measurementDist(World::getRandomEngine());
//	dy += measurementDist(World::getRandomEngine());
//}	// distortMeasurement
//


