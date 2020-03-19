#include "robot.h"
#include "world.h"

#include <exception>
#include <cmath>
//#include <numbers>
#include <Eigen/Dense>

//thread_local std::mt19937 Robot::randomEngine(std::random_device{}());


//Robot::Robot(int x, int y, World& world)
Robot::Robot(double x, double y, double sensorRange, double stepSize, double measurementNoise, double motionNoise, World& world)
	: x(x >= 0 && x < world.getWidth() ? x : throw std::invalid_argument("Robot's X coordinate is out of the world."))
	, y(y >= 0 && y < world.getHeight() ? y : throw std::invalid_argument("Robot's Y coordinate is out of the world."))
	, sensorRange(sensorRange > 0 ? sensorRange : throw std::invalid_argument("Robot's sensor range is invalid."))
	, stepSize(stepSize > 0 && stepSize < world.getHeight() && stepSize < world.getWidth() ? stepSize : throw std::invalid_argument("Robot's step size is invalid."))
	, measurementNoise(measurementNoise > 0 && measurementNoise < sensorRange ? measurementNoise : throw std::invalid_argument("Robot's measurement noise must be in range (0, sensorRange)."))
	, motionNoise(motionNoise > 0 && motionNoise < stepSize ? motionNoise : throw std::invalid_argument("Robot's motion noise must be in range (0, stepSize)."))
	, world(world)
{

}

//std::pair<Measurements, Displacements> Robot::moveAndSense(int timesteps)
void Robot::moveAndSense(int timesteps)
{
	if (timesteps < 1 || timesteps > MaxTimeSteps)
		throw std::invalid_argument("The number of time steps is invalid.");

	// There is an additional measurement and displacement for the initial position.
	this->measurements.resize(static_cast<std::size_t>(timesteps) + 1);
	this->displacements.resize(static_cast<std::size_t>(timesteps) + 1);
	//Measurements measurements(timesteps+1);
	//Displacements displacements(timesteps+1);

	// The initial position is represented as a displacement from the world origin (0; 0).
	// TODO: try not to use the initial position, because in practice it may be unknown.
	// Probably, we should not even have (x, y) as Robot's members, because, strictly speaking,
	// the robot doesn't know it's real position.
	this->measurements[0] = std::move(this->sense());
	this->displacements[0] = Displacement(this->x, this->y);
	//double x0 = 0, y0 = 0;

	for (int t = 1; t <= timesteps; ++t)
	{
		this->measurements[t] = std::move(this->sense());		
		this->displacements[t] = this->wander();	// returns the displacement from the previous position
	}

	//return { measurements, displacements };
}	// moveAndSense


std::pair<Positions, Positions> Robot::localize() const
{
	if (this->measurements.empty())
		throw std::runtime_error("No measurement data.");

	if (this->measurements.size() != this->displacements.size())
		throw std::runtime_error("Measurement data is inconsistent with robot displacements.");

	// Initialize the constraints.

	const std::size_t size = this->measurements.size() + this->world.getLandmarkNum();
	
	Eigen::MatrixXd omegaX(size, size), omegaY(size, size);
	omegaX.fill(0);
	omegaY.fill(0);

	Eigen::VectorXd xiX(size), xiY(size);
	xiX.fill(0);
	xiY.fill(0);

	// Here we assume that x0, y0 is the first displacement.
	// TODO: try not to rely on the initial position as it may not be known.
	omegaX(0, 0) = 1;
	xiX(0) = this->displacements[0].first;

	omegaY(0, 0) = 1;
	xiY(0) = this->displacements[0].second;


	// Set measurement constraints.

	void addConstraints(Eigen::MatrixXd & omega, Eigen::VectorXd & xi, int i, int j, double d, double noise);

	const int m = static_cast<int>(this->measurements.size());

	for (int t = 0; t < m; ++t)
	{
		for (const auto & [lk, dx, dy] : this->measurements[t])
		{

			// dx = Lx[lk] - x[t]
			addConstraints(omegaX, xiX, t, m + lk, -dx, this->measurementNoise);	// x[t] - Lx[lk] = -dx
			addConstraints(omegaX, xiX, m + lk, t, dx, this->measurementNoise);		// Lx[lk] - x[t] = dx

			// dy = Ly[lk] - y[t]
			addConstraints(omegaY, xiY, t, m + lk, -dy, this->measurementNoise);	// y[t] - Ly[lk] = -dy
			addConstraints(omegaY, xiY, m + lk, t, dy, this->measurementNoise);		// Ly[lk] - y[t] = dy
		}
	}	// t

	// Set motion constraints

	//Positions 
	return std::pair<Positions, Positions>();
}	// localize

Measurement Robot::sense() const
{
	Measurement measurement;
	measurement.reserve(this->world.getLandmarkNum());

	for (int i = 0; i < this->world.getLandmarkNum(); ++i)
	{		
		auto [lkx, lky] = world.getLandmark(i);

		// Get the distance to the landmark.
		double dx = lkx - this->x;
		double dy = lky - this->y;

		// Distort the measurement.		
		distortMeasurement(dx, dy);
		
		if (dx * dx + dy * dy <= this->sensorRange*this->sensorRange)
			measurement.emplace_back(i, dx, dy);
		//if (() + ())
	}

	return measurement;
}	// sense


Displacement Robot::wander()
{
	// As long as we are not using threads, static can be used instead of thread_local.
	// The Standard Library doesn't define pi, hence std::acos(-1) is used to calculate it.
	thread_local std::uniform_real_distribution<double> orientDist(0, 2*std::acos(-1));
	
	double dx = 0, dy = 0;

	do	// try to move the robot until we succeed
	{
		//double orientation = orientDist(Robot::randomEngine);
		double orientation = orientDist(World::getRandomEngine());

		dx = this->stepSize * std::cos(orientation);
		dy = this->stepSize * std::sin(orientation);

		distortMotion(dx, dy);

	} while (!this->move(dx, dy));

	return Displacement(dx, dy);
}	// wander


bool Robot::move(double dx, double dy)
{
	double newX = this->x + dx, newY = this->y + dy;

	if (newX < 0 || newX >= this->world.getWidth() || newY < 0 || newY >= this->world.getHeight())
		return false;

	this->x = newX;
	this->y = newY;

	return true;
}	// move


void Robot::distortMotion(double& dx, double& dy) const
{
	// TODO: remove thread_local because this->motionNoise may be changed
	thread_local std::uniform_real_distribution<double> motionDist(-this->motionNoise, this->motionNoise);

	dx += motionDist(World::getRandomEngine());
	dy += motionDist(World::getRandomEngine());
}	// distortMotion

void Robot::distortMeasurement(double& dx, double& dy)	const
{
	// TODO: remove thread_local because this->measurementNoise may be changed
	thread_local std::uniform_real_distribution<double> measurementDist(-this->measurementNoise, this->measurementNoise);

	// TODO: try using something Gaussian kernel to distort farther landmark distances more
	//double factorX = 1 - std::exp(-dx * dx), factorY = 1 - std::exp(-dy * dy);
	//dx += rand()*measurementNoise*factorX
	//dy += rand()*measurementNoise*factorY
	dx += measurementDist(World::getRandomEngine());
	dy += measurementDist(World::getRandomEngine());
}	// distortMeasurement



void addConstraints(Eigen::MatrixXd& omega, Eigen::VectorXd& xi, int i, int j, double d, double noise)
{
	omega(i, j) += 1.0 / noise;
	omega(j, i) -= 1.0 / noise;
	xi(i) += d / noise;
}