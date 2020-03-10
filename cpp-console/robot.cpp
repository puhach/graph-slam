#include "robot.h"
#include "world.h"

#include <exception>

//Robot::Robot(int x, int y, World& world)
Robot::Robot(double x, double y, World& world)
	: x(x >= 0 && x < world.getWidth() ? x : throw std::invalid_argument("Robot X coordinate is out of the world."))
	, y(y >= 0 && y < world.getHeight() ? y : throw std::invalid_argument("Robot Y coordinate is out of the world."))
	, world(world)
{

}

std::pair<Measurements, Displacements> Robot::moveAndSense(int timesteps)
{
	if (timesteps < 1 || timesteps > MaxTimeSteps)
		throw std::invalid_argument("The number of time steps is invalid.");

	// There is an additional measurement and displacement for the initial position.
	Measurements measurements(timesteps+1);
	Displacements displacements(timesteps+1);

	// The initial position is represented as a displacement from the world origin (0; 0).
	// TODO: try not to use the initial position, because in practice it may be unknown.
	measurements[0] = std::move(this->sense());
	displacements[0] = Displacement(this->x, this->y);
	//double x0 = 0, y0 = 0;

	for (int t = 1; t <= timesteps; ++t)
	{
		measurements[t] = std::move(sense());		
		displacements[t] = this->wander();	// returns the displacement from the previous position
	}

	return { measurements, displacements };
}	// moveAndSense


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
		// TODO: try using something Gaussian kernel to distort farther landmark distances more
		//double factorX = 1 - std::exp(-dx * dx), factorY = 1 - std::exp(-dy * dy);
		//dx += rand()*measurementNoise*factorX
		//dy += rand()*measurementNoise*factorY
		distortMeasurement(dx, dy);
		
		if (dx * dx + dy * dy <= this->sensorRange)
			measurement.emplace_back(i, dx, dy);
		//if (() + ())
	}

	return measurement;
}	// sense