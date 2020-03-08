#include "robot.h"
#include "world.h"

#include <exception>

Robot::Robot(int x, int y, World& world)
	: x(x >= 0 && x < world.getWidth() ? x : throw std::invalid_argument("Robot X coordinate is out of the world."))
	, y(y >= 0 && y < world.getHeight() ? y : throw std::invalid_argument("Robot Y coordinate is out of the world."))
	, world(world)
{

}

std::pair<Measurements, Positions> Robot::moveAndSense(int timesteps)
{
	if (timesteps < 1 || timesteps > MaxTimeSteps)
		throw std::invalid_argument("The number of time steps is invalid.");

	Measurements measurements(timesteps);
	Positions positions(timesteps);

	for (int t = 0; t < timesteps; ++t)
	{
		measurements[t] = std::move(sense());		
		positions[t] = Position(this->x, this->y);
		this->wander();
	}

	return { measurements, positions};
}
