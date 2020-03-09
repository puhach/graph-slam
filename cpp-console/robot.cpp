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
}
