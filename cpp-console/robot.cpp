#include "robot.h"
#include "world.h"

Robot::Robot(int x, int y, World& world)
	: x(x >= 0 && x < world.getWidth() ? x : throw std::invalid_argument("Robot X coordinate is out of the world."))
	, y(y >= 0 && y < world.getHeight() ? y : throw std::invalid_argument("Robot Y coordinate is out of the world."))
	, world(world)
{

}