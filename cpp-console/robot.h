#pragma once

class World;

class Robot
{
public:
	Robot(int x, int y, World &world);

private:
	int x, y;
	World& world;
};	// Robot

