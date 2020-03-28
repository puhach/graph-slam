#pragma once

#include <vector>
#include <utility>

class GraphSlam
{
public:
	GraphSlam(double x0, double y0): x0(x0), y0(y0) {}

	//std::pair<Positions, Positions> solve(double x0, double y0) const;

private:
	double x0, y0;
};	// GraphSlam