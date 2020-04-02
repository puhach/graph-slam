#pragma once

#include "position.h"
#include "measurement.h"

#include <vector>
#include <utility>



class GraphSlam
{
public:
	GraphSlam(double x0, double y0, int nLandmarks)
		: x0(x0), y0(y0), nLandmarks(nLandmarks) {}

	std::pair<Positions, Positions> localize(const Measurements &measurements, const Displacements &displacements, double measurementNoise, double motionNoise) const;

private:
	//static void addConstraints(Eigen::MatrixXd &omega, Eigen::VectorXd &xi, int i, int j, double d, double noise);

	double x0, y0;
	int nLandmarks;
};	// GraphSlam