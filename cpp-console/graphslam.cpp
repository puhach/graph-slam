#include "graphslam.h"

#include <Eigen/Dense>

std::pair<Positions, Positions> GraphSlam::localize(const Measurements &measurements, 
	const Displacements &displacements, double measurementNoise, double motionNoise) const
{
	// Initialize the constraints.

	const std::size_t size = measurements.size() + this->nLandmarks;

	Eigen::MatrixXd omegaX(size, size), omegaY(size, size);
	omegaX.fill(0);
	omegaY.fill(0);

	Eigen::VectorXd xiX(size), xiY(size);
	xiX.fill(0);
	xiY.fill(0);

	// Here we assume that x0, y0 is the first displacement.
	// TODO: try not to rely on the initial position as it may not be known.
	omegaX(0, 0) = 1;
	xiX(0) = x0; //this->displacements[0].first;

	omegaY(0, 0) = 1;
	xiY(0) = y0; //this->displacements[0].second;



	// Set measurement constraints.

	void addConstraints(Eigen::MatrixXd & omega, Eigen::VectorXd & xi, int i, int j, double d, double noise);

	const int m = static_cast<int>(measurements.size());

	for (int t = 0; t < m; ++t)
	{
		for (const auto& [lk, dx, dy] : measurements[t])
		{

			// dx = Lx[lk] - x[t]

			addConstraints(omegaX, xiX, t, m + lk, -dx, measurementNoise);	// x[t] - Lx[lk] = -dx
			addConstraints(omegaX, xiX, m + lk, t, dx, measurementNoise);		// Lx[lk] - x[t] = dx

			// dy = Ly[lk] - y[t]

			addConstraints(omegaY, xiY, t, m + lk, -dy, measurementNoise);	// y[t] - Ly[lk] = -dy
			addConstraints(omegaY, xiY, m + lk, t, dy, measurementNoise);		// Ly[lk] - y[t] = dy
		}
	}	// t


	// Set motion constraints.

	for (int t = 1; t < m; ++t)
	{
		// dx = x[t] - x[t-1]

		addConstraints(omegaX, xiX, t, t - 1, displacements[t].first, motionNoise);	// x[t] - x[t-1] = dx
		addConstraints(omegaX, xiX, t - 1, t, -displacements[t].first, motionNoise);	// x[t-1] - x[t] = -dx

		// dy = y[t] - y[t-1]

		addConstraints(omegaY, xiY, t, t - 1, displacements[t].second, motionNoise);	// y[t] - y[t-1] = dy
		addConstraints(omegaY, xiY, t - 1, t, -displacements[t].second, motionNoise);	// y[t-1] - y[t] = -dy
	}	// t


	// Compute the best estimate for the robot and landmarks positions.

	Eigen::VectorXd muX = omegaX.colPivHouseholderQr().solve(xiX);
	Eigen::VectorXd muY = omegaY.colPivHouseholderQr().solve(xiY);
	//Eigen::VectorXd muX = omegaX.fullPivHouseholderQr().solve(xiX);
	//Eigen::VectorXd muY = omegaY.fullPivHouseholderQr().solve(xiY);
	//Eigen::VectorXd muX = omegaX.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(xiX);
	//Eigen::VectorXd muY = omegaY.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(xiY);
	//Eigen::VectorXd muX = omegaX.inverse() * xiX;
	//Eigen::VectorXd muY = omegaY.inverse() * xiY;
	//std::cout << muX.rows() << std::endl;
	//std::cout << muX << std::endl << muY << std::endl;


	//std::cout << "xiX:" << std::endl;
	//std::cout << xiX << std::endl;
	//double relativeErrorX = (omegaX * muX - xiX).norm() / xiX.norm()
	//	,  relativeErrorY = (omegaY * muY - xiY).norm() / xiY.norm();

	//std::cout << "Relative error: " << relativeErrorX << " " << relativeErrorY << std::endl;
	//std::cout << omegaX * muX << std::endl;

	Positions robotPositions(measurements.size());
	for (int t = 0; t < m; ++t)
	{
		robotPositions[t] = Position(muX(t), muY(t));
	}

	Positions landmarkLocations(nLandmarks);
	for (int lk = 0; lk < this->nLandmarks; ++lk)
	{
		landmarkLocations[lk] = Position(muX(m + lk), muY(m + lk));
	}

	return std::pair<Positions, Positions>(robotPositions, landmarkLocations);
}	// localize

void addConstraints(Eigen::MatrixXd& omega, Eigen::VectorXd& xi, int i, int j, double d, double noise)
{
	// TODO: perhaps, it's better to use exp(-noise)
	omega(i, i) += 1.0 / noise;
	omega(i, j) -= 1.0 / noise;
	xi(i) += d / noise;
	/*omega(i, i) += 1.0;
	omega(i, j) -= 1.0;
	xi(i) += d;*/
}