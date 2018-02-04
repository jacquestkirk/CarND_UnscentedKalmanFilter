#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */

	VectorXd rmse = VectorXd(4);
	rmse << 0, 0, 0, 0;

	if (estimations.size() != ground_truth.size())
	{
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}
	if (estimations.size() == 0)
	{
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}



	//accumulate squared residuals

	VectorXd sum_residuals_squared(4);
	sum_residuals_squared << 0, 0, 0, 0;

	for (int i = 0; i < estimations.size(); ++i) {
		// ... your code here

		VectorXd diff = estimations[i] - ground_truth[i];


		VectorXd residuals_squared = diff.array() * diff.array();

		sum_residuals_squared += residuals_squared;

	}

	//calculate the mean
	// ... your code here
	VectorXd mean = sum_residuals_squared / estimations.size();

	//calculate the squared root
	// ... your code here
	rmse = mean.array().sqrt();

	//return the result
	return rmse;
}
