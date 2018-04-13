#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using namespace std;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	// Check the validity of the following inputs:
	// The estimation vector size should not be zero
	float t = estimations.size(); // Current timestep index

								  // check inputs
	if (t == 0)
		cout << "Error in CalculateRMSE:  estimations.size() = 0" << endl;
	if (t != ground_truth.size())
		cout << "Error in CalculateRMSE: sizes of estimation and ground truth do not match" << endl;


	// Accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {
		VectorXd residual = estimations[i] - ground_truth[i];
		// Coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// Calculate the mean
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();

	// cout << estimations.size() << endl;
	if (rmse(0) > .09 ||
		rmse(1) > .10 ||
		rmse(2) > .40 ||
		rmse(3) > .30)
		cout << "Warning at timestep " << t << endl;

	if (rmse(0) > .09)
		cout << "rmse(0) exceeds tolerance of .09" << endl;
	if (rmse(1) > .10)
		cout << "rmse(1) exceeds tolerance of .10" << endl;
	if (rmse(2) > .40)
		cout << "rmse(2) exceeds tolerance of .40" << endl;
	if (rmse(3) > .30)
		cout << "rmse(3) exceeds tolerance of .30" << endl;

	return rmse;
}