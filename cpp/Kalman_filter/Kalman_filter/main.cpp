#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tracking.h"
#include "tools.h"
#include "FusionEKF.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;


int main() {

	/**
	 * Set Measurements
	 */
	MeasurementPackage meas_package;
	Tools tools;
	FusionEKF fusionEKF;

	meas_package.sensor_type_ = MeasurementPackage::DVL;
	meas_package.raw_measurements_ = VectorXd(2);
	meas_package.raw_measurements_ << 8.44818	,0.251553;
	meas_package.timestamp_ = 1477010443449633;
	fusionEKF.ProcessMeasurement(meas_package);

	meas_package.raw_measurements_ << 8.45582,	0.253997	;
	meas_package.timestamp_ = 1477010443549747;
	fusionEKF.ProcessMeasurement(meas_package);

	/**
	 * Compute the Jacobian Matrix
	 */

	 // predicted state example
	 // px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = tools.CalculateJacobian(x_predicted);

	cout << "Hj:" << endl << Hj << endl;


	/**
   * Compute RMSE
   */
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	// the input list of estimations
	VectorXd e(4);
	e << 1, 1, 0.2, 0.1;
	estimations.push_back(e);
	e << 2, 2, 0.3, 0.2;
	estimations.push_back(e);
	e << 3, 3, 0.4, 0.3;
	estimations.push_back(e);

	// the corresponding list of ground truth values
	VectorXd g(4);
	g << 1.1, 1.1, 0.3, 0.2;
	ground_truth.push_back(g);
	g << 2.1, 2.1, 0.4, 0.3;
	ground_truth.push_back(g);
	g << 3.1, 3.1, 0.5, 0.4;
	ground_truth.push_back(g);

	// call the CalculateRMSE and print out the result
	cout << tools.CalculateRMSE(estimations, ground_truth) << endl;

	return 0;
}

