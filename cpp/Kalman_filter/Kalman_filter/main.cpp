#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include "Eigen/Dense"
#include "measurement_package.h"
#include "tracking.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::ifstream;
using std::istringstream;
using std::string;
using std::vector;

MatrixXd CalculateJacobian(const VectorXd& x_state);
VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth);

int main() {

	/**
	 * Set Measurements
	 */
	vector<MeasurementPackage> measurement_pack_list;

	// hardcoded input file with laser and radar measurements
	string in_file_name_ = "obj_pose-laser-radar-synthetic-input.txt";
	ifstream in_file(in_file_name_.c_str(), ifstream::in);

	if (!in_file.is_open()) {
		cout << "Cannot open input file: " << in_file_name_ << endl;
	}

	string line;
	// set i to get only first 3 measurments
	int i = 0;
	while (getline(in_file, line) && (i <= 3)) {

		MeasurementPackage meas_package;

		istringstream iss(line);
		string sensor_type;
		iss >> sensor_type; // reads first element from the current line
		int64_t timestamp;
		if (sensor_type.compare("L") == 0) {  // laser measurement
		  // read measurements
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x, y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);

		}
		else if (sensor_type.compare("R") == 0) {
			// Skip Radar measurements
			continue;
		}
		++i;
	}

	// Create a Tracking instance
	Tracking tracking;

	// call the ProcessingMeasurement() function for each measurement
	size_t N = measurement_pack_list.size();
	// start filtering from the second frame 
	// (the speed is unknown in the first frame)
	for (size_t k = 0; k < N; ++k) {
		tracking.ProcessMeasurement(measurement_pack_list[k]);
	}

	if (in_file.is_open()) {
		in_file.close();
	}

	/**
	 * Compute the Jacobian Matrix
	 */

	 // predicted state example
	 // px = 1, py = 2, vx = 0.2, vy = 0.4
	VectorXd x_predicted(4);
	x_predicted << 1, 2, 0.2, 0.4;

	MatrixXd Hj = CalculateJacobian(x_predicted);

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
	cout << CalculateRMSE(estimations, ground_truth) << endl;

	return 0;
}

MatrixXd CalculateJacobian(const VectorXd& x_state) {

	MatrixXd Hj(3, 4);
	// recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	// pre-compute a set of terms to avoid repeated calculation
	float c1 = px * px + py * py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	// check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	// compute the Jacobian matrix
	Hj << (px / c2), (py / c2), 0, 0,
		-(py / c1), (px / c1), 0, 0,
		py*(vx*py - vy * px) / c3, px*(px*vy - py * vx) / c3, px / c2, py / c2;

	return Hj;
}


VectorXd CalculateRMSE(const vector<VectorXd> &estimations,
	const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	// accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		// coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	// calculate the mean
	rmse = rmse / estimations.size();

	// calculate the squared root
	rmse = rmse.array().sqrt();

	// return the result
	return rmse;
}