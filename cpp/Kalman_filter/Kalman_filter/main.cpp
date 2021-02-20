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

	meas_package.raw_measurements_ << 8.44818, 0.251553;
	meas_package.timestamp_ = 1477010443449633;
	fusionEKF.ProcessMeasurement(meas_package);

	meas_package.raw_measurements_ << 8.44818, 0.251553;
	meas_package.timestamp_ = 1477010443449633;
	fusionEKF.ProcessMeasurement(meas_package);

	meas_package.raw_measurements_ << 8.45582, 0.253997;
	meas_package.timestamp_ = 1477010443549747;
	fusionEKF.ProcessMeasurement(meas_package); 
	

	return 0;
}

