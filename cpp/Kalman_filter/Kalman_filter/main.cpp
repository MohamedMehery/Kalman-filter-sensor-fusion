#include <iostream>
#include "Eigen/Dense"
#include "ukf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


int main() {

	// Create a UKF instance
	UKF ukf;

	/**
	 * Programming assignment calls
	 */
	MatrixXd Xsig = MatrixXd(5, 11);
	ukf.GenerateSigmaPoints(&Xsig);

	/**
	 * Programming assignment calls
	 */
	MatrixXd Xsig_aug = MatrixXd(7, 15);
	ukf.AugmentedSigmaPoints(&Xsig_aug);

	MatrixXd Xsig_pred = MatrixXd(15, 5);
	ukf.SigmaPointPrediction(&Xsig_pred);
	
	VectorXd x_pred = VectorXd(5);
	MatrixXd P_pred = MatrixXd(5, 5);
	ukf.PredictMeanAndCovariance(&x_pred, &P_pred);

	return 0;
}