#include "Eigen/Dense"
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;


// Kalman Filter variables
VectorXd x;	// object init state
MatrixXd P;	// object covariance matrix
VectorXd u;	// external motion
MatrixXd F; // state transition matrix
MatrixXd H;	// measurement matrix
MatrixXd R;	// measurement covariance matrix
MatrixXd I; // Identity matrix
MatrixXd Q;	// process covariance matrix

vector <VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);
void test_eigen_lib(void);

int main3223()
{
	 // design the KF with 1D motion
	x = VectorXd(2);
	x << 0, 0;

	P = MatrixXd(2, 2);
	P << 1000, 0, 0, 1000;

	u = VectorXd(2);
	u << 0, 0;

	F = MatrixXd(2, 2);
	F << 1, 1, 0, 1;

	H = MatrixXd(1, 2);
	H << 1, 0;

	R = MatrixXd(1, 1);
	R << 1;

	I = MatrixXd::Identity(2, 2);

	Q = MatrixXd(2, 2);
	Q << 0, 0, 0, 0;

	// create a list of measurements
	VectorXd single_meas(1);
	single_meas << 2;
	measurements.push_back(single_meas);
	single_meas << 4;
	measurements.push_back(single_meas);
	single_meas << 16;
	measurements.push_back(single_meas);

	// call Kalman filter algorithm
	filter(x, P);

	return 0;
}

/*
* @Description: this function takes the initial state and initial uncertainty to apply KF
* @in_param: x and P
* @out_param: none
*/

void filter(VectorXd &x, MatrixXd &P)
{
	for (unsigned int n = 0; n < measurements.size(); ++n) {

		VectorXd z = measurements[n];
		// TODO: YOUR CODE HERE
		/**
		 * KF Measurement update step
		 */
		VectorXd y = z - H * x;
		MatrixXd Ht = H.transpose();
		MatrixXd S = H * P * Ht + R;
		MatrixXd Si = S.inverse();
		MatrixXd K = P * Ht * Si;

		// new state
		x = x + (K * y);
		P = (I - K * H) * P;
		cout << "New state \n";
		cout << "x=" << endl << x << endl;
		cout << "P=" << endl << P << endl;
		/**
		 * KF Prediction step
		 */
		x = F * x + u;
		MatrixXd Ft = F.transpose();
		P = F * P * Ft + Q;
		cout << "Prediction step\n";
		cout << "x=" << endl << x << endl;
		cout << "P=" << endl << P << endl;
	}
}

void test_eigen_lib()
{
	VectorXd my_vector(2);
	my_vector << 1, 2;
	cout << "my vector \n" << my_vector << endl;

	MatrixXd mymatrix(2, 2);
	mymatrix << 1, 2, 3, 4;
	mymatrix(1, 0) = 11;
	mymatrix(1, 1) = 12;
	cout << "mymatrix \n" << mymatrix << endl;

	/** matrix transpose */
	MatrixXd my_matrix_t = mymatrix.transpose();
	cout << "Matrix transpose \n" << my_matrix_t << endl;

	/** matrix inverse */
	MatrixXd my_matrix_inv = mymatrix.inverse();
	cout << "Matrix inverse \n" << my_matrix_inv << endl;

	/** matrix multiplication */
	MatrixXd another_mtx;
	another_mtx = mymatrix * my_vector;
	cout << "Matrix multiplication result \n" << another_mtx << endl;

}