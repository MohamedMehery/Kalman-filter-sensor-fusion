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

int main()
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
	single_meas << 1;
	measurements.push_back(single_meas);
	single_meas << 2;
	measurements.push_back(single_meas);
	single_meas << 3;
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
		MatrixXd y = z - (H * x);
		MatrixXd s = H * P * H.transpose() + R;
		MatrixXd k = P * H.transpose() * s.inverse();

		// KF Measurement update step
		x = x + (k * y);
		P = (I - (k *H)) * P;
		// new state
		cout << "measurement x=" << endl << x << endl << endl;
		cout << "measurement P=" << endl << P << endl << endl;
		// KF Prediction step
		x = (F * x) + u;
		P = F * P*F.transpose();
		cout << "Prediction x=" << endl << x << endl << endl;
		cout << "Prediction P=" << endl << P << endl << endl; 
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