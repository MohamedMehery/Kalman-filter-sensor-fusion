#include "Eigen/Dense"
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

VectorXd x;
MatrixXd P;
VectorXd u;
MatrixXd F;
MatrixXd H;
MatrixXd R;
MatrixXd I;
MatrixXd Q;

vector <VectorXd> measurements;
void filter(VectorXd &x, MatrixXd &P);
void test_eigen_lib(void);

int main()
{



	return 0;
}

void filter(VectorXd &x, MatrixXd &P)
{


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

}