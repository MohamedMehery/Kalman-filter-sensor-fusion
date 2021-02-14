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

int main()
{

	return 0;
}
