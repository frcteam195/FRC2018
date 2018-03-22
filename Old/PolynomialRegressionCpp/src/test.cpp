#include "PolynomialRegression.h"
#include <iostream>
#include <vector>
#include "Point.h"

#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

int main() {
	//double[] x = {0, 0.5, 1, 1.5, 2, 3, 4, 5.5, 7, 8, 8.5, 9};
	//double[] y = {0, 1, 2, 3, 4, 5, 6, 6.5, 7, 8, 9, 10};

	//With Point struct
	vector<Point> points = vector<Point>();
	points.push_back(Point(1, 1));
	points.push_back(Point(2, 4));
	points.push_back(Point(3, 9));

	//Quadratic
	/*vector<double> x = vector<double>();
	x.push_back(1);
	x.push_back(2);
	x.push_back(3);

	vector<double> y = vector<double>();
	y.push_back(1);
	y.push_back(4);
	y.push_back(9);*/

	//Right then left
	/*vector<double> x = vector<double>();
	x.push_back(0);
	x.push_back(0.5);
	x.push_back(1);
	x.push_back(1.5);
	x.push_back(2);
	x.push_back(3);
	x.push_back(4);
	x.push_back(5.5);
	x.push_back(7);
	x.push_back(8);
	x.push_back(8.5);
	x.push_back(9);

	vector<double> y = vector<double>();
	y.push_back(0);
	y.push_back(1);
	y.push_back(2);
	y.push_back(3);
	y.push_back(4);
	y.push_back(5);
	y.push_back(6);
	y.push_back(6.5);
	y.push_back(7);
	y.push_back(8);
	y.push_back(9);
	y.push_back(10);*/

	//PolynomialRegression p = PolynomialRegression(x, y, 3);
	PolynomialRegression p = PolynomialRegression(points, 2);

	cout << p.predict(4) << endl;
	cout << p.predict(6) << endl;
	cout << p.predict(7) << endl;
	cout << p.toString() << endl;

	return 0;
}
