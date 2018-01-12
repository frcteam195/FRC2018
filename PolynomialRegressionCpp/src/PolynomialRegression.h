/*
 * PolynomialRegression.h
 *
 *  Created on: Jan 11, 2018
 *      Author: chris bonomi
 */

#ifndef POLYNOMIALREGRESSION_H_
#define POLYNOMIALREGRESSION_H_

#include "Eigen/Dense"
#include <vector>
#include <sstream>
#include <iomanip>
#include "Point.h"

using namespace Eigen;
using namespace std;

class PolynomialRegression {
private:
	int degree;
	MatrixXd beta;
	double sse;
	double sst;

	MatrixXd createMatrix(vector<vector<double> > xy);
	MatrixXd createMatrix(vector<double> y, int n);

	void solve(vector<double> x, vector<double> y, int degree);
public:
	PolynomialRegression(vector<Point> points, int degree);
	PolynomialRegression(vector<double> x, vector<double> y, int degree);

	double getBeta(int j);
	int getDegree();
	double R2();
	double predict(double x);
	string toString();
};

#endif /* POLYNOMIALREGRESSION_H_ */
