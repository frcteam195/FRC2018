//============================================================================
// Name        : PolynomialRegression.cpp
// Author      : Chris Bonomi
// Description : PolynomialRegression
//============================================================================

#include "PolynomialRegression.h"

PolynomialRegression::PolynomialRegression(vector<Point> points, int degree) {
	vector<double> x = vector<double>();
	vector<double> y = vector<double>();

	for(int i = 0; i < points.size(); i++) {
		x.push_back(points[i].x);
		y.push_back(points[i].y);
	}

	solve(x, y, degree);
}

PolynomialRegression::PolynomialRegression(vector<double> x, vector<double> y, int degree) {
	solve(x, y, degree);
}

MatrixXd PolynomialRegression::createMatrix(vector<vector<double> > xy) {
	MatrixXd matrix(xy.size(), xy[0].size());

	for(int i = 0; i < xy.size(); i++) {
		matrix.row(i) = VectorXd::Map(&xy[i][0], xy[0].size());
	}

	return matrix;
}

MatrixXd PolynomialRegression::createMatrix(vector<double> y, int n) {
	int cols = n != 0 ? y.size() / n : 0;

	MatrixXd matrix(n,  cols);

	for(int i = 0; i < n; i++) {
		matrix.row(i) = VectorXd::Map(&y[i], cols);
	}

	return matrix;
}

bool PolynomialRegression::isFullRank(CompleteOrthogonalDecomposition<MatrixXd> m) {
	return m.rank() == min(m.rows(), m.cols());
}

void PolynomialRegression::solve(vector<double> x, vector<double> y, int degree) {
	this->degree = degree;

	int n = x.size();
	CompleteOrthogonalDecomposition<MatrixXd> qr;
	MatrixXd matrixX;

	while(true) {
		vector<vector<double> > vandermonde = vector<vector<double> >();
		for(int i = 0; i < n; i++) {
			vandermonde.push_back(vector<double>());
			for(int j = 0; j <= this->degree; j++) {
				vandermonde[i].push_back(pow(x[i], j));
			}
		}

		matrixX = createMatrix(vandermonde);

		qr = CompleteOrthogonalDecomposition<MatrixXd>(matrixX);

		if(isFullRank(qr))
			break;

		this->degree--;
	}

	MatrixXd matrixY = createMatrix(y, n);

	beta = qr.solve(matrixY);

	double sum = 0.0;
	for(int i = 0; i < n; i++)
		sum += y[i];
	double mean = sum / n;

	for(int i = 0; i < n; i++) {
		double dev = y[i] - mean;
		sst += dev * dev;
	}


    MatrixXd residuals = matrixX * beta - matrixY;
    sse = residuals.norm() * residuals.norm();
}

double PolynomialRegression::getBeta(int j) {
	if(abs(beta.coeff(j, 0)) < 0.00001)
		return 0.0;
	return beta.coeff(j, 0);
}

int PolynomialRegression::getDegree() {
	return degree;
}

double PolynomialRegression::R2() {
	if(sst == 0.0)
		return 1.0;
	return 1.0 - sse / sst;
}

double PolynomialRegression::predict(double x) {
	double y = 0.0;
	for(int j = degree; j >= 0; j--)
		y = getBeta(j) + (x * y);
	return y;
}

string PolynomialRegression::toString() {
	stringstream s;
	s << fixed << setprecision(2);
	int j = degree;

	while(j >= 0 && abs(getBeta(j)) < 1E-5)
		j--;

	while(j >= 0) {
		if(j == 0)
			s << getBeta(j) << " ";
		else if(j == 1)
			s << getBeta(j) << " x + ";
		else
			s << getBeta(j) << " x^" << j << " + ";
		j--;
	}
	s << "  (R^2 = " << R2() << ")";

	return s.str();
}
