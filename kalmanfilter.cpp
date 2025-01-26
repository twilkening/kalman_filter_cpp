/*
this function is going to read data from a csv file and then filter it
based on the Kalman Filter algorithm. Lastly it will output it to a new csv
file.
*/ 

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <cmath>
#include <numbers>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

//#include "kalmanfilter.h"

using namespace std;
using namespace Eigen;

// Function to read data from a csv file
MatrixXd readData(string filename) {
    ifstream file(filename);
    vector<vector<double>> data;
    string line;
    while (getline(file, line)) {
        stringstream ss(line);
        vector<double> row;
        string cell;
        while (getline(ss, cell, ',')) {
            row.push_back(stod(cell));
        }
        data.push_back(row);
    }
    MatrixXd data_matrix(data.size(), data[0].size());
    for (int i = 0; i < data.size(); i++) {
        for (int j = 0; j < data[0].size(); j++) {
            data_matrix(i, j) = data[i][j];
        }
    }
    return data_matrix;
}

// Function to write data to a csv file
void writeData(MatrixXd data, string filename) {
	ofstream file(filename);
	for (int i = 0; i < data.rows(); i++) {
		for (int j = 0; j < data.cols(); j++) {
			file << data(i, j);
			if (j < data.cols() - 1) {
				file << ",";
			}
		}
		file << endl;
	}
}

// Function to implement the Kalman Filter algorithm
MatrixXd kalmanFilter(MatrixXd data, MatrixXd A, MatrixXd H, MatrixXd Q, MatrixXd R, MatrixXd P) {
	MatrixXd x(data.rows(), data.cols());
	MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
	for (int i = 0; i < data.rows(); i++) {
		if (i == 0) {
			x.row(i) = data.row(i);
		}
		else {
			MatrixXd x_pred = A * x.row(i - 1).transpose();
			MatrixXd P_pred = A * P * A.transpose() + Q;
			MatrixXd K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse();
			x.row(i) = (x_pred + K * (data.row(i) - H * x_pred)).transpose();
			P = (I - K * H) * P_pred;
		}
	}
	return x;
}

int main() {
	MatrixXd data = readData("data.csv");
	//To do: update the initialization matrices A, H, Q, R, P
	double g, l, m1, m2, a, qf, T, sigma;
	g = 9.81; // m/s^2
	l = 1; // m
	m1 = 1; // kg
	m2 = 2; // kg
	a = 3 * g * (m1 / 2 + m2) / (l * (m1 + 3 * m2));
	qf = 3 / (pow(l,2) * (m1 + 3 * m2));
	T = 0.01; // s
	constexpr double PI = 3.14159265358979323846;
    sigma = PI / 18.0 / 5.0;
	MatrixXd A(2, 2);
	A << 0, 1, a, 0;
	MatrixXd F(2, 2);
	F << A.exp();
	MatrixXd H(1, 2);
	H << 1, 0;
	MatrixXd Q(2, 2);
	Q << 0.000006121163394e-3, 0.000918045963364e-3, 0.000918045963364e-3, 0.183609197172119e-3;
	MatrixXd R(2, 2);
	R << sigma, 0, 0, sigma;
	MatrixXd P(2, 2);
	P << 0.5, 0, 0, 0.5;
	MatrixXd filtered_data = kalmanFilter(data, A, H, Q, R, P);
	writeData(filtered_data, "filtered_data.csv");
	return 0;
}

