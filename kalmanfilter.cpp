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
#include <Eigen/Dense>
#include <Eigen/Core>

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
MatrixXd kalmanFilter(MatrixXd data, MatrixXd A, MatrixXd B, MatrixXd H, MatrixXd Q, MatrixXd R, MatrixXd P) {
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
	MatrixXd A(1, 1);
	A << 1;
	MatrixXd B(1, 1);
	B << 0;
	MatrixXd H(1, 1);
	H << 1;
	MatrixXd Q(1, 1);
	Q << 0.1;
	MatrixXd R(1, 1);
	R << 0.1;
	MatrixXd P(1, 1);
	P << 0.1;
	MatrixXd filtered_data = kalmanFilter(data, A, B, H, Q, R, P);
	writeData(filtered_data, "filtered_data.csv");
	return 0;
}

