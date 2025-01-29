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
        // Ignore comments after double backslashes
        size_t comment_pos = line.find("//");
        if (comment_pos != string::npos) {
            line = line.substr(0, comment_pos);
        }
        stringstream ss(line);
        vector<double> row;
        string cell;
        while (getline(ss, cell, ',')) {
            if (!cell.empty()) {
                row.push_back(stod(cell));
            }
        }
        if (!row.empty()) {
            data.push_back(row);
        }
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
MatrixXd kalmanFilter(MatrixXd data, MatrixXd xinit, MatrixXd A, MatrixXd H, MatrixXd Q, MatrixXd R, MatrixXd P) {
	MatrixXd x(data.rows(), xinit.cols());
	MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
	for (int i = 0; i < data.rows(); i++) {
		if (i == 0) {
			x.row(i) = xinit;
		}
		else {
			//cout << "A: " << A << endl;
			//cout << "x.row(i - 1).transpose(): " << x.row(i - 1).transpose() << endl;
			MatrixXd x_pred = A * x.row(i - 1).transpose();
			//cout << "x_pred: " << x_pred << endl;
			MatrixXd P_pred = A * P * A.transpose() + Q;
			//cout << "P_pred: " << P_pred << endl;
			MatrixXd K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse();
			//cout << "K: " << K << endl;
			x.row(i) = (x_pred + K * (data.row(i).col(0) - H * x_pred)).transpose();
			//cout << "x.row(i): " << x.row(i) << endl;
			P = (I - K * H) * P_pred;
			//cout << "P: " << P << endl;
		}
	}
	return x;
}

int main() {
	MatrixXd data = readData("data/theta_dtheta_rand.csv");
	MatrixXd xinit = readData("data/xinit.csv");
	MatrixXd A = readData("data/A.csv");
	MatrixXd H = readData("data/H.csv");
	MatrixXd Q = readData("data/Q.csv");
	MatrixXd R = readData("data/R.csv");
	MatrixXd P = readData("data/P.csv");
	MatrixXd filtered_data = kalmanFilter(data, xinit, A, H, Q, R, P);
	writeData(filtered_data, "data/filtered_data.csv");
	return 0;
}

