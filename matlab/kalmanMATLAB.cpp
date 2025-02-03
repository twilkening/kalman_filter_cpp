// NOTE: does not compile in MATLAB. abandoned code

#include "mex.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

using namespace Eigen;

void kalmanFilter(const MatrixXd& data, const MatrixXd& xinit, const MatrixXd& A, const MatrixXd& H,
                  const MatrixXd& Q, const MatrixXd& R, MatrixXd& P, MatrixXd& result) {
    MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
    Eigen::Index rows = data.rows();
    Eigen::Index cols = xinit.cols();
    result.resize(rows, cols);

    for (int i = 0; i < rows; i++) {
        if (i == 0) {
            result.row(i) = xinit;
        } else {
            MatrixXd x_pred = A * result.row(i - 1).transpose();
            MatrixXd P_pred = A * P * A.transpose() + Q;
            MatrixXd K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse();
            result.row(i) = (x_pred + K * (data.row(i).transpose() - H * x_pred)).transpose();
            P = (I - K * H) * P_pred;
        }
    }
}

// MEX gateway function
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    if (nrhs != 7) {
        mexErrMsgIdAndTxt("KalmanFilter:InvalidInput", "Seven inputs required: data, xinit, A, H, Q, R, P.");
    }
    if (nlhs != 1) {
        mexErrMsgIdAndTxt("KalmanFilter:InvalidOutput", "One output required.");
    }

    // Read inputs from MATLAB
    Map<MatrixXd> data(mxGetPr(prhs[0]), mxGetM(prhs[0]), mxGetN(prhs[0]));
    Map<MatrixXd> xinit(mxGetPr(prhs[1]), mxGetM(prhs[1]), mxGetN(prhs[1]));
    Map<MatrixXd> A(mxGetPr(prhs[2]), mxGetM(prhs[2]), mxGetN(prhs[2]));
    Map<MatrixXd> H(mxGetPr(prhs[3]), mxGetM(prhs[3]), mxGetN(prhs[3]));
    Map<MatrixXd> Q(mxGetPr(prhs[4]), mxGetM(prhs[4]), mxGetN(prhs[4]));
    Map<MatrixXd> R(mxGetPr(prhs[5]), mxGetM(prhs[5]), mxGetN(prhs[5]));
    Map<MatrixXd> P(mxGetPr(prhs[6]), mxGetM(prhs[6]), mxGetN(prhs[6]));

    // Allocate output matrix
    mwSize rows = mxGetM(prhs[0]);
    mwSize cols = mxGetN(prhs[1]);
    plhs[0] = mxCreateDoubleMatrix(rows, cols, mxREAL);
    Map<MatrixXd> result(mxGetPr(plhs[0]), rows, cols);

    // Perform Kalman filtering
    kalmanFilter(data, xinit, A, H, Q, R, P, result);
}

// /* 
//  * Kalman filter implementation for MATLAB compatibility
//  * This function takes input matrices for the state estimation and performs the Kalman filter algorithm.
//  * The readData and writeData functions have been removed for simplicity, as MATLAB has native support for these operations.
//  */
// 
// #include <Eigen/Dense>
// #include <Eigen/Core>
// #include <unsupported/Eigen/MatrixFunctions>
// 
// using namespace Eigen;
// 
// // Function to implement the Kalman Filter algorithm
// extern "C" MatrixXd kalmanFilter(MatrixXd data, MatrixXd xinit, MatrixXd A, MatrixXd H, MatrixXd Q, MatrixXd R, MatrixXd P) {
//     MatrixXd x(data.rows(), xinit.cols());
//     MatrixXd I = MatrixXd::Identity(A.rows(), A.cols());
// 
//     for (int i = 0; i < data.rows(); i++) {
//         if (i == 0) {
//             x.row(i) = xinit;
//         } else {
//             MatrixXd x_pred = A * x.row(i - 1).transpose();
//             MatrixXd P_pred = A * P * A.transpose() + Q;
//             MatrixXd K = P_pred * H.transpose() * (H * P_pred * H.transpose() + R).inverse();
//             x.row(i) = (x_pred + K * (data.row(i).transpose() - H * x_pred)).transpose();
//             P = (I - K * H) * P_pred;
//         }
//     }
// 
//     return x;
// }
