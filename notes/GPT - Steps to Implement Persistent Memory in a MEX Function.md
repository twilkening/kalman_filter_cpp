To allow your Kalman filter C++ function to retain its state across multiple calls from MATLAB for "on-line" testing, you can use **persistent state** within the MEX function. This involves storing the Kalman filter’s internal state (e.g., estimates, error covariances, etc.) in a static or global variable that persists across invocations of the MEX file.

Here’s how you can achieve this:

------

### Steps to Implement Persistent Memory in a MEX Function

#### 1. **Define a Persistent State**

Use a `static` variable within the MEX function to store the Kalman filter's state. This memory will persist across multiple calls to the function.

------

#### 2. **Initialize the State on the First Call**

Check whether the state has been initialized. If not, allocate and initialize the state on the first call.

------

#### 3. **Update and Use the State**

During subsequent calls, retrieve and update the stored state.

------

### Example: Kalman Filter with Persistent State

```cpp
#include "mex.h"
#include <iostream>
#include <vector>

class KalmanFilter {
public:
    KalmanFilter(double process_variance, double measurement_variance)
        : estimate(0.0),
          error_covariance(1.0),
          process_variance(process_variance),
          measurement_variance(measurement_variance) {}

    double update(double measurement) {
        // Prediction step
        error_covariance += process_variance;

        // Update step
        double kalman_gain = error_covariance / (error_covariance + measurement_variance);
        estimate += kalman_gain * (measurement - estimate);
        error_covariance *= (1 - kalman_gain);

        return estimate;
    }

private:
    double estimate;
    double error_covariance;
    double process_variance;
    double measurement_variance;
};

// Global pointer to the Kalman filter object
static KalmanFilter* kalman_filter = nullptr;

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    // Command input to initialize or reset the filter
    if (nrhs > 0 && mxIsChar(prhs[0])) {
        char command[10];
        mxGetString(prhs[0], command, sizeof(command));

        if (strcmp(command, "init") == 0) {
            // Initialize the Kalman filter with given parameters
            if (nrhs != 3) {
                mexErrMsgIdAndTxt("KalmanFilter:InvalidInput", "Usage: kalman_filter('init', process_variance, measurement_variance)");
            }
            double process_variance = mxGetScalar(prhs[1]);
            double measurement_variance = mxGetScalar(prhs[2]);

            // Free previous instance, if any
            if (kalman_filter) {
                delete kalman_filter;
            }

            // Create a new Kalman filter
            kalman_filter = new KalmanFilter(process_variance, measurement_variance);
            mexPrintf("Kalman filter initialized.\n");
            return;
        } else if (strcmp(command, "reset") == 0) {
            // Reset the Kalman filter state
            delete kalman_filter;
            kalman_filter = nullptr;
            mexPrintf("Kalman filter reset.\n");
            return;
        } else {
            mexErrMsgIdAndTxt("KalmanFilter:InvalidCommand", "Unknown command. Use 'init' or 'reset'.");
        }
    }

    // Ensure the Kalman filter is initialized
    if (!kalman_filter) {
        mexErrMsgIdAndTxt("KalmanFilter:NotInitialized", "Kalman filter not initialized. Use 'init' command first.");
    }

    // Apply the Kalman filter to the input data
    if (nrhs != 1 || !mxIsDouble(prhs[0])) {
        mexErrMsgIdAndTxt("KalmanFilter:InvalidInput", "Input must be a double scalar.");
    }

    double measurement = mxGetScalar(prhs[0]);
    double filtered_value = kalman_filter->update(measurement);

    // Return the filtered value
    plhs[0] = mxCreateDoubleScalar(filtered_value);
}
```

------

### Explanation of the Code

1. **Persistent State**:
   - The `KalmanFilter` object is stored in a `static` variable (`kalman_filter`).
   - This ensures the object persists across multiple calls to the MEX function.
2. **Initialization**:
   - The `init` command initializes the filter with process and measurement variances.
   - If the filter is already initialized, the previous instance is deleted.
3. **Reset**:
   - The `reset` command deletes the current filter, allowing a fresh start.
4. **On-Line Filtering**:
   - On each call, the input measurement is processed, and the filtered value is returned.

------

### Usage in MATLAB

```matlab
% Initialize the Kalman filter
kalman_filter('init', 1.0, 2.0);

% Simulate streaming data
raw_data = sin(0:0.1:10) + randn(1, 101) * 0.1;
filtered_data = zeros(size(raw_data));

for i = 1:length(raw_data)
    % Call the MEX function with each measurement
    filtered_data(i) = kalman_filter(raw_data(i));
end

% Plot the results
plot(0:0.1:10, raw_data, 'r--', 0:0.1:10, filtered_data, 'b-');
legend('Raw Data', 'Filtered Data');
title('Kalman Filter with Persistent State in MEX');
```

------

### Key Benefits

- **Persistence**: The filter retains its state across calls, enabling online data processing.
- **Flexibility**: Allows you to reset or reinitialize the filter as needed.
- **Performance**: The C++ implementation ensures high performance.

Would you like further clarification or help setting this up?