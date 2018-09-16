# **Unscented Kalman Filter**

---

**Unscented Kalman Filter Project**

The goals / steps of this project are the following:

* Implementation of the Unscented Kalman Filter in C++ to track the position and velocity of an object.
* Achievement of a RMSE under the specified in the project rubric.

[//]: # (Image References)

[image1]: ./outputs/RMSE_Unscented.png "Undistorted"
[image2]: ./outputs/YawAngle.png "Undistorted"
[image3]: ./outputs/YawRate.png "Undistorted"
[image4]: ./outputs/Lidar_NIS.png "Undistorted"
[image5]: ./outputs/Radar_NIS.png "Undistorted"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/projects/284/view) individually and describe how I addressed each point in my implementation.  

---


### Algorithm structure and processing flow

The algorithm is divided in the following source code files located in `/src`:

`main.cpp` -> Main routine to read the sensor data and call the processing measurement function.

`ukf.cpp` -> Processing of each measurement, initialization of Kalman Filter matrices and implementation of the predict and update steps.

`tools.cpp` -> Calculation of RMSE and adjustment of diff. angle.

The main function calls the processing measurement function of `ukf.cpp`. From here the basic steps of the filter, predict and update, are performed.

#### 1. Handling of first measurement

The function `ProcessMeasurement` initializes the state vector in the first cycle with the first received measurement. The flag `is_initialized_` determines whether the initialization has already been made.

On the other hand, the initialization of the rest of variables of the filter is performed during the call of the `UKF` constructor. Here, it was necessary to experiment with the process noise and the process covariance matrix P to optimize the results. The chosen values are:

Process noise standard deviation longitudinal acceleration: 1.5 m/s2

Process noise standard deviation yaw acceleration: Pi/3 rad/s2

First two diagonal elements of P: 0.0225

#### 2. Prediction and update steps

The algorithm predicts first and then updates. `ukf.cpp` firstly calls `Prediction` and then `UpdateLidar` or `UpdateRadar`, in function of the corresponding sensor.

#### 3. Handling of radar and lidar measurements

The prediction step is the for same both sensors. Augmented sigma points are generated via `AugmentedSigmaPoints`. Then `Prediction` is called, where the sigma points are predicted (`SigmaPointPrediction`) as well as the mean and covariance (`PredictMeanAndCovariance`).

The algorithm performs the handling of the update step via `UpdateLidar` (laser) or `UpdateRadar` (radar). For laser, the routine calls `PredictLidarMeasurement` for measurement prediction and `UpdateLidarState` to perform the update. For radar, the called functions are `PredictRadarMeasurement` and `UpdateRadarState`.

The difference between radar and lidar is produced during the measurement prediction. For radar, the predicted sigma points are converted to polar coordinates and then the difference between prediction and measurement is taken. For lidar there is no conversion, and just the H matrix is applied to get the x and y positions and discard the rest of state elements. So, during the radar update we work with a vector of three elements, and during lidar with a vector of two elements.

### Accuracy and results

The algorithm reaches a final RMSE of [.0679 .0827 .3325 .2243], as shown in this [video](https://www.youtube.com/watch?v=9osHme6ntOI).

This amount satisfies the limit proposed in the project rubric ([.09 .10 .40 .30]).

It is particularly interesting the reduction of RMSE in the velocity in comparation with the obtained in the EKF project (.3325 .2243 against .4542 .4408). This is explained because of the use of the CTRV model, which allows to make a more accurate prediction during curves.

It is also of interest how the algorithm also precisely calculates the yaw angle and the yaw rate. For the yaw angle the results are:

![alt text][image2]

And for the yaw rate:

![alt text][image3]

Finally, the consistency of the filter has been measured by calculating the NIS for both lidar and radar measurements. For lidar (2 degrees of freedom, 95% limit of 5.991):

![alt text][image4]

And for radar (3 degrees of freedom, 95% limit of 7.815):

![alt text][image5]

In both cases the result matches the expected values for the chi-squared distribution. We could say that the filter is consistent, although maybe it slightly overestimates the uncertainty.


### Compilation

As specified in the rubric, the code compiles using `cmake` and `make`.

It is important to mention that `CMakeLists.txt` was moved to the `/src` folder, since the Eclipse IDE was used during the project.

### Code efficiency

With regard to this point, particular attention has been paid to the realization of repeated calculations only once and its storage in variables. This makes the code specially faster during the calculation of sigma points.

Additionally, unnecessary loops or loops that run many times have been avoided.

### Final notes

It was observed that the RMSE is influenced by the use of functions that modify the precision in the calculations. In particular, I initially used the function `pow` of the C++ `math` library to convert the sigma points to the polar sytem for radar, but the RMSE went above the limit for the X component of the velocity. Without its use (computing the square using simple multiplication) I achieved a RMSE under this limit.
