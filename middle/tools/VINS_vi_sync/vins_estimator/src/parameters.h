#pragma once

#include <ros/ros.h>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "utility/utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

// Fixed focal length of the camera (in pixels)
const double FOCAL_LENGTH = 460.0;
// Size of the sliding window used in VINS state estimation
const int WINDOW_SIZE = 10;
// Number of cameras used in the system
const int NUM_OF_CAM = 1;
// Maximum number of features tracked in the system
const int NUM_OF_F = 1000;
//#define UNIT_SPHERE_ERROR  // Uncomment to use unit sphere error model for features

// Initial depth assumption for triangulating new features (in meters)
extern double INIT_DEPTH;
// Minimum parallax (normalized by focal length) to select a new keyframe
extern double MIN_PARALLAX;
// Flag to control extrinsic calibration (0: fixed, 1: optimize around initial guess, 2: full calibration)
extern int ESTIMATE_EXTRINSIC;

// IMU noise parameters: acceleration white noise (m/s²) and random walk (m/s³)
extern double ACC_N, ACC_W;
// IMU noise parameters: angular velocity white noise (rad/s) and random walk (rad/s²)
extern double GYR_N, GYR_W;

// Extrinsic rotation from camera to IMU (RIC[i] is rotation for camera i)
extern std::vector<Eigen::Matrix3d> RIC;
// Extrinsic translation from camera to IMU (TIC[i] is translation for camera i)
extern std::vector<Eigen::Vector3d> TIC;
// Gravity vector (in world frame, default [0,0,9.8] m/s²)
extern Eigen::Vector3d G;

// Thresholds for IMU bias convergence check (acceleration bias in m/s²)
extern double BIAS_ACC_THRESHOLD;
// Thresholds for IMU bias convergence check (gyroscope bias in rad/s)
extern double BIAS_GYR_THRESHOLD;
// Maximum allowed time for the nonlinear solver (in seconds)
extern double SOLVER_TIME;
// Maximum number of iterations for the nonlinear solver
extern int NUM_ITERATIONS;
// Path to store extrinsic calibration results
extern std::string EX_CALIB_RESULT_PATH;
// Path to store VINS estimation results (pose, etc.)
extern std::string VINS_RESULT_PATH;
// ROS topic name for IMU data
extern std::string IMU_TOPIC;
// Time offset between camera and IMU (in seconds)
extern double TD;
// Read-out time per line for rolling shutter camera (in seconds)
extern double TR;
// Flag to enable online estimation of time offset TD (1: enable, 0: disable)
extern int ESTIMATE_TD;
// Flag to indicate if the camera is a rolling shutter (1: yes, 0: global shutter)
extern int ROLLING_SHUTTER;
// Image height and width (in pixels)
extern double ROW, COL;

// Function to read all parameters from config file and ROS parameter server
void readParameters(ros::NodeHandle &n);

// Enumeration for dimensions of state parameters in optimization
enum SIZE_PARAMETERIZATION
{
    SIZE_POSE = 7,       // Dimension of pose (3 for position + 4 for quaternion)
    SIZE_SPEEDBIAS = 9,  // Dimension of speed (3) + acceleration bias (3) + gyro bias (3)
    SIZE_FEATURE = 1     // Dimension of inverse depth for a feature
};

// Enumeration for indices of state variables in the optimization vector
enum StateOrder
{
    O_P = 0,   // Index of position (0-2)
    O_R = 3,   // Index of rotation (quaternion, 3-6)
    O_V = 6,   // Index of velocity (6-8)
    O_BA = 9,  // Index of acceleration bias (9-11)
    O_BG = 12  // Index of gyroscope bias (12-14)
};

// Enumeration for indices of noise terms in the noise vector
enum NoiseOrder
{
    O_AN = 0,  // Index of acceleration white noise (0-2)
    O_GN = 3,  // Index of gyro white noise (3-5)
    O_AW = 6,  // Index of acceleration random walk (6-8)
    O_GW = 9   // Index of gyro random walk (9-11)
};