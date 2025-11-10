#pragma once
// Include ROS header for ROS node and communication
#include <ros/ros.h>
// Include OpenCV highgui for image processing
#include <opencv2/highgui/highgui.hpp>

// Image rows (height) of the input camera
extern int ROW;
// Image columns (width) of the input camera
extern int COL;
// Focal length of the camera (in pixels)
extern int FOCAL_LENGTH;
// Number of cameras (fixed as 1 here)
const int NUM_OF_CAM = 1;


// ROS topic name for input images
extern std::string IMAGE_TOPIC;
// ROS topic name for input IMU data
extern std::string IMU_TOPIC;
// Path to the fisheye mask image (for distortion correction)
extern std::string FISHEYE_MASK;
// List of camera configuration files (paths)
extern std::vector<std::string> CAM_NAMES;
// Maximum number of feature points to track in each frame
extern int MAX_CNT;
// Minimum distance (in pixels) between adjacent tracked features
extern int MIN_DIST;
// Size of the sliding window for visual-inertial optimization
extern int WINDOW_SIZE;
// Frequency of image processing (in Hz)
extern int FREQ;
// Threshold for fundamental matrix estimation (RANSAC)
extern double F_THRESHOLD;
// Flag: whether to show feature tracking results (1 for show, 0 for not)
extern int SHOW_TRACK;
// Flag: whether to use stereo tracking (1 for stereo, 0 for mono)
extern int STEREO_TRACK;
// Flag: whether to equalize image histogram (1 for equalize, 0 for not)
extern int EQUALIZE;
// Flag: whether the camera is a fisheye lens (1 for fisheye, 0 for pinhole)
extern int FISHEYE;
// Flag: whether to publish the current frame (for visualization or downstream modules)
extern bool PUB_THIS_FRAME;

// Function to read parameters from ROS and config file
void readParameters(ros::NodeHandle &n);
