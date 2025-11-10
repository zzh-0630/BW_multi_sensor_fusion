#pragma once

#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"

#include "parameters.h"
#include "tic_toc.h"

using namespace std;
using namespace camodocal;
using namespace Eigen;

/**
 * @brief Check if a 2D point is within the image border
 * @param pt The 2D point to check (in image coordinates)
 * @return True if the point is inside the border, false otherwise
 */
bool inBorder(const cv::Point2f &pt);

/**
 * @brief Reduce a vector of 2D points by keeping only those with valid status
 * @param v The vector of points to be reduced
 * @param status The status vector (uchar, typically 0 for invalid, non-zero for valid)
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
/**
 * @brief Reduce a vector of integers by keeping only those with valid status
 * @param v The vector of integers to be reduced
 * @param status The status vector (uchar, typically 0 for invalid, non-zero for valid)
 */
void reduceVector(vector<int> &v, vector<uchar> status);

/**
 * @brief Class for tracking image features across consecutive frames, handling undistortion, 
 *        and managing feature IDs and tracking counts.
 */
class FeatureTracker
{
  public:
    /**
     * @brief Constructor: Initialize feature tracker members
     */
    FeatureTracker();

    /**
     * @brief Read current image, process feature tracking from previous frame to current frame
     * @param _img Input current image
     * @param _cur_time Timestamp of the current image
     * @param encoding Image encoding (e.g., "mono8" for grayscale)
     */
    void readImage(const cv::Mat &_img, double _cur_time, const std::string& encoding);

    /**
     * @brief Set a mask to avoid extracting new features in areas with dense existing features
     *        (used to ensure uniform feature distribution)
     */
    void setMask();

    /**
     * @brief Add new features to maintain sufficient feature count (e.g., when some features are lost)
     */
    void addPoints();

    /**
     * @brief Update the ID of the i-th feature point (assign new ID to newly detected features)
     * @param i Index of the feature point in the current frame
     * @return True if ID is updated successfully, false otherwise
     */
    bool updateID(unsigned int i);

    /**
     * @brief Read camera intrinsic parameters from calibration file and initialize camera model
     * @param calib_file Path to the camera calibration file
     */
    void readIntrinsicParameter(const string &calib_file);

    /**
     * @brief Visualize the undistortion effect of feature points (for debugging)
     * @param name Window name for visualization
     */
    void showUndistortion(const string &name);

    /**
     * @brief Reject outlier feature matches using the Fundamental Matrix (via RANSAC)
     */
    void rejectWithF();

    /**
     * @brief Undistort feature points using camera intrinsic parameters and distortion model
     *        (convert distorted image points to undistorted normalized/plane points)
     */
    void undistortedPoints();

    cv::Mat mask;                  // Mask for controlling new feature distribution (avoid dense clusters)
    cv::Mat fisheye_mask;          // Special mask for fisheye cameras (if applicable)
    cv::Mat prev_img;              // Previous frame image
    cv::Mat cur_img;               // Current frame image
    cv::Mat forw_img;              // Next frame image (forward frame)
    vector<cv::Point2f> n_pts;     // Newly detected feature points in current frame
    vector<cv::Point2f> prev_pts;  // Feature points in previous frame (distorted)
    vector<cv::Point2f> cur_pts;   // Feature points in current frame (distorted)
    vector<cv::Point2f> forw_pts;  // Feature points in forward frame (distorted)
    vector<cv::Point2f> prev_un_pts; // Undistorted feature points in previous frame
    vector<cv::Point2f> cur_un_pts;  // Undistorted feature points in current frame
    vector<cv::Point2f> pts_velocity; // Velocity of feature points (pixel motion per unit time)
    vector<int> ids;               // IDs of feature points (to track across frames)
    vector<int> track_cnt;         // Tracking count for each feature (how many frames it's been tracked)
    map<int, cv::Point2f> cur_un_pts_map; // Map from feature ID to its undistorted position in current frame
    map<int, cv::Point2f> prev_un_pts_map; // Map from feature ID to its undistorted position in previous frame
    camodocal::CameraPtr m_camera; // Camera model pointer (supports pinhole, catadioptric, etc.)
    double cur_time;               // Timestamp of current frame
    double prev_time;              // Timestamp of previous frame

    static int n_id;               // Static variable to assign unique IDs to new features
};