#ifndef FEATURE_MANAGER_H
#define FEATURE_MANAGER_H

#include <list>
#include <algorithm>
#include <vector>
#include <numeric>
using namespace std;

#include <eigen3/Eigen/Dense>
using namespace Eigen;

#include <ros/console.h>
#include <ros/assert.h>

#include "parameters.h"

// Store information of a single feature observation in one frame
class FeaturePerFrame
{
  public:
    /**
     * @brief Constructor: initialize feature info from a 7D vector and time offset
     * @param _point 7D vector [x, y, z, u, v, vx, vy] (3D coordinate, image uv, velocity)
     * @param td Time offset between camera and IMU
     */
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        point.x() = _point(0); // 3D coordinate in camera frame (normalized plane)
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3); // Image plane coordinates (u, v)
        uv.y() = _point(4);
        velocity.x() = _point(5); // Feature velocity in image plane (vx, vy)
        velocity.y() = _point(6); 
        cur_td = td; // Time offset at current frame
    }
    double cur_td;             // Time offset between camera and IMU for current frame
    Vector3d point;            // 3D coordinate in camera frame (normalized, z=1 if depth unknown)
    Vector2d uv;               // Image coordinates (u, v)
    Vector2d velocity;         // Velocity in image plane
    double z;                  // Depth (not used in current implementation)
    bool is_used;              // Whether this observation is used in optimization
    double parallax;           // Parallax with previous frame
    MatrixXd A;                // Jacobian matrix for optimization
    VectorXd b;                // Residual vector for optimization
    double dep_gradient;       // Depth gradient (for outlier detection)
};

// Store all observations of a feature with the same ID across multiple frames
class FeaturePerId
{
  public:
    const int feature_id;              // Unique feature ID
    int start_frame;                   // The first frame this feature is observed
    vector<FeaturePerFrame> feature_per_frame;  // Observations in each frame

    int used_num;                      // Number of valid observations
    bool is_outlier;                   // Whether this feature is an outlier
    bool is_margin;                    // Whether this feature is marginalized
    double estimated_depth;            // Estimated depth (inverse depth in optimization)
    int solve_flag;                    // Depth estimation status: 0=unsolved, 1=success, 2=fail

    Vector3d gt_p;                     // Ground truth 3D position (for debugging)

    /**
     * @brief Constructor: initialize with feature ID and start frame
     * @param _feature_id Unique feature ID
     * @param _start_frame First observed frame index
     */
    FeaturePerId(int _feature_id, int _start_frame)
        : feature_id(_feature_id), start_frame(_start_frame),
          used_num(0), estimated_depth(-1.0), solve_flag(0) // Initialize depth as invalid
    {
    }

    /**
     * @brief Get the last frame index this feature is observed
     * @return Last frame index
     */
    int endFrame();
};

// Manage all features in the sliding window, handle feature tracking, parallax calculation, depth estimation, etc.
class FeatureManager
{
  public:
    /**
     * @brief Constructor: initialize with rotation matrices of IMU frames
     * @param _Rs Array of rotation matrices from IMU to world frame in each window frame
     */
    FeatureManager(Matrix3d _Rs[]);

    /**
     * @brief Set camera extrinsic rotation (ric: rotation from camera to IMU)
     * @param _ric Array of camera-IMU rotation matrices for each camera
     */
    void setRic(Matrix3d _ric[]);

    /**
     * @brief Clear all feature data
     */
    void clearState();

    /**
     * @brief Get the number of valid features (tracked in enough frames)
     * @return Count of valid features
     */
    int getFeatureCount();

    /**
     * @brief Add new features from current frame and check parallax for keyframe selection
     * @param frame_count Current frame index in the window
     * @param image Map of {feature ID -> {camera ID, 7D feature info}}
     * @param td Time offset between camera and IMU
     * @return True if parallax is sufficient (or not enough frames), False otherwise
     */
    bool addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td);
    
    /**
     * @brief Print feature info for debugging
     */
    void debugShow();
    /**
     * @brief Get corresponding 3D features between two frames in the window
     * @param frame_count_l Left frame index
     * @param frame_count_r Right frame index
     * @return Pairs of 3D points (left frame, right frame)
     */
    vector<pair<Vector3d, Vector3d>> getCorresponding(int frame_count_l, int frame_count_r);

    /**
     * @brief Set feature depths from optimization results (inverse depth)
     * @param x Optimization result vector (inverse depths)
     */
    void setDepth(const VectorXd &x);
    /**
     * @brief Remove features with failed depth estimation
     */
    void removeFailures();
    /**
     * @brief Clear and reset feature depths from optimization results
     * @param x Optimization result vector (inverse depths)
     */
    void clearDepth(const VectorXd &x);

    /**
     * @brief Get a vector of inverse depths of all valid features
     * @return Vector of inverse depths
     */
    VectorXd getDepthVector();

    /**
     * @brief Triangulate 3D positions of features using multiple frames in the window
     * @param Ps Array of IMU positions in world frame
     * @param tic Array of camera-IMU translation vectors
     * @param ric Array of camera-IMU rotation matrices
     */
    void triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[]);

    /**
     * @brief Remove the oldest frame in the window and adjust feature depths (after marginalization)
     * @param marg_R Rotation of the marginalized frame (IMU to world)
     * @param marg_P Position of the marginalized frame (IMU in world)
     * @param new_R Rotation of the new first frame (IMU to world)
     * @param new_P Position of the new first frame (IMU in world)
     */
    void removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P);

    /**
     * @brief Remove the oldest frame in the window (simplified version)
     */
    void removeBack();

    /**
     * @brief Remove the front frame (latest frame) in the window
     * @param frame_count Current frame index
     */
    void removeFront(int frame_count);

    /**
     * @brief Remove outlier features
     */
    void removeOutlier();

    list<FeaturePerId> feature;  // List of all features (tracked by ID)
    int last_track_num;          // Number of features tracked from previous frame

private:
    /**
     * @brief Calculate compensated parallax squared between two consecutive frames
     * @param it_per_id Feature to calculate parallax
     * @param frame_count Current frame index
     * @return Compensated parallax (squared)
     */
    double compensatedParallax2(const FeaturePerId &it_per_id, int frame_count);

    const Matrix3d *Rs;  // Rotation matrices of IMU frames (from IMU to world)
    Matrix3d ric[NUM_OF_CAM];  // Camera-IMU rotation matrices (for each camera)
};

#endif