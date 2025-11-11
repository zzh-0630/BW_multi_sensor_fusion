#pragma once

#include "parameters.h"
#include "feature_manager.h"
#include "utility/utility.h"
#include "utility/tic_toc.h"
#include "initial/solve_5pts.h"
#include "initial/initial_sfm.h"
#include "initial/initial_alignment.h"
#include "initial/initial_ex_rotation.h"
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>

#include <ceres/ceres.h>
#include "factor/imu_factor.h"
#include "factor/pose_local_parameterization.h"
#include "factor/projection_factor.h"
#include "factor/projection_td_factor.h"
#include "factor/marginalization_factor.h"

#include <unordered_map>
#include <queue>
#include <opencv2/core/eigen.hpp>

/**
 * @brief Core estimator class for Visual-Inertial Navigation System (VINS)
 *        Fuses IMU data and image features to estimate state (pose, velocity, biases, etc.)
 */
class Estimator
{
  public:
    /**
     * @brief Constructor: Initializes feature manager with rotation states
     */
    Estimator();

    /**
     * @brief Sets initial parameters (extrinsics, noise covariance, etc.)
     */
    void setParameter();

    // Interface functions
    /**
     * @brief Processes IMU data: performs pre-integration and updates current state
     * @param t Timestamp of IMU data
     * @param linear_acceleration Linear acceleration from IMU
     * @param angular_velocity Angular velocity from IMU
     */
    void processIMU(double t, const Vector3d &linear_acceleration, const Vector3d &angular_velocity);
    
    /**
     * @brief Processes image feature data: manages features, triggers initialization/optimization
     * @param image Map of camera IDs to feature observations (id, [u, v, u', v', vx, vy, depth])
     * @param header ROS header containing timestamp
     */
    void processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header);
    
    /**
     * @brief Sets relocalization information to correct drift
     * @param _frame_stamp Timestamp of the relocalization frame
     * @param _frame_index Index of the relocalization frame in global map
     * @param _match_points Matched 2D features between current frame and relocalization frame
     * @param _relo_t Translation of the relocalization frame
     * @param _relo_r Rotation of the relocalization frame
     */
    void setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r);

    // Internal functions
    /**
     * @brief Clears all state variables (poses, velocities, biases, etc.) for system reset
     */
    void clearState();
    
    /**
     * @brief Initializes 3D structure and camera poses using SFM (Structure from Motion)
     * @return True if initialization succeeds, false otherwise
     */
    bool initialStructure();
    
    /**
     * @brief Aligns visual structure (from SFM) with IMU data to get scale, gravity, and velocities
     * @return True if alignment succeeds, false otherwise
     */
    bool visualInitialAlign();
    
    /**
     * @brief Estimates relative pose between two frames using feature correspondences
     * @param relative_R Output relative rotation
     * @param relative_T Output relative translation
     * @param l Output index of the reference frame
     * @return True if relative pose is valid, false otherwise
     */
    bool relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l);
    
    /**
     * @brief Slides the window by marginalizing old or second-newest frame
     */
    void slideWindow();
    
    /**
     * @brief Solves odometry using triangulation and nonlinear optimization
     */
    void solveOdometry();
    
    /**
     * @brief Slides window by marginalizing the second-newest frame (non-keyframe)
     */
    void slideWindowNew();
    
    /**
     * @brief Slides window by marginalizing the oldest frame (keyframe)
     */
    void slideWindowOld();
    
    /**
     * @brief Performs nonlinear optimization using Ceres Solver
     *        Adds IMU factors, projection factors, and marginalization factors
     */
    void optimization();
    
    /**
     * @brief Converts state variables (Eigen vectors) to double arrays for Ceres optimization
     */
    void vector2double();
    
    /**
     * @brief Converts optimized double arrays back to Eigen vectors (updates state variables)
     */
    void double2vector();
    
    /**
     * @brief Detects system failures (e.g., insufficient features, large biases, pose jumps)
     * @return True if failure is detected, false otherwise
     */
    bool failureDetection();

    // Enums for solver and marginalization states
    enum SolverFlag
    {
        INITIAL,    ///< System in initialization phase
        NON_LINEAR  ///< System in nonlinear optimization phase
    };

    enum MarginalizationFlag
    {
        MARGIN_OLD = 0,          ///< Marginalize the oldest frame
        MARGIN_SECOND_NEW = 1    ///< Marginalize the second-newest frame
    };

    SolverFlag solver_flag;               ///< Current solver state (initial/non-linear)
    MarginalizationFlag marginalization_flag; ///< Flag for marginalization strategy
    Vector3d g;                           ///< Gravity vector in world frame
    MatrixXd Ap[2], backup_A;             ///< Intermediate matrices for optimization
    VectorXd bp[2], backup_b;             ///< Intermediate vectors for optimization

    Matrix3d ric[NUM_OF_CAM];             ///< Rotation from camera to IMU (extrinsics)
    Vector3d tic[NUM_OF_CAM];             ///< Translation from camera to IMU (extrinsics)

    Vector3d Ps[(WINDOW_SIZE + 1)];       ///< Position of IMU in world frame (window states)
    Vector3d Vs[(WINDOW_SIZE + 1)];       ///< Velocity of IMU in world frame (window states)
    Matrix3d Rs[(WINDOW_SIZE + 1)];       ///< Rotation of IMU in world frame (window states)
    Vector3d Bas[(WINDOW_SIZE + 1)];      ///< Acceleration bias (window states)
    Vector3d Bgs[(WINDOW_SIZE + 1)];      ///< Gyroscope bias (window states)
    double td;                            ///< Time offset between camera and IMU

    Matrix3d back_R0, last_R, last_R0;    ///< Backup rotations for window sliding
    Vector3d back_P0, last_P, last_P0;    ///< Backup positions for window sliding
    std_msgs::Header Headers[(WINDOW_SIZE + 1)]; ///< Timestamps of frames in window

    IntegrationBase *pre_integrations[(WINDOW_SIZE + 1)]; ///< IMU pre-integrators for each frame
    Vector3d acc_0, gyr_0;                ///< Previous IMU measurements (for pre-integration)

    vector<double> dt_buf[(WINDOW_SIZE + 1)]; ///< Buffer of IMU time intervals
    vector<Vector3d> linear_acceleration_buf[(WINDOW_SIZE + 1)]; ///< Buffer of linear accelerations
    vector<Vector3d> angular_velocity_buf[(WINDOW_SIZE + 1)];    ///< Buffer of angular velocities

    int frame_count;                      ///< Number of frames processed
    int sum_of_outlier, sum_of_back, sum_of_front, sum_of_invalid; ///< Statistics for marginalization

    FeatureManager f_manager;             ///< Manages feature tracking and triangulation
    MotionEstimator m_estimator;          ///< Estimates relative motion between frames
    InitialEXRotation initial_ex_rotation; ///< Initializes camera-IMU rotation extrinsics

    bool first_imu;                       ///< Flag for first IMU data
    bool is_valid, is_key;                ///< Flags for valid/key frames
    bool failure_occur;                   ///< Flag for system failure

    vector<Vector3d> point_cloud;         ///< Reconstructed 3D points
    vector<Vector3d> margin_cloud;        ///< Marginalized 3D points
    vector<Vector3d> key_poses;           ///< Keyframe poses for output
    double initial_timestamp;             ///< Timestamp of initialization

    // Parameters for Ceres optimization (double arrays)
    double para_Pose[WINDOW_SIZE + 1][SIZE_POSE];       ///< Pose parameters (x,y,z,qx,qy,qz,qw)
    double para_SpeedBias[WINDOW_SIZE + 1][SIZE_SPEEDBIAS]; ///< Speed and bias parameters (vx,vy,vz,ba_x,ba_y,ba_z,bg_x,bg_y,bg_z)
    double para_Feature[NUM_OF_F][SIZE_FEATURE];         ///< Feature depth parameters
    double para_Ex_Pose[NUM_OF_CAM][SIZE_POSE];          ///< Extrinsic parameters (camera-IMU)
    double para_Retrive_Pose[SIZE_POSE];                 ///< Retrieved pose for relocalization
    double para_Td[1][1];                               ///< Time offset parameter
    double para_Tr[1][1];                               ///< Reserved parameter

    int loop_window_index;                ///< Index of loop frame in window

    MarginalizationInfo *last_marginalization_info; ///< Information for marginalization
    vector<double *> last_marginalization_parameter_blocks; ///< Parameter blocks for marginalization

    map<double, ImageFrame> all_image_frame; ///< Map of timestamp to image frame data
    IntegrationBase *tmp_pre_integration;   ///< Temporary pre-integrator for new frames

    // Relocalization variables
    bool relocalization_info;             ///< Flag for valid relocalization info
    double relo_frame_stamp;              ///< Timestamp of relocalization frame
    double relo_frame_index;              ///< Global index of relocalization frame
    int relo_frame_local_index;           ///< Local index of relocalization frame in window
    vector<Vector3d> match_points;        ///< Matched features for relocalization
    double relo_Pose[SIZE_POSE];          ///< Pose of relocalization frame in window
    Matrix3d drift_correct_r;             ///< Rotation correction for drift
    Vector3d drift_correct_t;             ///< Translation correction for drift
    Vector3d prev_relo_t;                 ///< Previous relocalization translation
    Matrix3d prev_relo_r;                 ///< Previous relocalization rotation
    Vector3d relo_relative_t;             ///< Relative translation for relocalization
    Quaterniond relo_relative_q;          ///< Relative rotation for relocalization
    double relo_relative_yaw;             ///< Relative yaw for relocalization
};