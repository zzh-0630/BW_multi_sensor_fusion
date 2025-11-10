#include "parameters.h"

// Global variables initialization (defined in parameters.h)
double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};  // Default gravity vector

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string VINS_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

/**
 * @brief Template function to read a parameter from ROS NodeHandle
 * @tparam T Type of the parameter to read
 * @param n ROS NodeHandle
 * @param name Name of the parameter in ROS parameter server
 * @return The value of the parameter
 * @note Shuts down the node if parameter reading fails
 */
template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
    T ans;
    if (n.getParam(name, ans))
    {
        ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else
    {
        ROS_ERROR_STREAM("Failed to load " << name);
        n.shutdown();  // Terminate node if parameter is missing
    }
    return ans;
}

/**
 * @brief Read all system parameters from config file and initialize global variables
 * @param n ROS NodeHandle
 * @note The config file path is first read from ROS parameter server, then parameters are loaded from this file
 */
void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    // Read the path of the config file from ROS parameter server
    config_file = readParam<std::string>(n, "config_file");
    // Open the config file using OpenCV's FileStorage (supports YAML/XML)
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return;
    }

    // Read IMU topic name (for subscribing to IMU data)
    fsSettings["imu_topic"] >> IMU_TOPIC;

    // Read solver parameters: maximum time per optimization and iteration count
    SOLVER_TIME = fsSettings["max_solver_time"];
    NUM_ITERATIONS = fsSettings["max_num_iterations"];
    // Read minimum parallax for keyframe selection (convert to normalized by focal length)
    MIN_PARALLAX = fsSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    // Read output path for results and initialize result file
    std::string OUTPUT_PATH;
    fsSettings["output_path"] >> OUTPUT_PATH;
    VINS_RESULT_PATH = OUTPUT_PATH + "/vins_result_no_loop.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;

    // Create output directory if it does not exist
    FileSystemHelper::createDirectoryIfNotExists(OUTPUT_PATH.c_str());

    // Create and close the result file to ensure it exists
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    // Read IMU noise parameters (acceleration and gyroscope)
    ACC_N = fsSettings["acc_n"];
    ACC_W = fsSettings["acc_w"];
    GYR_N = fsSettings["gyr_n"];
    GYR_W = fsSettings["gyr_w"];
    // Read gravity magnitude and update the z-component of G
    G.z() = fsSettings["g_norm"];
    // Read image dimensions (height and width)
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    ROS_INFO("ROW: %f COL: %f ", ROW, COL);

    // Read extrinsic calibration mode (0: fixed, 1: optimize, 2: calibrate from scratch)
    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        // Initialize extrinsic with identity (no prior) if full calibration is needed
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
    }
    else 
    {
        if (ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
            EX_CALIB_RESULT_PATH = OUTPUT_PATH + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        // Read initial extrinsic rotation and translation from config file
        cv::Mat cv_R, cv_T;
        fsSettings["extrinsicRotation"] >> cv_R;
        fsSettings["extrinsicTranslation"] >> cv_T;
        // Convert OpenCV Mat to Eigen matrices
        Eigen::Matrix3d eigen_R;
        Eigen::Vector3d eigen_T;
        cv::cv2eigen(cv_R, eigen_R);
        cv::cv2eigen(cv_T, eigen_T);
        // Normalize the rotation matrix to ensure orthogonality
        Eigen::Quaterniond Q(eigen_R);
        eigen_R = Q.normalized();
        RIC.push_back(eigen_R);
        TIC.push_back(eigen_T);
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl << RIC[0]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl << TIC[0].transpose());
    } 

    // Initialize depth assumption for new features and bias convergence thresholds
    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    // Read time offset between camera and IMU, and flag for online estimation
    TD = fsSettings["td"];
    ESTIMATE_TD = fsSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    // Read rolling shutter flag and read-out time per line
    ROLLING_SHUTTER = fsSettings["rolling_shutter"];
    if (ROLLING_SHUTTER)
    {
        TR = fsSettings["rolling_shutter_tr"];
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;  // No read-out time for global shutter
    }
    
    // Release the config file resource
    fsSettings.release();
}