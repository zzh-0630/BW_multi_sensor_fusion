#include "parameters.h"

// Define global variables declared in parameters.h
std::string IMAGE_TOPIC;
std::string IMU_TOPIC;
std::vector<std::string> CAM_NAMES;
std::string FISHEYE_MASK;
int MAX_CNT;
int MIN_DIST;
int WINDOW_SIZE;
int FREQ;
double F_THRESHOLD;
int SHOW_TRACK;
int STEREO_TRACK;
int EQUALIZE;
int ROW;
int COL;
int FOCAL_LENGTH;
int FISHEYE;
bool PUB_THIS_FRAME;

/**
 * @brief Template function to read a parameter from ROS node handle
 * @tparam T Type of the parameter (e.g., int, string, double)
 * @param n ROS node handle
 * @param name Name of the parameter to read
 * @return The value of the parameter
 * @note If reading fails, print error and shut down the node
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
        n.shutdown();
    }
    return ans;
}

/**
 * @brief Read system parameters from ROS parameter server and config file
 * @param n ROS node handle
 * @note Steps: 
 * 1. Read config file path from ROS parameters
 * 2. Open config file with OpenCV FileStorage
 * 3. Load parameters from config file into global variables
 * 4. Set default values for parameters not in config file
 */
void readParameters(ros::NodeHandle &n)
{
    std::string config_file;
    // Read path of the config file from ROS parameter server
    config_file = readParam<std::string>(n, "config_file");
    // Open the config file (supports YAML/XML format)
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    // Read VINS folder path from ROS parameters
    std::string VINS_FOLDER_PATH = readParam<std::string>(n, "vins_folder");

    // Load parameters from config file
    fsSettings["image_topic"] >> IMAGE_TOPIC;
    fsSettings["imu_topic"] >> IMU_TOPIC;
    MAX_CNT = fsSettings["max_cnt"];
    MIN_DIST = fsSettings["min_dist"];
    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    FREQ = fsSettings["freq"];
    F_THRESHOLD = fsSettings["F_threshold"];
    SHOW_TRACK = fsSettings["show_track"];
    EQUALIZE = fsSettings["equalize"];
    FISHEYE = fsSettings["fisheye"];
    // If using fisheye camera, set mask path (VINS folder + default mask path)
    if (FISHEYE == 1)
        FISHEYE_MASK = VINS_FOLDER_PATH + "config/fisheye_mask.jpg";
    // Add the config file path to camera name list
    CAM_NAMES.push_back(config_file);

    // Set default values for parameters not in config file
    WINDOW_SIZE = 20; // Default sliding window size
    STEREO_TRACK = false; // Default: not use stereo tracking
    FOCAL_LENGTH = 460; // Default focal length (pixels)
    PUB_THIS_FRAME = false; // Default: not publish current frame

    // Set default frequency if not specified (0 -> 100 Hz)
    if (FREQ == 0)
        FREQ = 100;

    // Release the config file handle
    fsSettings.release();


}
