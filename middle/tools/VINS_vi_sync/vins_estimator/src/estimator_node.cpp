#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include "estimator.h"
#include "parameters.h"
#include "utility/visualization.h"

// New publishers: publish IMU data between frames and time offset (td)
ros::Publisher pub_interframe_imu;  // Publisher for IMU data between consecutive image frames
ros::Publisher pub_td;              // Publisher for current time offset estimation (IMU-camera synchronization)

Estimator estimator; // Core estimator instance for VIO state estimation

// Synchronization primitives for thread-safe data handling
std::condition_variable con;  // Condition variable to notify process thread of new data
double current_time = -1;     // Current processing time
queue<sensor_msgs::ImuConstPtr> imu_buf;               // Buffer for IMU messages
queue<sensor_msgs::PointCloudConstPtr> feature_buf;    // Buffer for visual feature messages
queue<sensor_msgs::PointCloudConstPtr> relo_buf;       // Buffer for relocalization messages
int sum_of_wait = 0;          // Counter for waiting times (debug) 

std::mutex m_buf;    // Mutex for protecting data buffers
std::mutex m_state;  // Mutex for protecting state variables
std::mutex i_buf;    // (Unused in current snippet)
std::mutex m_estimator;  // Mutex for protecting estimator operations

double latest_time;  // Latest timestamp of processed IMU data
// Temporary variables for state prediction (position, orientation, velocity, biases)
Eigen::Vector3d tmp_P;
Eigen::Quaterniond tmp_Q;
Eigen::Vector3d tmp_V;
Eigen::Vector3d tmp_Ba;
Eigen::Vector3d tmp_Bg;
Eigen::Vector3d acc_0;  // Previous acceleration measurement
Eigen::Vector3d gyr_0;  // Previous angular velocity measurement
bool init_feature = 0;  // Flag to skip first feature message (no optical flow)
bool init_imu = 1;      // Flag to initialize IMU processing
double last_imu_t = 0;  // Timestamp of last received IMU message

/**
 * @brief Predict IMU state (position, orientation, velocity) using latest IMU data
 * @param imu_msg New IMU message containing acceleration and angular velocity
 */
void predict(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    if (init_imu) // Initialize IMU time on first call
    {
        latest_time = t;
        init_imu = 0;
        return;
    }
    double dt = t - latest_time; // Time interval since last IMU message
    latest_time = t;

    // Extract acceleration and angular velocity from IMU message
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    // Transform acceleration to world frame (compensate for orientation and bias)
    Eigen::Vector3d un_acc_0 = tmp_Q * (acc_0 - tmp_Ba) - estimator.g;

    // Integrate angular velocity to update orientation (midpoint integration)
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - tmp_Bg;
    tmp_Q = tmp_Q * Utility::deltaQ(un_gyr * dt);

    // Update acceleration in world frame using new orientation
    Eigen::Vector3d un_acc_1 = tmp_Q * (linear_acceleration - tmp_Ba) - estimator.g;

    // Midpoint integration for position and velocity
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    tmp_P = tmp_P + dt * tmp_V + 0.5 * dt * dt * un_acc;  // Update position
    tmp_V = tmp_V + dt * un_acc;                          // Update velocity

    // Save current measurements for next integration step
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/**
 * @brief Update temporary state variables using latest estimator results
 *        (Synchronize prediction with estimator's optimized state)
 */
void update()
{
    TicToc t_predict; //Timer for debug
    latest_time = current_time;
    // Update temporary state with estimator's latest output (at window end)
    tmp_P = estimator.Ps[WINDOW_SIZE];
    tmp_Q = estimator.Rs[WINDOW_SIZE];
    tmp_V = estimator.Vs[WINDOW_SIZE];
    tmp_Ba = estimator.Bas[WINDOW_SIZE];
    tmp_Bg = estimator.Bgs[WINDOW_SIZE];
    acc_0 = estimator.acc_0;
    gyr_0 = estimator.gyr_0;

    // Re-predict using all IMU data in buffer to catch up with latest state
    queue<sensor_msgs::ImuConstPtr> tmp_imu_buf = imu_buf;
    for (sensor_msgs::ImuConstPtr tmp_imu_msg; !tmp_imu_buf.empty(); tmp_imu_buf.pop())
        predict(tmp_imu_buf.front());

}

/**
 * @brief Extract synchronized IMU and image feature measurements from buffers
 * @return Vector of (IMU messages, image message) pairs with time alignment
 */
std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>>
getMeasurements()
{
    std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;

    while (true)
    {
        if (imu_buf.empty() || feature_buf.empty())
            return measurements; // No enough data, return empty

        // Check if IMU data covers the image time (considering time offset td)
        if (!(imu_buf.back()->header.stamp.toSec() > feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            sum_of_wait++;// Increase wait counter (debug)
            return measurements;
        }

        // Remove outdated image messages (IMU data starts after image time)
        if (!(imu_buf.front()->header.stamp.toSec() < feature_buf.front()->header.stamp.toSec() + estimator.td))
        {
            ROS_WARN("throw img, only should happen at the beginning");
            feature_buf.pop();
            continue;
        }

        // Extract the earliest image message
        sensor_msgs::PointCloudConstPtr img_msg = feature_buf.front();
        feature_buf.pop();

        // Collect all IMU messages before the image time (plus td)
        std::vector<sensor_msgs::ImuConstPtr> IMUs;
        while (imu_buf.front()->header.stamp.toSec() < img_msg->header.stamp.toSec() + estimator.td)
        {
            IMUs.emplace_back(imu_buf.front());
            imu_buf.pop();
        }
        IMUs.emplace_back(imu_buf.front()); // Include the first IMU after image time for interpolation
        if (IMUs.empty())
            ROS_WARN("no imu between two image");
        measurements.emplace_back(IMUs, img_msg);
    }
    return measurements;
}

/**
 * @brief Callback function for IMU messages
 * @param imu_msg Received IMU message
 *        - Stores IMU data in buffer
 *        - Triggers state prediction
 *        - Notifies process thread
 */
void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    if (imu_msg->header.stamp.toSec() <= last_imu_t)
    {
        ROS_WARN("imu message in disorder!"); // Detect out-of-order IMU messages
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();

    // Lock buffer and push new IMU message
    m_buf.lock();
    imu_buf.push(imu_msg);
    m_buf.unlock();
    con.notify_one(); // Notify process thread of new data

    last_imu_t = imu_msg->header.stamp.toSec();

    // Update state prediction with new IMU data
    {
        std::lock_guard<std::mutex> lg(m_state);
        predict(imu_msg);
        std_msgs::Header header = imu_msg->header;
        header.frame_id = "world";
        // Publish latest odometry if estimator is in non-linear optimization mode
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
    }
}

/**
 * @brief Callback function for visual feature messages
 * @param feature_msg Received feature message (from feature tracker)
 *        - Skips first message (no optical flow)
 *        - Stores feature data in buffer
 *        - Notifies process thread
 */ 
void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    if (!init_feature)
    {
        //skip the first detected feature, which doesn't contain optical flow speed
        init_feature = 1;
        return;
    }
    // Lock buffer and push new feature message
    m_buf.lock();
    feature_buf.push(feature_msg);
    m_buf.unlock();
    con.notify_one(); // Notify process thread of new data
}

/**
 * @brief Callback function for restart command
 * @param restart_msg Boolean message to trigger restart
 *        - Clears buffers and resets estimator state
 */
void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        m_buf.lock();
        // Clear all data buffers
        while(!feature_buf.empty())
            feature_buf.pop();
        while(!imu_buf.empty())
            imu_buf.pop();
        m_buf.unlock();
        m_estimator.lock();
        estimator.clearState(); // Reset estimator internal state
        estimator.setParameter();
        m_estimator.unlock();
        current_time = -1;
        last_imu_t = 0;
    }
    return;
}

/**
 * @brief Callback function for relocalization messages
 * @param points_msg Received relocalization data (match points and pose)
 *        - Stores relocalization data in buffer
 */
void relocalization_callback(const sensor_msgs::PointCloudConstPtr &points_msg)
{
    m_buf.lock();
    relo_buf.push(points_msg); // Push relocalization data to buffer
    m_buf.unlock();
}

// thread: visual-inertial odometry
/**
 * @brief Main processing thread for VIO
 *        - Waits for synchronized measurements
 *        - Processes IMU and image data
 *        - Updates estimator and publishes results
 */
void process()
{
    while (true)
    {
        std::vector<std::pair<std::vector<sensor_msgs::ImuConstPtr>, sensor_msgs::PointCloudConstPtr>> measurements;
        
        // Wait for new measurements using condition variable
        std::unique_lock<std::mutex> lk(m_buf);
        con.wait(lk, [&]
                 {
            return (measurements = getMeasurements()).size() != 0;
                 });
        lk.unlock();
        m_estimator.lock();
        // Publish inter-frame IMU data and time offset (td)
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            auto imu_buf = measurement.first;
            
            // Publish all IMU messages in this measurement group
            for (auto &imu_msg : imu_buf)
            {
                pub_interframe_imu.publish(imu_msg);
            }
            
            // Publish current time offset estimation
            std_msgs::Float64 td_msg;
            td_msg.data = estimator.td; 
            pub_td.publish(td_msg);
            
            ROS_DEBUG("Published %zu IMU messages and td: %f for image stamp: %f", 
                     imu_buf.size(), estimator.td, img_msg->header.stamp.toSec());
        }
        
        // Process each (IMU, image) measurement pair
        for (auto &measurement : measurements)
        {
            auto img_msg = measurement.second;
            double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;
            // Process all IMU messages in the measurement group
            for (auto &imu_msg : measurement.first)
            {
                double t = imu_msg->header.stamp.toSec();
                double img_t = img_msg->header.stamp.toSec() + estimator.td; // Image time with td compensation
                if (t <= img_t) // IMU time is before/at image time
                { 
                    if (current_time < 0)
                        current_time = t;
                    double dt = t - current_time; // Time since last processed IMU
                    ROS_ASSERT(dt >= 0);
                    current_time = t;
                    // Extract IMU data
                    dx = imu_msg->linear_acceleration.x;
                    dy = imu_msg->linear_acceleration.y;
                    dz = imu_msg->linear_acceleration.z;
                    rx = imu_msg->angular_velocity.x;
                    ry = imu_msg->angular_velocity.y;
                    rz = imu_msg->angular_velocity.z;
                    // Update estimator with IMU data
                    estimator.processIMU(dt, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
                else // IMU time is after image time: interpolate IMU data to image time
                {
                    double dt_1 = img_t - current_time; // Time from last IMU to image time
                    double dt_2 = t - img_t; // Time from image time to current IMU
                    current_time = img_t;
                    ROS_ASSERT(dt_1 >= 0);
                    ROS_ASSERT(dt_2 >= 0);
                    ROS_ASSERT(dt_1 + dt_2 > 0);
                    // Weighted average for interpolation
                    double w1 = dt_2 / (dt_1 + dt_2);
                    double w2 = dt_1 / (dt_1 + dt_2);
                    dx = w1 * dx + w2 * imu_msg->linear_acceleration.x;
                    dy = w1 * dy + w2 * imu_msg->linear_acceleration.y;
                    dz = w1 * dz + w2 * imu_msg->linear_acceleration.z;
                    rx = w1 * rx + w2 * imu_msg->angular_velocity.x;
                    ry = w1 * ry + w2 * imu_msg->angular_velocity.y;
                    rz = w1 * rz + w2 * imu_msg->angular_velocity.z;
                    // Update estimator with interpolated IMU data
                    estimator.processIMU(dt_1, Vector3d(dx, dy, dz), Vector3d(rx, ry, rz));
                }
            }
            // set relocalization frame
            // Handle relocalization data if available
            sensor_msgs::PointCloudConstPtr relo_msg = NULL;
            while (!relo_buf.empty())
            {
                relo_msg = relo_buf.front();
                relo_buf.pop();
            }
            if (relo_msg != NULL)
            {
                vector<Vector3d> match_points;
                double frame_stamp = relo_msg->header.stamp.toSec();
                // Extract match points from relocalization message
                for (unsigned int i = 0; i < relo_msg->points.size(); i++)
                {
                    Vector3d u_v_id;
                    u_v_id.x() = relo_msg->points[i].x;
                    u_v_id.y() = relo_msg->points[i].y;
                    u_v_id.z() = relo_msg->points[i].z;
                    match_points.push_back(u_v_id);
                }
                // Extract relocalization pose (translation and rotation)
                Vector3d relo_t(relo_msg->channels[0].values[0], relo_msg->channels[0].values[1], relo_msg->channels[0].values[2]);
                Quaterniond relo_q(relo_msg->channels[0].values[3], relo_msg->channels[0].values[4], relo_msg->channels[0].values[5], relo_msg->channels[0].values[6]);
                Matrix3d relo_r = relo_q.toRotationMatrix();
                int frame_index;
                frame_index = relo_msg->channels[0].values[7];
                // Set relocalization frame in estimator
                estimator.setReloFrame(frame_stamp, frame_index, match_points, relo_t, relo_r);
            }

            ROS_DEBUG("processing vision data with stamp %f \n", img_msg->header.stamp.toSec());

            TicToc t_s; // Timer for vision processing
            // Parse image feature data into a map (feature ID -> (camera ID, feature info))
            map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> image;
            for (unsigned int i = 0; i < img_msg->points.size(); i++)
            {
                int v = img_msg->channels[0].values[i] + 0.5;
                int feature_id = v / NUM_OF_CAM; // Extract feature ID
                int camera_id = v % NUM_OF_CAM; // Extract camera ID (for multi-camera)
                // Extract feature coordinates, pixel position, and velocity
                double x = img_msg->points[i].x;
                double y = img_msg->points[i].y;
                double z = img_msg->points[i].z;
                double p_u = img_msg->channels[1].values[i];
                double p_v = img_msg->channels[2].values[i];
                double velocity_x = img_msg->channels[3].values[i];
                double velocity_y = img_msg->channels[4].values[i];
                ROS_ASSERT(z == 1); // z=1 for normalized plane
                Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
                xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
                image[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
            }
            // Process image features in estimator (visual-inertial fusion)
            estimator.processImage(image, img_msg->header);

            // Publish estimation results
            double whole_t = t_s.toc();
            printStatistics(estimator, whole_t); // Print processing stats
            std_msgs::Header header = img_msg->header;
            header.frame_id = "world";

            pubOdometry(estimator, header);        // Publish odometry (position, orientation, velocity)
            pubKeyPoses(estimator, header);        // Publish keyframe poses
            pubCameraPose(estimator, header);      // Publish camera pose
            pubPointCloud(estimator, header);      // Publish 3D point cloud
            pubTF(estimator, header);              // Publish TF transform
            pubKeyframe(estimator);                // Publish keyframe data
            if (relo_msg != NULL)
                pubRelocalization(estimator);      // Publish relocalization result
        }
        m_estimator.unlock();

        // Update temporary prediction with latest estimator state
        m_buf.lock();
        m_state.lock();
        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
            update();
        m_state.unlock();
        m_buf.unlock();
    }
}

/**
 * @brief Main function: initialize node and start processing
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_estimator"); // Initialize ROS node
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n); // Read configuration parameters
    estimator.setParameter(); // Configure estimator
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE"); // Disable Eigen parallelism if defined
#endif
    ROS_WARN("waiting for image and imu...");

    registerPub(n); // Register default publishers

    // Initialize new publishers
    pub_interframe_imu = n.advertise<sensor_msgs::Imu>("/interframe_imu", 1000);
    pub_td = n.advertise<std_msgs::Float64>("/current_td", 10);

    // Subscribe to input topics
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_image = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    ros::Subscriber sub_restart = n.subscribe("/feature_tracker/restart", 2000, restart_callback);
    ros::Subscriber sub_relo_points = n.subscribe("/pose_graph/match_points", 2000, relocalization_callback);

    // Start main processing thread
    std::thread measurement_process{process};
    ros::spin(); // Spin ROS node to handle callbacks

    return 0;
}
