#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <std_msgs/Float64.h>

#include "feature_tracker.h"

// Macro to control whether to show undistorted images (0: disable, 1: enable)
#define SHOW_UNDISTORTION 0

// Callback function declaration for time delay (td) messages
void td_callback(const std_msgs::Float64::ConstPtr& msg);
// Variables to store tracking status and errors (used in feature tracking)
vector<uchar> r_status;
vector<float> r_err;
// Buffer to store incoming image messages
queue<sensor_msgs::ImageConstPtr> img_buf;

// ROS publishers: publish feature points, matched images, and restart signals
ros::Publisher pub_img,pub_match;
ros::Publisher pub_restart;

// New publisher: publish time-aligned grayscale images
ros::Publisher pub_aligned_image;
// ROS subscriber: subscribe to time delay (td) messages
ros::Subscriber sub_td;

// Feature tracker instances for each camera (NUM_OF_CAM is defined in feature_tracker.h)
FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time; // Time of the first received image
int pub_count = 1; // Counter for published frames
bool first_image_flag = true; // Flag to indicate if the first image has been received
double last_image_time = 0; // Time of the last received image (for checking continuity)
bool init_pub = 0; // Flag to indicate if initial publication is done
double current_td = 0.0; // Current time delay (td) between camera and other sensors (e.g., IMU)

/**
 * @brief Callback function for time delay (td) messages
 * @param msg Pointer to the Float64 message containing the time delay
 * Updates the current time delay used for timestamp alignment
 */
void td_callback(const std_msgs::Float64::ConstPtr& msg)
{
    current_td = msg->data;
    ROS_DEBUG("Updated time delay: %f", current_td);
}

/**
 * @brief Callback function for incoming image messages
 * @param img_msg Pointer to the Image message
 * Processes the image, performs feature tracking, and publishes results
 */
void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    // Initialize time for the first received image
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = img_msg->header.stamp.toSec();
        last_image_time = img_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    // Check for discontinuous image stream (time jump >1s or time goes backward)
    if (img_msg->header.stamp.toSec() - last_image_time > 1.0 || img_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        // Publish restart signal to notify other nodes
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = img_msg->header.stamp.toSec();
    // frequency control
    // Control publication frequency to match the preset FREQ
    if (round(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // Reset frequency control if the actual frequency is close to FREQ
        if (abs(1.0 * pub_count / (img_msg->header.stamp.toSec() - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = img_msg->header.stamp.toSec();
            pub_count = 0;
        }
    }
    else
        PUB_THIS_FRAME = false; // Skip publishing for this frame

    // Convert ROS Image message to OpenCV Mat (handle encoding conversion if needed)
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        // Convert "8UC1" encoding to "mono8" for compatibility
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        // Directly convert to "mono8" if encoding is different
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image; // Image for visualization
    TicToc t_r; // Timer to measure processing time

    // Process each camera's image (split from the combined image if multiple cameras)
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_DEBUG("processing camera %d", i);
        // For non-stereo or first camera: read image and perform feature tracking
        if (i != 1 || !STEREO_TRACK)
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), img_msg->header.stamp.toSec(), img_msg->encoding);
        else
        {
            // For stereo camera (second camera), apply histogram equalization if enabled
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_img);
            }
            else
                trackerData[i].cur_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

#if SHOW_UNDISTORTION
        // Show undistorted images for debugging (if enabled)
        trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
#endif
    }

    // Update feature IDs across all cameras (ensure consistent ID assignment)
    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break; // Exit loop when all IDs are updated
    }

    // Publish results if current frame is marked for publication
   if (PUB_THIS_FRAME)
   {
        pub_count++; // Increment publication counter

        // Prepare PointCloud message to store feature points and their attributes
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point; // Feature IDs (with camera index)
        sensor_msgs::ChannelFloat32 u_of_point; // Pixel x-coordinates
        sensor_msgs::ChannelFloat32 v_of_point; // Pixel y-coordinates
        sensor_msgs::ChannelFloat32 velocity_x_of_point; // x-velocity of features
        sensor_msgs::ChannelFloat32 velocity_y_of_point; // y-velocity of features

        feature_points->header = img_msg->header;
        feature_points->header.frame_id = "world"; // Coordinate frame: world

        // Collect valid features (track count > 1) from each camera
        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_un_pts; // Undistorted feature points
            auto &cur_pts = trackerData[i].cur_pts; // Distorted feature points (pixel coordinates)
            auto &ids = trackerData[i].ids; // Feature IDs
            auto &pts_velocity = trackerData[i].pts_velocity; // Feature velocities
            for (unsigned int j = 0; j < ids.size(); j++)
            {
                // Only publish features tracked for more than 1 frame (stable tracking)
                if (trackerData[i].track_cnt[j] > 1)
                {
                    int p_id = ids[j];
                    hash_ids[i].insert(p_id);
                    // Add 2D undistorted point (homogeneous coordinate: z=1)
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i); // Encode camera index in ID
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        // Add attribute channels to the point cloud
        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);
        ROS_DEBUG("publish %f, at %f", feature_points->header.stamp.toSec(), ros::Time::now().toSec());
        // skip the first image; since no optical speed on frist image
        // Skip publishing the first frame (no velocity info available)
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_img.publish(feature_points); // Publish feature points

        // Publish visualization image if tracking visualization is enabled
        if (SHOW_TRACK)
        {
            // Convert grayscale image to BGR for color drawing
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::BGR8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image; // Image for drawing tracks

            // Draw features on each camera's image region
            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB); // Convert to RGB for color markers

                // Draw each feature with color based on tracking duration (longer track = more red)
                for (unsigned int j = 0; j < trackerData[i].cur_pts.size(); j++)
                {
                    double len = std::min(1.0, 1.0 * trackerData[i].track_cnt[j] / WINDOW_SIZE);
                    cv::circle(tmp_img, trackerData[i].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
                }
                // Publish the visualization image
                pub_match.publish(ptr->toImageMsg());
            }

            // Publish time-aligned grayscale image (compensated with current_td)
            try {
                // Get the original grayscale image without processing
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        
                // Create a new image message with timestamp compensated by current_td
                sensor_msgs::ImagePtr aligned_image_msg = cv_bridge::CvImage(
                    img_msg->header, 
                    "mono8", 
                    cv_ptr->image
                ).toImageMsg();
        
                // Adjust timestamp: subtract current time delay (td) for synchronization
                aligned_image_msg->header.stamp = ros::Time(img_msg->header.stamp.toSec() - current_td);
        
                // Publish the time-aligned image
                pub_aligned_image.publish(aligned_image_msg);
            } 
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }
        }
        ROS_INFO("whole feature tracker processing costs: %f", t_r.toc());
    }
}

/**
 * @brief Main function of the feature tracker node
 * Initializes ROS, reads parameters, sets up subscribers/publishers, and starts spinning
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker"); // Initialize ROS node
    ros::NodeHandle n("~"); // Private node handle
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info); // Set log level
    readParameters(n); // Read configuration parameters (from launch file or parameter server)

    // Initialize each camera's feature tracker with intrinsic parameters
    for (int i = 0; i < NUM_OF_CAM; i++)
        trackerData[i].readIntrinsicParameter(CAM_NAMES[i]);

    // Load fisheye mask if fisheye camera is used
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_INFO("load mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }

    // Subscribe to image messages (topic defined by IMAGE_TOPIC)
    ros::Subscriber sub_img = n.subscribe(IMAGE_TOPIC, 100, img_callback);

    // Advertise publishers
    pub_img = n.advertise<sensor_msgs::PointCloud>("feature", 1000); // Published feature points
    pub_match = n.advertise<sensor_msgs::Image>("feature_img",1000); // Published visualization image
    pub_restart = n.advertise<std_msgs::Bool>("restart",1000); // Published restart signal
    pub_aligned_image = n.advertise<sensor_msgs::Image>("/aligned_grayscale_image", 10);// Published time-aligned image
    sub_td = n.subscribe("/current_td", 10, td_callback); // Subscribed to time delay messages
    ros::spin(); // Start ROS event loop
    return 0;
}