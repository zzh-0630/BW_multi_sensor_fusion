#include "feature_tracker.h"
#include "image_preprocess/image_converter.h"

// Static variable to assign unique IDs to new features
int FeatureTracker::n_id = 0;

/**
 * @brief Check if a point is within the image border (avoid boundary effects)
 * @param pt The point to check (in image coordinates)
 * @return True if the point is inside the border, false otherwise
 */
bool inBorder(const cv::Point2f &pt)
{
    const int BORDER_SIZE = 1;
    int img_x = cvRound(pt.x);
    int img_y = cvRound(pt.y);
    // Check if the point is within [BORDER_SIZE, COL-BORDER_SIZE) and [BORDER_SIZE, ROW-BORDER_SIZE)
    return BORDER_SIZE <= img_x && img_x < COL - BORDER_SIZE && BORDER_SIZE <= img_y && img_y < ROW - BORDER_SIZE;
}

/**
 * @brief Reduce a vector of points by keeping only those with valid status
 * @param v The vector of points to reduce
 * @param status The status vector (1 for valid, 0 for invalid)
 */
void reduceVector(vector<cv::Point2f> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i]) // Keep only points with valid status
            v[j++] = v[i];
    v.resize(j); // Resize to the number of valid points
}

/**
 * @brief Overload: Reduce a vector of integers by keeping only those with valid status
 * @param v The vector of integers to reduce
 * @param status The status vector (1 for valid, 0 for invalid)
 */
void reduceVector(vector<int> &v, vector<uchar> status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (status[i]) // Keep only integers with valid status
            v[j++] = v[i];
    v.resize(j); // Resize to the number of valid elements
}

/**
 * @brief Constructor of FeatureTracker
 */
FeatureTracker::FeatureTracker()
{
}

/**
 * @brief Set a mask to avoid detecting new features too close to existing ones
 *                Prioritize keeping features that have been tracked for a long time
 */
void FeatureTracker::setMask()
{
    // Initialize mask: use fisheye mask if fisheye camera, else a full white mask (255 means valid region)
    if(FISHEYE)
        mask = fisheye_mask.clone();
    else
        mask = cv::Mat(ROW, COL, CV_8UC1, cv::Scalar(255));
    
    // Store (track count, (point, ID)) to sort by track duration (longer first)
    // prefer to keep features that are tracked for long time
    vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;

    for (unsigned int i = 0; i < forw_pts.size(); i++)
        cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(forw_pts[i], ids[i])));

    // Sort in descending order of track count (prioritize long-tracked features)
    sort(cnt_pts_id.begin(), cnt_pts_id.end(), [](const pair<int, pair<cv::Point2f, int>> &a, const pair<int, pair<cv::Point2f, int>> &b)
         {
            return a.first > b.first;
         });

    // Clear current containers to re-fill with sorted features
    forw_pts.clear();
    ids.clear();
    track_cnt.clear();

    // Re-insert features, marking their regions in the mask to avoid overlap
    for (auto &it : cnt_pts_id)
    {
        // Check if the point's region in the mask is still valid (255)
        if (mask.at<uchar>(it.second.first) == 255)
        {
            forw_pts.push_back(it.second.first);
            ids.push_back(it.second.second);
            track_cnt.push_back(it.first);
            // Draw a circle in the mask to block nearby regions (radius = MIN_DIST)
            cv::circle(mask, it.second.first, MIN_DIST, 0, -1);
        }
    }
}

/**
 * @brief Add newly detected features to the tracking list
 */
void FeatureTracker::addPoints()
{
    for (auto &p : n_pts) // n_pts: newly detected features
    {
        forw_pts.push_back(p); // Add to current frame's feature list
        ids.push_back(-1); // Temporarily assign -1 (will be updated to unique ID later)
        track_cnt.push_back(1); // Initialize track count to 1 (just detected)
    }
}

/**
 * @brief Main function to process input image, track features, and manage feature lifecycle
 * @param _img Input image
 * @param _cur_time Timestamp of the current image
 * @param encoding Encoding of the input image
 */
void FeatureTracker::readImage(const cv::Mat &_img, double _cur_time, const std::string& encoding)
{
    // Convert input image to grayscale
    cv::Mat gray_img = convertToGray(_img, encoding);
    cv::Mat img = gray_img;
    TicToc t_r; // Timer for debugging
    cur_time = _cur_time;
    if (img.empty()) {
        ROS_ERROR("Failed to convert image to gray");
        return;
    }

    // Apply CLAHE for contrast enhancement if enabled
    if (EQUALIZE)
    {
        cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8)); // Clip limit = 3.0, tile size = 8x8
        TicToc t_c;
        clahe->apply(_img, img); // Apply CLAHE to the image
        ROS_DEBUG("CLAHE costs: %fms", t_c.toc());
    }
    else
        img = _img;

    // Initialize image buffers for the first frame
    if (forw_img.empty())
    {
        prev_img = cur_img = forw_img = img;
    }
    else
    {
        forw_img = img; // Update forward image (current frame to process)
    }

    forw_pts.clear(); // Clear forward points (will be filled by optical flow)

    // Track features using Lucas-Kanade optical flow if there are previous features
    if (cur_pts.size() > 0)
    {
        TicToc t_o;
        vector<uchar> status; // Status of tracking (1: success, 0: failure)
        vector<float> err; // Tracking error
        // Compute optical flow from current image to forward image
        cv::calcOpticalFlowPyrLK(cur_img, forw_img, cur_pts, forw_pts, status, err, cv::Size(21, 21), 3);

        // Mark points outside the border as invalid
        for (int i = 0; i < int(forw_pts.size()); i++)
            if (status[i] && !inBorder(forw_pts[i]))
                status[i] = 0;

        // Reduce all feature-related vectors using the tracking status
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(ids, status);
        reduceVector(cur_un_pts, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("temporal optical flow costs: %fms", t_o.toc());
    }

    // Increment track count for all surviving features
    for (auto &n : track_cnt)
        n++;

    // If current frame is set to be published (PUB_THIS_FRAME flag)
    if (PUB_THIS_FRAME)
    {
        // Reject outliers using Fundamental Matrix RANSAC
        rejectWithF();
        ROS_DEBUG("set mask begins");
        TicToc t_m;
        setMask(); // Set mask to avoid dense features
        ROS_DEBUG("set mask costs %fms", t_m.toc());

        // Detect new features if current count is less than MAX_CNT
        ROS_DEBUG("detect feature begins");
        TicToc t_t;
        int n_max_cnt = MAX_CNT - static_cast<int>(forw_pts.size()); // Number of new features needed
        if (n_max_cnt > 0)
        {
            // Check mask validity (debug messages)
            if(mask.empty())
                cout << "mask is empty " << endl;
            if (mask.type() != CV_8UC1)
                cout << "mask type wrong " << endl;
            if (mask.size() != forw_img.size())
                cout << "wrong size " << endl;
            // Detect new features in valid mask regions
            cv::goodFeaturesToTrack(forw_img, n_pts, MAX_CNT - forw_pts.size(), 0.01, MIN_DIST, mask);
        }
        else
            n_pts.clear(); // No need to detect new features
        ROS_DEBUG("detect feature costs: %fms", t_t.toc());

        // Add newly detected features to the tracking list
        ROS_DEBUG("add feature begins");
        TicToc t_a;
        addPoints();
        ROS_DEBUG("selectFeature costs: %fms", t_a.toc());
    }

    // Update image and feature buffers for next iteration
    prev_img = cur_img;
    prev_pts = cur_pts;
    prev_un_pts = cur_un_pts;
    cur_img = forw_img;
    cur_pts = forw_pts;
    undistortedPoints(); // Undistort current features and compute velocity
    prev_time = cur_time;
}

/**
 * @brief Reject outlier feature matches using Fundamental Matrix RANSAC
 */
void FeatureTracker::rejectWithF()
{
    // Need at least 8 points to compute Fundamental Matrix
    if (forw_pts.size() >= 8)
    {
        ROS_DEBUG("FM ransac begins");
        TicToc t_f;
        // Store undistorted points (normalized plane) for current and forward frames
        vector<cv::Point2f> un_cur_pts(cur_pts.size()), un_forw_pts(forw_pts.size());
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            Eigen::Vector3d tmp_p;
            // Lift image point to normalized plane using camera model
            m_camera->liftProjective(Eigen::Vector2d(cur_pts[i].x, cur_pts[i].y), tmp_p);
            // Convert normalized coordinates back to image plane for FM computation
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_cur_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());

            // Same for forward points
            m_camera->liftProjective(Eigen::Vector2d(forw_pts[i].x, forw_pts[i].y), tmp_p);
            tmp_p.x() = FOCAL_LENGTH * tmp_p.x() / tmp_p.z() + COL / 2.0;
            tmp_p.y() = FOCAL_LENGTH * tmp_p.y() / tmp_p.z() + ROW / 2.0;
            un_forw_pts[i] = cv::Point2f(tmp_p.x(), tmp_p.y());
        }

        // Compute Fundamental Matrix with RANSAC to get inlier status
        vector<uchar> status;
        cv::findFundamentalMat(un_cur_pts, un_forw_pts, cv::FM_RANSAC, F_THRESHOLD, 0.99, status);
        int size_a = cur_pts.size();
        // Reduce feature lists using inlier status
        reduceVector(prev_pts, status);
        reduceVector(cur_pts, status);
        reduceVector(forw_pts, status);
        reduceVector(cur_un_pts, status);
        reduceVector(ids, status);
        reduceVector(track_cnt, status);
        ROS_DEBUG("FM ransac: %d -> %lu: %f", size_a, forw_pts.size(), 1.0 * forw_pts.size() / size_a);
        ROS_DEBUG("FM ransac costs: %fms", t_f.toc());
    }
}

/**
 * @brief Update the ID of a feature (assign unique ID to new features)
 * @param i Index of the feature in the list
 * @return True if the ID is updated successfully, false otherwise
 */
bool FeatureTracker::updateID(unsigned int i)
{
    if (i < ids.size())
    {
        if (ids[i] == -1) // Assign new ID to newly detected features (marked as -1)
            ids[i] = n_id++;
        return true;
    }
    else
        return false;
}

/**
 * @brief Read camera intrinsic parameters from a calibration file
 * @param calib_file Path to the calibration YAML file
 */
void FeatureTracker::readIntrinsicParameter(const string &calib_file)
{
    ROS_INFO("reading paramerter of camera %s", calib_file.c_str());
    m_camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file);
}

/**
 * @brief Visualize undistorted points (for debugging/verification)
 * @param name Window name for display
 */
void FeatureTracker::showUndistortion(const string &name)
{
    // Create a large image to display undistorted points (with offset 300)
    cv::Mat undistortedImg(ROW + 600, COL + 600, CV_8UC1, cv::Scalar(0));
    vector<Eigen::Vector2d> distortedp, undistortedp;
    // Iterate over all pixels in the original image
    for (int i = 0; i < COL; i++)
        for (int j = 0; j < ROW; j++)
        {
            Eigen::Vector2d a(i, j); // Distorted pixel coordinate
            Eigen::Vector3d b;
            m_camera->liftProjective(a, b); // Undistort to normalized plane
            distortedp.push_back(a);
            undistortedp.push_back(Eigen::Vector2d(b.x() / b.z(), b.y() / b.z()));
            //printf("%f,%f->%f,%f,%f\n)\n", a.x(), a.y(), b.x(), b.y(), b.z());
        }

    // Map undistorted points back to image plane and draw on the display image
    for (int i = 0; i < int(undistortedp.size()); i++)
    {
        cv::Mat pp(3, 1, CV_32FC1);
        pp.at<float>(0, 0) = undistortedp[i].x() * FOCAL_LENGTH + COL / 2;
        pp.at<float>(1, 0) = undistortedp[i].y() * FOCAL_LENGTH + ROW / 2;
        pp.at<float>(2, 0) = 1.0;
        // Draw the pixel if it's within the display image bounds
        //cout << trackerData[0].K << endl;
        //printf("%lf %lf\n", p.at<float>(1, 0), p.at<float>(0, 0));
        //printf("%lf %lf\n", pp.at<float>(1, 0), pp.at<float>(0, 0));
        if (pp.at<float>(1, 0) + 300 >= 0 && pp.at<float>(1, 0) + 300 < ROW + 600 && pp.at<float>(0, 0) + 300 >= 0 && pp.at<float>(0, 0) + 300 < COL + 600)
        {
            undistortedImg.at<uchar>(pp.at<float>(1, 0) + 300, pp.at<float>(0, 0) + 300) = cur_img.at<uchar>(distortedp[i].y(), distortedp[i].x());
        }
        else
        {
            //ROS_ERROR("(%f %f) -> (%f %f)", distortedp[i].y, distortedp[i].x, pp.at<float>(1, 0), pp.at<float>(0, 0));
        }
    }
    cv::imshow(name, undistortedImg);
    cv::waitKey(0);
}

/**
 * @brief Undistort current features to normalized plane and compute their velocity
 */
void FeatureTracker::undistortedPoints()
{
    cur_un_pts.clear();
    cur_un_pts_map.clear();
    // Undistort each current feature point using camera model
    //cv::undistortPoints(cur_pts, un_pts, K, cv::Mat());
    for (unsigned int i = 0; i < cur_pts.size(); i++)
    {
        Eigen::Vector2d a(cur_pts[i].x, cur_pts[i].y); // Distorted coordinate
        Eigen::Vector3d b;
        m_camera->liftProjective(a, b); // Lift to normalized 3D direction
        // Store undistorted 2D point (x/z, y/z) in normalized plane
        cur_un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
        // Map feature ID to its undistorted point for velocity calculation
        cur_un_pts_map.insert(make_pair(ids[i], cv::Point2f(b.x() / b.z(), b.y() / b.z())));
        //printf("cur pts id %d %f %f", ids[i], cur_un_pts[i].x, cur_un_pts[i].y);
    }
    // Calculate velocity of features using previous undistorted points
    if (!prev_un_pts_map.empty())
    {
        double dt = cur_time - prev_time; // Time difference between current and previous frame
        pts_velocity.clear();
        for (unsigned int i = 0; i < cur_un_pts.size(); i++)
        {
            if (ids[i] != -1) // Only for features with valid ID
            {
                std::map<int, cv::Point2f>::iterator it;
                it = prev_un_pts_map.find(ids[i]); // Find previous position of the same feature
                if (it != prev_un_pts_map.end())
                {
                    // Velocity = (current position - previous position) / time difference
                    double v_x = (cur_un_pts[i].x - it->second.x) / dt;
                    double v_y = (cur_un_pts[i].y - it->second.y) / dt;
                    pts_velocity.push_back(cv::Point2f(v_x, v_y));
                }
                else
                    pts_velocity.push_back(cv::Point2f(0, 0)); // No previous data: velocity 0
            }
            else
            {
                pts_velocity.push_back(cv::Point2f(0, 0)); // New feature: velocity 0
            }
        }
    }
    else
    {
        // No previous frame: initialize velocity to 0 for all features
        for (unsigned int i = 0; i < cur_pts.size(); i++)
        {
            pts_velocity.push_back(cv::Point2f(0, 0));
        }
    }
    // Update previous undistorted points map for next iteration
    prev_un_pts_map = cur_un_pts_map;
}
