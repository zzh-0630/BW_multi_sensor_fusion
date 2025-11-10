#include "feature_manager.h"

/**
 * @brief Get the last frame index this feature is observed
 * @return Last frame index = start_frame + number of observations - 1
 */
int FeaturePerId::endFrame()
{
    return start_frame + feature_per_frame.size() - 1;
}

/**
 * @brief Constructor: initialize with rotation matrices of IMU frames, set ric to identity
 * @param _Rs Array of rotation matrices from IMU to world frame
 */
FeatureManager::FeatureManager(Matrix3d _Rs[])
    : Rs(_Rs)
{
    for (int i = 0; i < NUM_OF_CAM; i++)
        ric[i].setIdentity();
}

/**
 * @brief Set camera extrinsic rotation (ric: rotation from camera to IMU)
 * @param _ric Array of camera-IMU rotation matrices
 */
void FeatureManager::setRic(Matrix3d _ric[])
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ric[i] = _ric[i];
    }
}

/**
 * @brief Clear all feature data in the manager
 */
void FeatureManager::clearState()
{
    feature.clear();
}

/**
 * @brief Count valid features (tracked in >=2 frames and start frame within the window)
 * @return Number of valid features
 */
int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : feature)
    {
        it.used_num = it.feature_per_frame.size();  // Update used_num with total observations

        // Valid if tracked in >=2 frames and start frame is not too close to window end
        if (it.used_num >= 2 && it.start_frame < WINDOW_SIZE - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

/**
 * @brief Add new features from current frame and check parallax for keyframe decision
 * @param frame_count Current frame index in window
 * @param image New features: {feature ID -> {camera ID, 7D info [x,y,z,u,v,vx,vy]}}
 * @param td Time offset between camera and IMU
 * @return True if parallax is sufficient (or not enough frames), else False
 */
bool FeatureManager::addFeatureCheckParallax(int frame_count, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, double td)
{
    ROS_DEBUG("input feature: %d", (int)image.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());
    double parallax_sum = 0;  // Sum of parallax for valid features
    int parallax_num = 0;     // Number of features used for parallax calculation
    last_track_num = 0;       // Number of features tracked from previous frame

    for (auto &id_pts : image)
    {
        // Create FeaturePerFrame from the first observation (assuming single camera)
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);

        int feature_id = id_pts.first;
        // Find if this feature ID is already tracked
        auto it = find_if(feature.begin(), feature.end(), [feature_id](const FeaturePerId &it)
                          {
                              return it.feature_id == feature_id;
                          });

        if (it == feature.end())
        {
            // New feature: add to list with current frame as start frame
            feature.push_back(FeaturePerId(feature_id, frame_count));
            feature.back().feature_per_frame.push_back(f_per_fra);
        }
        else if (it->feature_id == feature_id)
        {
            // Existing feature: add new observation, increment track count
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
        }
    }

    // No parallax check if frame count <2 or too few tracked features
    if (frame_count < 2 || last_track_num < 20)
        return true;

    // Calculate parallax for features observed in the last two frames
    for (auto &it_per_id : feature)
    {
        // Check if the feature is observed in frame_count-2 and frame_count-1
        if (it_per_id.start_frame <= frame_count - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frame_count - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frame_count);
            parallax_num++;
        }
    }

    if (parallax_num == 0)
    {
        return true;  // No valid features for parallax, treat as sufficient
    }
    else
    {
        // Check if average parallax meets the threshold
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= MIN_PARALLAX;
    }
}

/**
 * @brief Print feature debug info (ID, used count, start frame, observations)
 */
void FeatureManager::debugShow()
{
    ROS_DEBUG("debug show");
    for (auto &it : feature)
    {
        ROS_ASSERT(it.feature_per_frame.size() != 0);
        ROS_ASSERT(it.start_frame >= 0);
        ROS_ASSERT(it.used_num >= 0);

        ROS_DEBUG("%d,%d,%d ", it.feature_id, it.used_num, it.start_frame);
        int sum = 0;
        for (auto &j : it.feature_per_frame)
        {
            ROS_DEBUG("%d,", int(j.is_used));
            sum += j.is_used;
            printf("(%lf,%lf) ",j.point(0), j.point(1));  // Print 2D coordinates
        }
        ROS_ASSERT(it.used_num == sum);  // Verify used_num matches sum of is_used
    }
}

/**
 * @brief Get corresponding 3D features between two frames
 * @param frame_count_l Left frame index
 * @param frame_count_r Right frame index
 * @return Pairs of (3D point in left frame, 3D point in right frame)
 */
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    for (auto &it : feature)
    {
        // Check if the feature is observed in both frames
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            // Calculate indices in feature_per_frame for the two frames
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;  // 3D point in left frame
            b = it.feature_per_frame[idx_r].point;  // 3D point in right frame
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

/**
 * @brief Set feature depths from optimization results (inverse depth)
 * @param x Optimization output vector (inverse depths)
 */
void FeatureManager::setDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // Skip features with insufficient observations or out of window
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        // Convert inverse depth to depth (x stores inverse depth)
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
        // Update solve_flag based on depth validity
        if (it_per_id.estimated_depth < 0)
        {
            it_per_id.solve_flag = 2;  // Invalid depth (fail)
        }
        else
            it_per_id.solve_flag = 1;  // Valid depth (success)
    }
}

/**
 * @brief Remove features with failed depth estimation (solve_flag == 2)
 */
void FeatureManager::removeFailures()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        if (it->solve_flag == 2)
            feature.erase(it);  // Erase features with failed depth estimation
    }
}

/**
 * @brief Clear and reset feature depths from optimization results (without updating solve_flag)
 * @param x Optimization output vector (inverse depths)
 */
void FeatureManager::clearDepth(const VectorXd &x)
{
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        // Reset depth (used for re-initialization)
        it_per_id.estimated_depth = 1.0 / x(++feature_index);
    }
}

/**
 * @brief Get vector of inverse depths for all valid features
 * @return Vector of inverse depths
 */
VectorXd FeatureManager::getDepthVector()
{
    VectorXd dep_vec(getFeatureCount());  // Initialize vector with valid feature count
    int feature_index = -1;
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
#if 1
        dep_vec(++feature_index) = 1. / it_per_id.estimated_depth;  // Store inverse depth
#else
        dep_vec(++feature_index) = it_per_id.estimated_depth;  // (Alternative: store depth directly)
#endif
    }
    return dep_vec;
}

/**
 * @brief Triangulate 3D positions of features using multiple observations in the window
 * @param Ps Array of IMU positions in world frame (for each window frame)
 * @param tic Array of camera-IMU translation vectors
 * @param ric Array of camera-IMU rotation matrices
 */
void FeatureManager::triangulate(Vector3d Ps[], Vector3d tic[], Matrix3d ric[])
{
    for (auto &it_per_id : feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // Skip features with sufficient observations but uninitialized depth
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;

        if (it_per_id.estimated_depth > 0)
            continue;  // Depth already initialized

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;  // Start from initial frame

        ROS_ASSERT(NUM_OF_CAM == 1);  // Only support single camera in current implementation
        // SVD matrix for triangulation: 2*N rows (N=number of observations)
        Eigen::MatrixXd svd_A(2 * it_per_id.feature_per_frame.size(), 4);
        int svd_idx = 0;

        // Camera projection matrix for the first observation frame (P0)
        Eigen::Matrix3d R0 = Rs[imu_i] * ric[0];  // Camera rotation in world frame
        Eigen::Vector3d t0 = Ps[imu_i] + Rs[imu_i] * tic[0];  // Camera position in world frame
        Eigen::Matrix<double, 3, 4> P0;
        P0.leftCols<3>() = Eigen::Matrix3d::Identity();  // Simplified for first frame
        P0.rightCols<1>() = Eigen::Vector3d::Zero();

        // Add projection constraints from all observed frames
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;  // Current frame index

            // Camera pose in world frame for current frame
            Eigen::Vector3d t1 = Ps[imu_j] + Rs[imu_j] * tic[0];
            Eigen::Matrix3d R1 = Rs[imu_j] * ric[0];
            // Transform to first frame's coordinate system
            Eigen::Vector3d t = R0.transpose() * (t1 - t0);
            Eigen::Matrix3d R = R0.transpose() * R1;
            // Projection matrix for current frame (in first frame's coordinate)
            Eigen::Matrix<double, 3, 4> P;
            P.leftCols<3>() = R.transpose();
            P.rightCols<1>() = -R.transpose() * t;

            // Normalized feature vector (from image plane)
            Eigen::Vector3d f = it_per_frame.point.normalized();
            // Add two rows to SVD matrix (from projection constraints)
            svd_A.row(svd_idx++) = f[0] * P.row(2) - f[2] * P.row(0);
            svd_A.row(svd_idx++) = f[1] * P.row(2) - f[2] * P.row(1);

            if (imu_i == imu_j)
                continue;  // Skip same frame
        }

        ROS_ASSERT(svd_idx == svd_A.rows());  // Verify SVD matrix size
        // Solve for 3D point using SVD (rightmost singular vector)
        Eigen::Vector4d svd_V = Eigen::JacobiSVD<Eigen::MatrixXd>(svd_A, Eigen::ComputeThinV).matrixV().rightCols<1>();
        double depth = svd_V[2] / svd_V[3];  // Depth in first frame's coordinate

        it_per_id.estimated_depth = depth;
        // Clamp minimum depth to avoid invalid values
        if (it_per_id.estimated_depth < 0.1)
        {
            it_per_id.estimated_depth = INIT_DEPTH;  // Use initial depth if too small
        }
    }
}

/**
 * @brief Remove outlier features (placeholder with ROS_BREAK)
 */
void FeatureManager::removeOutlier()
{
    ROS_BREAK();  // Not implemented in current version
    int i = -1;
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;
        i += it->used_num != 0;
        if (it->used_num != 0 && it->is_outlier == true)
        {
            feature.erase(it);
        }
    }
}

/**
 * @brief Remove the oldest frame and adjust feature depths after marginalization
 * @param marg_R Rotation of marginalized frame (IMU to world)
 * @param marg_P Position of marginalized frame (IMU in world)
 * @param new_R Rotation of new first frame (IMU to world)
 * @param new_P Position of new first frame (IMU in world)
 */
void FeatureManager::removeBackShiftDepth(Eigen::Matrix3d marg_R, Eigen::Vector3d marg_P, Eigen::Matrix3d new_R, Eigen::Vector3d new_P)
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;  // Shift frame index for features not in the oldest frame
        }
        else
        {
            // Remove observation in the oldest frame
            Eigen::Vector3d uv_i = it->feature_per_frame[0].point;  
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            // Erase feature if no observations left
            if (it->feature_per_frame.size() < 2)
            {
                feature.erase(it);
                continue;
            }
            else
            {
                // Transform depth to new coordinate system (after marginalization)
                Eigen::Vector3d pts_i = uv_i * it->estimated_depth;  // 3D point in old frame
                Eigen::Vector3d w_pts_i = marg_R * pts_i + marg_P;  // Transform to world
                Eigen::Vector3d pts_j = new_R.transpose() * (w_pts_i - new_P);  // Transform to new frame
                double dep_j = pts_j(2);  // Depth in new frame
                if (dep_j > 0)
                    it->estimated_depth = dep_j;
                else
                    it->estimated_depth = INIT_DEPTH;  // Use initial depth if invalid
            }
        }
    }
}

/**
 * @brief Remove the oldest frame (simplified, no depth adjustment)
 */
void FeatureManager::removeBack()
{
    for (auto it = feature.begin(), it_next = feature.begin();
         it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame != 0)
        {
            it->start_frame--;  // Shift frame index
        }
        else
        {
            // Remove observation in the oldest frame
            it->feature_per_frame.erase(it->feature_per_frame.begin());
            // Erase feature if no observations left
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

/**
 * @brief Remove the front (latest) frame from the window
 * @param frame_count Current frame index
 */
void FeatureManager::removeFront(int frame_count)
{
    for (auto it = feature.begin(), it_next = feature.begin(); it != feature.end(); it = it_next)
    {
        it_next++;

        if (it->start_frame == frame_count)
        {
            it->start_frame--;  // Shift frame index
        }
        else
        {
            // Calculate index of the front frame in feature_per_frame
            int j = WINDOW_SIZE - 1 - it->start_frame;
            if (it->endFrame() < frame_count - 1)
                continue;  // No observation in the front frame
            // Remove observation in the front frame
            it->feature_per_frame.erase(it->feature_per_frame.begin() + j);
            // Erase feature if no observations left
            if (it->feature_per_frame.size() == 0)
                feature.erase(it);
        }
    }
}

/**
 * @brief Calculate compensated parallax between two consecutive frames
 * @param it_per_id Feature to calculate parallax
 * @param frame_count Current frame index
 * @return Maximum of original and compensated parallax
 */
double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    // Get observations in frame_count-2 and frame_count-1
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;  // 3D point in frame j

    // Image coordinates in frame j (normalized by depth)
    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;  // 3D point in frame i
    Vector3d p_i_comp = p_i;       // Compensated point (simplified, no motion compensation)

    // Image coordinates in frame i (original and compensated)
    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;  // Parallax without compensation

    double dep_i_comp = p_i_comp(2);
    double u_i_comp = p_i_comp(0) / dep_i_comp;
    double v_i_comp = p_i_comp(1) / dep_i_comp;
    double du_comp = u_i_comp - u_j, dv_comp = v_i_comp - v_j;  // Compensated parallax

    // Take the maximum of the two parallax values (squared)
    ans = max(ans, sqrt(min(du * du + dv * dv, du_comp * du_comp + dv_comp * dv_comp)));

    return ans;
}