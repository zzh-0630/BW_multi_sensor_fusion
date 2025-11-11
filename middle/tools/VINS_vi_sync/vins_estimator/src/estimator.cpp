#include "estimator.h"

/**
 * @brief Constructor: Initializes feature manager with rotation states (Rs) and starts initialization
 */
Estimator::Estimator(): f_manager{Rs}
{
    ROS_INFO("init begins");
    clearState();
}

/**
 * @brief Sets initial parameters: extrinsics, noise covariance for projection factors, time offset
 */
void Estimator::setParameter()
{
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i]; // Initialize translation extrinsics from config
        ric[i] = RIC[i]; // Initialize rotation extrinsics from config
    }
    f_manager.setRic(ric); // Pass rotation extrinsics to feature manager
    // Set noise covariance for projection factors (scaled by focal length)
    ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    ProjectionTdFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity();
    td = TD; // Initialize time offset from config
}

/**
 * @brief Clears all state variables to reset the system
 */
void Estimator::clearState()
{
    // Reset pose, velocity, bias, and IMU data buffers for all window frames
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        Rs[i].setIdentity();
        Ps[i].setZero();
        Vs[i].setZero();
        Bas[i].setZero();
        Bgs[i].setZero();
        dt_buf[i].clear();
        linear_acceleration_buf[i].clear();
        angular_velocity_buf[i].clear();

        // Delete and reinitialize pre-integrators
        if (pre_integrations[i] != nullptr)
            delete pre_integrations[i];
        pre_integrations[i] = nullptr;
    }

    // Reset camera-IMU extrinsics
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d::Zero();
        ric[i] = Matrix3d::Identity();
    }

    // Clear image frames and their pre-integrators
    for (auto &it : all_image_frame)
    {
        if (it.second.pre_integration != nullptr)
        {
            delete it.second.pre_integration;
            it.second.pre_integration = nullptr;
        }
    }

    // Reset flags and counters
    solver_flag = INITIAL;
    first_imu = false,
    sum_of_back = 0;
    sum_of_front = 0;
    frame_count = 0;
    solver_flag = INITIAL;
    initial_timestamp = 0;
    all_image_frame.clear();
    td = TD;

    // Clean up temporary and marginalization data
    if (tmp_pre_integration != nullptr)
        delete tmp_pre_integration;
    if (last_marginalization_info != nullptr)
        delete last_marginalization_info;

    tmp_pre_integration = nullptr;
    last_marginalization_info = nullptr;
    last_marginalization_parameter_blocks.clear();

    f_manager.clearState(); // Reset feature manager

    failure_occur = 0;
    relocalization_info = 0;

    // Reset drift correction
    drift_correct_r = Matrix3d::Identity();
    drift_correct_t = Vector3d::Zero();
}

/**
 * @brief Processes IMU data: updates pre-integrator and predicts current state
 * @param dt Time interval since last IMU data
 * @param linear_acceleration Linear acceleration measurement
 * @param angular_velocity Angular velocity measurement
 */
void Estimator::processIMU(double dt, const Vector3d &linear_acceleration, const Vector3d &angular_velocity)
{
    if (!first_imu)
    {
        first_imu = true;
        acc_0 = linear_acceleration; // Initialize first acceleration
        gyr_0 = angular_velocity; // Initialize first angular velocity
    }

    // Create pre-integrator for current frame if not exists
    if (!pre_integrations[frame_count])
    {
        pre_integrations[frame_count] = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};
    }
    // For non-first frames: update pre-integrator, buffer data, and predict state
    if (frame_count != 0)
    {
        pre_integrations[frame_count]->push_back(dt, linear_acceleration, angular_velocity);
        //if(solver_flag != NON_LINEAR)
            tmp_pre_integration->push_back(dt, linear_acceleration, angular_velocity);

        // Buffer IMU data
        dt_buf[frame_count].push_back(dt);
        linear_acceleration_buf[frame_count].push_back(linear_acceleration);
        angular_velocity_buf[frame_count].push_back(angular_velocity);

        // Predict pose and velocity using IMU motion model
        int j = frame_count;         
        Vector3d un_acc_0 = Rs[j] * (acc_0 - Bas[j]) - g; // Unbiased acceleration (world frame)
        Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - Bgs[j]; // Unbiased angular velocity
        Rs[j] *= Utility::deltaQ(un_gyr * dt).toRotationMatrix(); // Update rotation
        Vector3d un_acc_1 = Rs[j] * (linear_acceleration - Bas[j]) - g; // Updated unbiased acceleration
        Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1); // Average acceleration
        Ps[j] += dt * Vs[j] + 0.5 * dt * dt * un_acc; // Update position
        Vs[j] += dt * un_acc; // Update velocity
    }
    // Update previous IMU measurements for next iteration
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

/**
 * @brief Processes incoming image data
 * Adds features to manager, determines keyframe status, handles initialization, solves odometry,
 * and manages sliding window
 * @param image Map of camera IDs to feature points (ID + 7D vector: [u, v, z, u', v', z', velocity])
 * @param header ROS header containing timestamp
 */
void Estimator::processImage(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &image, const std_msgs::Header &header)
{
    ROS_DEBUG("new image coming ------------------------------------------");
    ROS_DEBUG("Adding feature points %lu", image.size());
    // Determine if current frame is keyframe based on parallax
    if (f_manager.addFeatureCheckParallax(frame_count, image, td))
        marginalization_flag = MARGIN_OLD; // Non-keyframe: marginalize old frame
    else
        marginalization_flag = MARGIN_SECOND_NEW; // Keyframe: marginalize second new frame

    ROS_DEBUG("this frame is--------------------%s", marginalization_flag ? "reject" : "accept");
    ROS_DEBUG("%s", marginalization_flag ? "Non-keyframe" : "Keyframe");
    ROS_DEBUG("Solving %d", frame_count);
    ROS_DEBUG("number of feature: %d", f_manager.getFeatureCount());
    Headers[frame_count] = header; // Store header

    // Create image frame with features and timestamp
    ImageFrame imageframe(image, header.stamp.toSec());
    imageframe.pre_integration = tmp_pre_integration; // Attach pre-integration data
    all_image_frame.insert(make_pair(header.stamp.toSec(), imageframe));
    // Create new temporary pre-integration object for next frame
    tmp_pre_integration = new IntegrationBase{acc_0, gyr_0, Bas[frame_count], Bgs[frame_count]};

    // Calibrate extrinsic rotation if enabled
    if(ESTIMATE_EXTRINSIC == 2)
    {
        ROS_INFO("calibrating extrinsic param, rotation movement is needed");
        if (frame_count != 0)
        {
            // Get corresponding features between current and previous frame
            vector<pair<Vector3d, Vector3d>> corres = f_manager.getCorresponding(frame_count - 1, frame_count);
            Matrix3d calib_ric;
            // Calibrate extrinsic rotation using corresponding features and IMU pre-integration
            if (initial_ex_rotation.CalibrationExRotation(corres, pre_integrations[frame_count]->delta_q, calib_ric))
            {
                ROS_WARN("initial extrinsic rotation calib success");
                ROS_WARN_STREAM("initial extrinsic rotation: " << endl << calib_ric);
                ric[0] = calib_ric;
                RIC[0] = calib_ric;
                ESTIMATE_EXTRINSIC = 1; // Switch to using calibrated extrinsics
            }
        }
    }

    // Handle initialization phase (before first window is filled)
    if (solver_flag == INITIAL)
    {
        if (frame_count == WINDOW_SIZE)
        {
            bool result = false;
            // Attempt initialization if extrinsic calibration is done and enough time has passed
            if( ESTIMATE_EXTRINSIC != 2 && (header.stamp.toSec() - initial_timestamp) > 0.1)
            {
               result = initialStructure(); // Initialize 3D structure and pose
               initial_timestamp = header.stamp.toSec();
            }
            if(result)
            {
                solver_flag = NON_LINEAR; // Switch to non-linear optimization
                solveOdometry(); // Solve odometry with initialized structure
                slideWindow(); // Slide window to make space for new frames
                f_manager.removeFailures(); // Remove failed features
                ROS_INFO("Initialization finish!");
                // Store last poses for failure detection
                last_R = Rs[WINDOW_SIZE];
                last_P = Ps[WINDOW_SIZE];
                last_R0 = Rs[0];
                last_P0 = Ps[0];
                
            }
            else
                slideWindow(); // Slide window even if initialization failed
        }
        else
            frame_count++; // Increment frame counter if window not full
    }
    else // Non-linear optimization phase
    {
        TicToc t_solve;
        solveOdometry(); // Solve odometry using non-linear optimization
        ROS_DEBUG("solver costs: %fms", t_solve.toc());

        // Check for system failures
        if (failureDetection())
        {
            ROS_WARN("failure detection!");
            failure_occur = 1;
            clearState(); // Reset system state
            setParameter(); // Reinitialize parameters
            ROS_WARN("system reboot!");
            return;
        }

        TicToc t_margin;
        slideWindow(); // Slide window after optimization
        f_manager.removeFailures(); // Clean up features
        ROS_DEBUG("marginalization costs: %fms", t_margin.toc());
        // Prepare key poses for output
        key_poses.clear();
        for (int i = 0; i <= WINDOW_SIZE; i++)
            key_poses.push_back(Ps[i]);

        // Update last poses
        last_R = Rs[WINDOW_SIZE];
        last_P = Ps[WINDOW_SIZE];
        last_R0 = Rs[0];
        last_P0 = Ps[0];
    }
}

/**
 * @brief Initializes 3D structure and camera poses using SFM and PnP
 * Performs global SFM, refines poses with PnP, and aligns visual structure with IMU
 * @return True if initialization succeeds, false otherwise
 */
bool Estimator::initialStructure()
{
    TicToc t_sfm;
    // Check IMU excitation (motion) to ensure observability
    {
        map<double, ImageFrame>::iterator frame_it;
        Vector3d sum_g;
        // Calculate average gravity direction from IMU data
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt; // Approx gravity from velocity change
            sum_g += tmp_g;
        }
        Vector3d aver_g;
        aver_g = sum_g * 1.0 / ((int)all_image_frame.size() - 1);
        double var = 0;
        // Calculate variance of gravity estimates
        for (frame_it = all_image_frame.begin(), frame_it++; frame_it != all_image_frame.end(); frame_it++)
        {
            double dt = frame_it->second.pre_integration->sum_dt;
            Vector3d tmp_g = frame_it->second.pre_integration->delta_v / dt;
            var += (tmp_g - aver_g).transpose() * (tmp_g - aver_g);
        }
        var = sqrt(var / ((int)all_image_frame.size() - 1));
        if(var < 0.25)
        {
            ROS_INFO("IMU excitation not enouth!");
        }
    }
    // Prepare data for global SFM
    Quaterniond Q[frame_count + 1];  // Camera poses (rotation)
    Vector3d T[frame_count + 1];     // Camera poses (translation)
    map<int, Vector3d> sfm_tracked_points;  // 3D points from SFM
    vector<SFMFeature> sfm_f;  // Features for SFM

    // Extract features and their observations across frames
    for (auto &it_per_id : f_manager.feature)
    {
        int imu_j = it_per_id.start_frame - 1;
        SFMFeature tmp_feature;
        tmp_feature.state = false;
        tmp_feature.id = it_per_id.feature_id;
        // Collect observations (frame index + 2D image point)
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            Vector3d pts_j = it_per_frame.point;
            tmp_feature.observation.push_back(make_pair(imu_j, Eigen::Vector2d{pts_j.x(), pts_j.y()}));
        }
        sfm_f.push_back(tmp_feature);
    } 

    // Find relative pose between two frames with sufficient parallax
    Matrix3d relative_R;
    Vector3d relative_T;
    int l; // Index of reference frame
    if (!relativePose(relative_R, relative_T, l))
    {
        ROS_INFO("Not enough features or parallax; Move device around");
        return false;
    }
    // Run global SFM to get initial poses and 3D points
    GlobalSFM sfm;
    if(!sfm.construct(frame_count + 1, Q, T, l,
              relative_R, relative_T,
              sfm_f, sfm_tracked_points))
    {
        ROS_DEBUG("global SFM failed!");
        marginalization_flag = MARGIN_OLD;
        return false;
    }

    //solve pnp for all frame
    // Refine all frame poses using PnP with SFM 3D points
    map<double, ImageFrame>::iterator frame_it;
    map<int, Vector3d>::iterator it;
    frame_it = all_image_frame.begin( );
    for (int i = 0; frame_it != all_image_frame.end( ); frame_it++)
    {
        // provide initial guess frome SFM
        cv::Mat r, rvec, t, D, tmp_r;
        if((frame_it->first) == Headers[i].stamp.toSec())
        {
            // Keyframe: use SFM pose directly
            frame_it->second.is_key_frame = true;
            frame_it->second.R = Q[i].toRotationMatrix() * RIC[0].transpose();
            frame_it->second.T = T[i];
            i++;
            continue;
        }
        if((frame_it->first) > Headers[i].stamp.toSec())
        {
            i++;
        }
        // Initial guess for non-keyframes
        Matrix3d R_inital = (Q[i].inverse()).toRotationMatrix();
        Vector3d P_inital = - R_inital * T[i];
        cv::eigen2cv(R_inital, tmp_r);
        cv::Rodrigues(tmp_r, rvec); // Convert rotation matrix to angle-axis
        cv::eigen2cv(P_inital, t);

        frame_it->second.is_key_frame = false;
        // Collect 3D-2D correspondences for PnP
        vector<cv::Point3f> pts_3_vector;
        vector<cv::Point2f> pts_2_vector;
        for (auto &id_pts : frame_it->second.points)
        {
            int feature_id = id_pts.first;
            for (auto &i_p : id_pts.second)
            {
                it = sfm_tracked_points.find(feature_id);
                if(it != sfm_tracked_points.end())
                {
                    // 3D point from SFM
                    Vector3d world_pts = it->second;
                    cv::Point3f pts_3(world_pts(0), world_pts(1), world_pts(2));
                    pts_3_vector.push_back(pts_3);
                    // 2D image point
                    Vector2d img_pts = i_p.second.head<2>();
                    cv::Point2f pts_2(img_pts(0), img_pts(1));
                    pts_2_vector.push_back(pts_2);
                }
            }
        }
        // Identity camera matrix (assuming normalized coordinates)
        cv::Mat K = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);     
        if(pts_3_vector.size() < 6) // Need at least 6 points for PnP
        {
            cout << "pts_3_vector size " << pts_3_vector.size() << endl;
            ROS_DEBUG("Not enough points for solve pnp !");
            return false;
        }
        // Solve PnP to refine pose
        if (! cv::solvePnP(pts_3_vector, pts_2_vector, K, D, rvec, t, 1))
        {
            ROS_DEBUG("solve pnp fail!");
            return false;
        }
        // Convert back to rotation matrix and update pose
        cv::Rodrigues(rvec, r);
        MatrixXd R_pnp,tmp_R_pnp;
        cv::cv2eigen(r, tmp_R_pnp);
        R_pnp = tmp_R_pnp.transpose();
        MatrixXd T_pnp;
        cv::cv2eigen(t, T_pnp);
        T_pnp = R_pnp * (-T_pnp);
        frame_it->second.R = R_pnp * RIC[0].transpose();
        frame_it->second.T = T_pnp;
    }

    // Align visual structure with IMU measurements
    if (visualInitialAlign())
        return true;
    else
    {
        ROS_INFO("misalign visual structure with IMU");
        return false;
    }

}

/**
 * @brief Aligns visual SFM results with IMU data
 * Solves for scale, gravity direction, and velocity, then updates system states
 * @return True if alignment succeeds, false otherwise
 */
bool Estimator::visualInitialAlign()
{
    TicToc t_g;
    VectorXd x;
    // Solve for scale, gravity, and velocities using visual-IMU alignment
    bool result = VisualIMUAlignment(all_image_frame, Bgs, g, x);
    if(!result)
    {
        ROS_DEBUG("solve g failed!");
        return false;
    }

    // change state
    // Update poses from aligned SFM results
    for (int i = 0; i <= frame_count; i++)
    {
        Matrix3d Ri = all_image_frame[Headers[i].stamp.toSec()].R;
        Vector3d Pi = all_image_frame[Headers[i].stamp.toSec()].T;
        Ps[i] = Pi;
        Rs[i] = Ri;
        all_image_frame[Headers[i].stamp.toSec()].is_key_frame = true;
    }

    // Reset feature depths for triangulation
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < dep.size(); i++)
        dep[i] = -1;
    f_manager.clearDepth(dep);

    //triangulat on cam pose , no tic
    // Triangulate features using camera poses (without translation extrinsics)
    Vector3d TIC_TMP[NUM_OF_CAM];
    for(int i = 0; i < NUM_OF_CAM; i++)
        TIC_TMP[i].setZero();
    ric[0] = RIC[0];
    f_manager.setRic(ric);
    f_manager.triangulate(Ps, &(TIC_TMP[0]), &(RIC[0]));

    // Apply scale to poses and repropagate pre-integrations
    double s = (x.tail<1>())(0); // Scale factor
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        pre_integrations[i]->repropagate(Vector3d::Zero(), Bgs[i]); // Repropagate with zero acceleration bias
    }
    // Adjust positions by scale and extrinsic offset
    for (int i = frame_count; i >= 0; i--)
        Ps[i] = s * Ps[i] - Rs[i] * TIC[0] - (s * Ps[0] - Rs[0] * TIC[0]);
    // Update velocities from alignment result
    int kv = -1;
    map<double, ImageFrame>::iterator frame_i;
    for (frame_i = all_image_frame.begin(); frame_i != all_image_frame.end(); frame_i++)
    {
        if(frame_i->second.is_key_frame)
        {
            kv++;
            Vs[kv] = frame_i->second.R * x.segment<3>(kv * 3); // Velocity from alignment
        }
    }
    // Scale feature depths
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        it_per_id.estimated_depth *= s;
    }

    // Align gravity direction with IMU (z-axis)
    Matrix3d R0 = Utility::g2R(g); // Rotation from gravity to world z-axis
    double yaw = Utility::R2ypr(R0 * Rs[0]).x(); // Yaw angle to align
    R0 = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * R0; // Remove yaw error
    g = R0 * g; // Update gravity vector
    Matrix3d rot_diff = R0; // Rotation to align coordinate system
    // Apply rotation to poses, velocities
    for (int i = 0; i <= frame_count; i++)
    {
        Ps[i] = rot_diff * Ps[i];
        Rs[i] = rot_diff * Rs[i];
        Vs[i] = rot_diff * Vs[i];
    }
    ROS_DEBUG_STREAM("g0     " << g.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(Rs[0]).transpose()); 

    return true;
}

/**
 * @brief Finds relative pose between two frames with sufficient parallax
 * @param relative_R Output: relative rotation between frames
 * @param relative_T Output: relative translation between frames
 * @param l Output: index of reference frame
 * @return True if valid relative pose is found, false otherwise
 */
bool Estimator::relativePose(Matrix3d &relative_R, Vector3d &relative_T, int &l)
{
    // find previous frame which contians enough correspondance and parallex with newest frame
    for (int i = 0; i < WINDOW_SIZE; i++) 
    {
        vector<pair<Vector3d, Vector3d>> corres;
        corres = f_manager.getCorresponding(i, WINDOW_SIZE);
        if (corres.size() > 20) // Need at least 20 correspondences
        {
            double sum_parallax = 0;
            double average_parallax;
            // Calculate average parallax between features
            for (int j = 0; j < int(corres.size()); j++)
            {
                Vector2d pts_0(corres[j].first(0), corres[j].first(1));
                Vector2d pts_1(corres[j].second(0), corres[j].second(1));
                double parallax = (pts_0 - pts_1).norm(); // Pixel parallax
                sum_parallax = sum_parallax + parallax;

            }
            average_parallax = 1.0 * sum_parallax / int(corres.size());
            // Check if parallax is sufficient and solve relative pose
            if(average_parallax * 460 > 30 && m_estimator.solveRelativeRT(corres, relative_R, relative_T))
            {
                l = i;
                ROS_DEBUG("average_parallax %f choose l %d and newest frame to triangulate the whole structure", average_parallax * 460, l);
                return true;
            }
        }
    }
    return false;
}

/**
 * @brief Solves odometry using non-linear optimization
 * Performs triangulation of features and runs optimization
 */
void Estimator::solveOdometry()
{
    if (frame_count < WINDOW_SIZE)
        return;
    if (solver_flag == NON_LINEAR)
    {
        TicToc t_tri;
        f_manager.triangulate(Ps, tic, ric); // Triangulate 3D points from current poses
        ROS_DEBUG("triangulation costs %f", t_tri.toc());
        optimization(); // Run non-linear optimization
    }
}

/**
 * @brief Converts state vectors to double arrays for Ceres optimization
 * Converts poses, velocities, biases, extrinsics, feature depths, and time offset
 * to double arrays compatible with Ceres solver
 */
void Estimator::vector2double()
{
    // Convert poses (position + rotation)
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        para_Pose[i][0] = Ps[i].x();
        para_Pose[i][1] = Ps[i].y();
        para_Pose[i][2] = Ps[i].z();
        Quaterniond q{Rs[i]};
        para_Pose[i][3] = q.x();
        para_Pose[i][4] = q.y();
        para_Pose[i][5] = q.z();
        para_Pose[i][6] = q.w();

        // Convert velocities and biases
        para_SpeedBias[i][0] = Vs[i].x();
        para_SpeedBias[i][1] = Vs[i].y();
        para_SpeedBias[i][2] = Vs[i].z();

        para_SpeedBias[i][3] = Bas[i].x();
        para_SpeedBias[i][4] = Bas[i].y();
        para_SpeedBias[i][5] = Bas[i].z();

        para_SpeedBias[i][6] = Bgs[i].x();
        para_SpeedBias[i][7] = Bgs[i].y();
        para_SpeedBias[i][8] = Bgs[i].z();
    }

    // Convert camera extrinsics
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        para_Ex_Pose[i][0] = tic[i].x();
        para_Ex_Pose[i][1] = tic[i].y();
        para_Ex_Pose[i][2] = tic[i].z();
        Quaterniond q{ric[i]};
        para_Ex_Pose[i][3] = q.x();
        para_Ex_Pose[i][4] = q.y();
        para_Ex_Pose[i][5] = q.z();
        para_Ex_Pose[i][6] = q.w();
    }

    // Convert feature depths
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        para_Feature[i][0] = dep(i);

    // Convert time offset if estimated
    if (ESTIMATE_TD)
        para_Td[0][0] = td;
}

/**
 * @brief Converts optimized double arrays back to state vectors
 * Updates poses, velocities, biases, extrinsics, feature depths, and time offset
 * from Ceres optimization results, with handling for yaw alignment and relocalization
 */
void Estimator::double2vector()
{
    // Store original pose for yaw alignment
    Vector3d origin_R0 = Utility::R2ypr(Rs[0]);
    Vector3d origin_P0 = Ps[0];

    // Use last valid pose if failure occurred
    if (failure_occur)
    {
        origin_R0 = Utility::R2ypr(last_R0);
        origin_P0 = last_P0;
        failure_occur = 0;
    }

    // Calculate yaw difference to align with original coordinate system
    Vector3d origin_R00 = Utility::R2ypr(Quaterniond(para_Pose[0][6],
                                                      para_Pose[0][3],
                                                      para_Pose[0][4],
                                                      para_Pose[0][5]).toRotationMatrix());
    double y_diff = origin_R0.x() - origin_R00.x();
    //TODO
    Matrix3d rot_diff = Utility::ypr2R(Vector3d(y_diff, 0, 0)); // Yaw correction rotation

    // Handle singular cases (near 90-degree pitch)
    if (abs(abs(origin_R0.y()) - 90) < 1.0 || abs(abs(origin_R00.y()) - 90) < 1.0)
    {
        ROS_DEBUG("euler singular point!");
        rot_diff = Rs[0] * Quaterniond(para_Pose[0][6],
                                       para_Pose[0][3],
                                       para_Pose[0][4],
                                       para_Pose[0][5]).toRotationMatrix().transpose();
    }

    // Update poses, velocities, and biases with optimization results
    for (int i = 0; i <= WINDOW_SIZE; i++)
    {
        // Update rotation (with yaw correction)
        Rs[i] = rot_diff * Quaterniond(para_Pose[i][6], para_Pose[i][3], para_Pose[i][4], para_Pose[i][5]).normalized().toRotationMatrix();
        // Update position (relative to first frame, with yaw correction)
        Ps[i] = rot_diff * Vector3d(para_Pose[i][0] - para_Pose[0][0],
                                para_Pose[i][1] - para_Pose[0][1],
                                para_Pose[i][2] - para_Pose[0][2]) + origin_P0;
        // Update velocity (with yaw correction)
        Vs[i] = rot_diff * Vector3d(para_SpeedBias[i][0],
                                    para_SpeedBias[i][1],
                                    para_SpeedBias[i][2]);
        // Update acceleration bias
        Bas[i] = Vector3d(para_SpeedBias[i][3],
                          para_SpeedBias[i][4],
                          para_SpeedBias[i][5]);
        // Update gyroscope bias
        Bgs[i] = Vector3d(para_SpeedBias[i][6],
                          para_SpeedBias[i][7],
                          para_SpeedBias[i][8]);
    }

    // Update camera extrinsics
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = Vector3d(para_Ex_Pose[i][0],
                          para_Ex_Pose[i][1],
                          para_Ex_Pose[i][2]);
        ric[i] = Quaterniond(para_Ex_Pose[i][6],
                             para_Ex_Pose[i][3],
                             para_Ex_Pose[i][4],
                             para_Ex_Pose[i][5]).toRotationMatrix();
    }

    // Update feature depths
    VectorXd dep = f_manager.getDepthVector();
    for (int i = 0; i < f_manager.getFeatureCount(); i++)
        dep(i) = para_Feature[i][0];
    f_manager.setDepth(dep);
    // Update time offset if estimated
    if (ESTIMATE_TD)
        td = para_Td[0][0];

    // relative info between two loop frame
    // Handle relocalization information (correct drift)
    if(relocalization_info)
    { 
        Matrix3d relo_r;
        Vector3d relo_t;
        // Get relocalization pose from optimization results
        relo_r = rot_diff * Quaterniond(relo_Pose[6], relo_Pose[3], relo_Pose[4], relo_Pose[5]).normalized().toRotationMatrix();
        relo_t = rot_diff * Vector3d(relo_Pose[0] - para_Pose[0][0],
                                     relo_Pose[1] - para_Pose[0][1],
                                     relo_Pose[2] - para_Pose[0][2]) + origin_P0;
        // Calculate yaw drift correction
        double drift_correct_yaw;
        drift_correct_yaw = Utility::R2ypr(prev_relo_r).x() - Utility::R2ypr(relo_r).x();
        drift_correct_r = Utility::ypr2R(Vector3d(drift_correct_yaw, 0, 0));
        drift_correct_t = prev_relo_t - drift_correct_r * relo_t;   
        // Calculate relative pose for relocalization
        relo_relative_t = relo_r.transpose() * (Ps[relo_frame_local_index] - relo_t);
        relo_relative_q = relo_r.transpose() * Rs[relo_frame_local_index];
        relo_relative_yaw = Utility::normalizeAngle(Utility::R2ypr(Rs[relo_frame_local_index]).x() - Utility::R2ypr(relo_r).x());
        relocalization_info = 0; // Reset relocalization flag
    }
}

/**
 * @brief Detects system failures based on various conditions
 * Checks feature tracking quality, IMU bias magnitude, and pose changes
 * @return True if failure is detected, false otherwise
 */
bool Estimator::failureDetection()
{
    // Check if too few features are tracked
    if (f_manager.last_track_num < 2)
    {
        ROS_INFO(" little feature %d", f_manager.last_track_num);
    }
    // Check for large acceleration bias
    if (Bas[WINDOW_SIZE].norm() > 2.5)
    {
        ROS_INFO(" big IMU acc bias estimation %f", Bas[WINDOW_SIZE].norm());
        return true;
    }
    // Check for large gyroscope bias
    if (Bgs[WINDOW_SIZE].norm() > 1.0)
    {
        ROS_INFO(" big IMU gyr bias estimation %f", Bgs[WINDOW_SIZE].norm());
        return true;
    }
    
   // Check for large position change
    Vector3d tmp_P = Ps[WINDOW_SIZE];
    if ((tmp_P - last_P).norm() > 5)
    {
        ROS_INFO(" big translation");
        return true;
    }
    // Check for large vertical position change
    if (abs(tmp_P.z() - last_P.z()) > 1)
    {
        ROS_INFO(" big z translation");
        return true; 
    }
    // Check for large rotation change
    Matrix3d tmp_R = Rs[WINDOW_SIZE];
    Matrix3d delta_R = tmp_R.transpose() * last_R;
    Quaterniond delta_Q(delta_R);
    double delta_angle;
    delta_angle = acos(delta_Q.w()) * 2.0 / 3.14 * 180.0; // Convert to degrees
    if (delta_angle > 50)
    {
        ROS_INFO(" big delta_angle ");
        //return true;
    }
    return false;
}

/**
 * @brief Performs non-linear optimization using Ceres Solver
 * Constructs optimization problem with IMU factors, projection factors, and marginalization factors,
 * runs solver, and handles marginalization of old states
 */
void Estimator::optimization()
{
    ceres::Problem problem;
    ceres::LossFunction *loss_function;
    loss_function = new ceres::CauchyLoss(1.0); // Robust loss function to handle outliers
    // Add pose and speed-bias parameters to problem
    for (int i = 0; i < WINDOW_SIZE + 1; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization(); // For SE(3) manifold
        problem.AddParameterBlock(para_Pose[i], SIZE_POSE, local_parameterization);
        problem.AddParameterBlock(para_SpeedBias[i], SIZE_SPEEDBIAS); // Velocity + biases (no manifold)
    }

    // Add camera extrinsic parameters
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(para_Ex_Pose[i], SIZE_POSE, local_parameterization);
        if (!ESTIMATE_EXTRINSIC)
        {
            ROS_DEBUG("fix extinsic param");
            problem.SetParameterBlockConstant(para_Ex_Pose[i]); // Fix extrinsics if not estimated
        }
        else
            ROS_DEBUG("estimate extinsic param");
    }
    // Add time offset parameter if enabled
    if (ESTIMATE_TD)
    {
        problem.AddParameterBlock(para_Td[0], 1);
    }

    TicToc t_whole, t_prepare;
    vector2double(); // Convert states to double arrays for optimization

    // Add marginalization factor from previous step
    if (last_marginalization_info)
    {
        // construct new marginlization_factor
        MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
        problem.AddResidualBlock(marginalization_factor, NULL,
                                 last_marginalization_parameter_blocks);
    }

    // Add IMU factors between consecutive frames
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        int j = i + 1;
        if (pre_integrations[j]->sum_dt > 10.0) // Skip if too much time between frames
            continue;
        IMUFactor* imu_factor = new IMUFactor(pre_integrations[j]); // IMU pre-integration factor
        problem.AddResidualBlock(imu_factor, NULL, para_Pose[i], para_SpeedBias[i], para_Pose[j], para_SpeedBias[j]);
    }
    // Add visual projection factors
    int f_m_cnt = 0; // Count of visual factors
    int feature_index = -1;
    for (auto &it_per_id : f_manager.feature)
    {
        it_per_id.used_num = it_per_id.feature_per_frame.size();
        // Skip features with insufficient observations
        if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
 
        ++feature_index;

        int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
        
        Vector3d pts_i = it_per_id.feature_per_frame[0].point; // Reference observation

        // Add factors for all observations of this feature
        for (auto &it_per_frame : it_per_id.feature_per_frame)
        {
            imu_j++;
            if (imu_i == imu_j) // Skip same frame
            {
                continue;
            }
            Vector3d pts_j = it_per_frame.point; // Current observation
            // Use time-offset projection factor if enabled
            if (ESTIMATE_TD)
            {
                    ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                     it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                     it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                    problem.AddResidualBlock(f_td, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]);
            }
            else // Standard projection factor
            {
                ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                problem.AddResidualBlock(f, loss_function, para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]);
            }
            f_m_cnt++;
        }
    }

    ROS_DEBUG("visual measurement count: %d", f_m_cnt);
    ROS_DEBUG("prepare for ceres: %f", t_prepare.toc());

    // Add relocalization factors if enabled
    if(relocalization_info)
    {
        ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
        problem.AddParameterBlock(relo_Pose, SIZE_POSE, local_parameterization);
        int retrive_feature_index = 0;
        int feature_index = -1;
        for (auto &it_per_id : f_manager.feature)
        {
            it_per_id.used_num = it_per_id.feature_per_frame.size();
            if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            ++feature_index;
            int start = it_per_id.start_frame;
            if(start <= relo_frame_local_index)
            {   
                // Find matching features from relocalization
                while((int)match_points[retrive_feature_index].z() < it_per_id.feature_id)
                {
                    retrive_feature_index++;
                }
                if((int)match_points[retrive_feature_index].z() == it_per_id.feature_id)
                {
                    Vector3d pts_j = Vector3d(match_points[retrive_feature_index].x(), match_points[retrive_feature_index].y(), 1.0);
                    Vector3d pts_i = it_per_id.feature_per_frame[0].point;
                    // Add projection factor between current frame and relocalization frame
                    ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                    problem.AddResidualBlock(f, loss_function, para_Pose[start], relo_Pose, para_Ex_Pose[0], para_Feature[feature_index]);
                    retrive_feature_index++;
                }     
            }
        }

    }

    // Configure Ceres solver options
    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR; // Efficient for bundle adjustment
    options.trust_region_strategy_type = ceres::DOGLEG; // Trust region strategy
    options.max_num_iterations = NUM_ITERATIONS; // Maximum iterations
    // Adjust solver time based on marginalization type
    if (marginalization_flag == MARGIN_OLD)
        options.max_solver_time_in_seconds = SOLVER_TIME * 4.0 / 5.0;
    else
        options.max_solver_time_in_seconds = SOLVER_TIME;
    // Run optimization
    TicToc t_solver;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    ROS_DEBUG("Iterations : %d", static_cast<int>(summary.iterations.size()));
    ROS_DEBUG("solver costs: %f", t_solver.toc());
    // Convert optimization results back to state vectors
    double2vector();

    // Handle marginalization (remove old states while preserving information)
    TicToc t_whole_marginalization;
    if (marginalization_flag == MARGIN_OLD) // Marginalize oldest frame
    {
        MarginalizationInfo *marginalization_info = new MarginalizationInfo();
        vector2double(); // Update double arrays

        // Add previous marginalization info (excluding old frame)
        if (last_marginalization_info)
        {
            vector<int> drop_set;
            for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
            {
                if (last_marginalization_parameter_blocks[i] == para_Pose[0] ||
                    last_marginalization_parameter_blocks[i] == para_SpeedBias[0])
                    drop_set.push_back(i); // Drop oldest frame's parameters
            }
            // construct new marginlization_factor
            MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
            ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                           last_marginalization_parameter_blocks,
                                                                           drop_set);

            marginalization_info->addResidualBlockInfo(residual_block_info);
        }

        // Add IMU factor between oldest and next frame
        {
            if (pre_integrations[1]->sum_dt < 10.0)
            {
                IMUFactor* imu_factor = new IMUFactor(pre_integrations[1]);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(imu_factor, NULL,
                                                                           vector<double *>{para_Pose[0], para_SpeedBias[0], para_Pose[1], para_SpeedBias[1]},
                                                                           vector<int>{0, 1});
                marginalization_info->addResidualBlockInfo(residual_block_info);
            }
        }

        // Add visual factors involving the oldest frame
        {
            int feature_index = -1;
            for (auto &it_per_id : f_manager.feature)
            {
                it_per_id.used_num = it_per_id.feature_per_frame.size();
                if (!(it_per_id.used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                    continue;

                ++feature_index;

                int imu_i = it_per_id.start_frame, imu_j = imu_i - 1;
                if (imu_i != 0) // Only process features starting at oldest frame
                    continue;

                Vector3d pts_i = it_per_id.feature_per_frame[0].point;

                for (auto &it_per_frame : it_per_id.feature_per_frame)
                {
                    imu_j++;
                    if (imu_i == imu_j)
                        continue;

                    Vector3d pts_j = it_per_frame.point;
                    if (ESTIMATE_TD)
                    {
                        ProjectionTdFactor *f_td = new ProjectionTdFactor(pts_i, pts_j, it_per_id.feature_per_frame[0].velocity, it_per_frame.velocity,
                                                                          it_per_id.feature_per_frame[0].cur_td, it_per_frame.cur_td,
                                                                          it_per_id.feature_per_frame[0].uv.y(), it_per_frame.uv.y());
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f_td, loss_function,
                                                                                        vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index], para_Td[0]},
                                                                                        vector<int>{0, 3}); // Drop oldest frame and feature depth
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                    else
                    {
                        ProjectionFactor *f = new ProjectionFactor(pts_i, pts_j);
                        ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(f, loss_function,
                                                                                       vector<double *>{para_Pose[imu_i], para_Pose[imu_j], para_Ex_Pose[0], para_Feature[feature_index]},
                                                                                       vector<int>{0, 3}); // Drop oldest frame and feature depth
                        marginalization_info->addResidualBlockInfo(residual_block_info);
                    }
                }
            }
        }

        // Perform marginalization
        TicToc t_pre_margin;
        marginalization_info->preMarginalize(); // Precompute Jacobians
        ROS_DEBUG("pre marginalization %f ms", t_pre_margin.toc());
        
        TicToc t_margin;
        marginalization_info->marginalize(); // Compute Schur complement
        ROS_DEBUG("marginalization %f ms", t_margin.toc());

        // Update parameter addresses after sliding window
        std::unordered_map<long, double *> addr_shift;
        for (int i = 1; i <= WINDOW_SIZE; i++)
        {
            addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
            addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
        }
        for (int i = 0; i < NUM_OF_CAM; i++)
            addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
        if (ESTIMATE_TD)
        {
            addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
        }
        vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);

        // Update marginalization info for next iteration
        if (last_marginalization_info)
            delete last_marginalization_info;
        last_marginalization_info = marginalization_info;
        last_marginalization_parameter_blocks = parameter_blocks;
        
    }
    else // Marginalize second newest frame
    {
        if (last_marginalization_info &&
            std::count(std::begin(last_marginalization_parameter_blocks), std::end(last_marginalization_parameter_blocks), para_Pose[WINDOW_SIZE - 1]))
        {

            MarginalizationInfo *marginalization_info = new MarginalizationInfo();
            vector2double();
            // Add previous marginalization info (excluding second newest frame)
            if (last_marginalization_info)
            {
                vector<int> drop_set;
                for (int i = 0; i < static_cast<int>(last_marginalization_parameter_blocks.size()); i++)
                {
                    ROS_ASSERT(last_marginalization_parameter_blocks[i] != para_SpeedBias[WINDOW_SIZE - 1]);
                    if (last_marginalization_parameter_blocks[i] == para_Pose[WINDOW_SIZE - 1])
                        drop_set.push_back(i); // Drop second newest frame
                }
                // construct new marginlization_factor
                MarginalizationFactor *marginalization_factor = new MarginalizationFactor(last_marginalization_info);
                ResidualBlockInfo *residual_block_info = new ResidualBlockInfo(marginalization_factor, NULL,
                                                                               last_marginalization_parameter_blocks,
                                                                               drop_set);

                marginalization_info->addResidualBlockInfo(residual_block_info);
            }

            // Perform marginalization
            TicToc t_pre_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->preMarginalize();
            ROS_DEBUG("end pre marginalization, %f ms", t_pre_margin.toc());

            TicToc t_margin;
            ROS_DEBUG("begin marginalization");
            marginalization_info->marginalize();
            ROS_DEBUG("end marginalization, %f ms", t_margin.toc());
            
            // Update parameter addresses after sliding window
            std::unordered_map<long, double *> addr_shift;
            for (int i = 0; i <= WINDOW_SIZE; i++)
            {
                if (i == WINDOW_SIZE - 1)
                    continue; // Skip marginalized frame
                else if (i == WINDOW_SIZE)
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i - 1];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i - 1];
                }
                else
                {
                    addr_shift[reinterpret_cast<long>(para_Pose[i])] = para_Pose[i];
                    addr_shift[reinterpret_cast<long>(para_SpeedBias[i])] = para_SpeedBias[i];
                }
            }
            for (int i = 0; i < NUM_OF_CAM; i++)
                addr_shift[reinterpret_cast<long>(para_Ex_Pose[i])] = para_Ex_Pose[i];
            if (ESTIMATE_TD)
            {
                addr_shift[reinterpret_cast<long>(para_Td[0])] = para_Td[0];
            }
            
            vector<double *> parameter_blocks = marginalization_info->getParameterBlocks(addr_shift);
            // Update marginalization info for next iteration
            if (last_marginalization_info)
                delete last_marginalization_info;
            last_marginalization_info = marginalization_info;
            last_marginalization_parameter_blocks = parameter_blocks;        
        }
    }
    ROS_DEBUG("whole marginalization costs: %f", t_whole_marginalization.toc());
    
    ROS_DEBUG("whole time for ceres: %f", t_whole.toc());
}

/**
 * @brief Manages the sliding window by removing either the oldest frame or the second-newest frame
 *        based on the marginalization flag, and updates window states accordingly.
 *        This function maintains the fixed window size by shifting data and cleaning up obsolete states.
 */
void Estimator::slideWindow()
{
    TicToc t_margin; // Timer to measure marginalization time
    // Case 1: Marginalize the oldest frame (when current frame is a keyframe)
    if (marginalization_flag == MARGIN_OLD)
    {
        double t_0 = Headers[0].stamp.toSec();// Timestamp of the oldest frame to be removed
        back_R0 = Rs[0]; // Backup rotation of the oldest frame for depth adjustment
        back_P0 = Ps[0]; // Backup position of the oldest frame for depth adjustment
        // Only slide when the window is full (reached maximum size)
        if (frame_count == WINDOW_SIZE)
        {
            // Shift all states (poses, pre-integrations, buffers) forward by one position
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                Rs[i].swap(Rs[i + 1]); // Swap rotation with next frame

                std::swap(pre_integrations[i], pre_integrations[i + 1]); // Swap pre-integration objects

                // Swap IMU data buffers
                dt_buf[i].swap(dt_buf[i + 1]);
                linear_acceleration_buf[i].swap(linear_acceleration_buf[i + 1]);
                angular_velocity_buf[i].swap(angular_velocity_buf[i + 1]);

                Headers[i] = Headers[i + 1];  // Update header with next frame's timestamp
                Ps[i].swap(Ps[i + 1]);        // Swap position with next frame
                Vs[i].swap(Vs[i + 1]);        // Swap velocity with next frame
                Bas[i].swap(Bas[i + 1]);      // Swap acceleration bias with next frame
                Bgs[i].swap(Bgs[i + 1]);      // Swap gyro bias with next frame
            }
            // Initialize the new last frame (previously the second-last) with the latest data
            Headers[WINDOW_SIZE] = Headers[WINDOW_SIZE - 1];
            Ps[WINDOW_SIZE] = Ps[WINDOW_SIZE - 1];
            Vs[WINDOW_SIZE] = Vs[WINDOW_SIZE - 1];
            Rs[WINDOW_SIZE] = Rs[WINDOW_SIZE - 1];
            Bas[WINDOW_SIZE] = Bas[WINDOW_SIZE - 1];
            Bgs[WINDOW_SIZE] = Bgs[WINDOW_SIZE - 1];

            // Reinitialize pre-integrator for the new last frame
            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            // Clear IMU buffers for the new last frame (will be filled with new data)
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            // Clean up obsolete image frames from the all_image_frame map
            if (true || solver_flag == INITIAL)
            {
                map<double, ImageFrame>::iterator it_0; // Find the oldest frame
                it_0 = all_image_frame.find(t_0);
                delete it_0->second.pre_integration; // Free pre-integration data of the oldest frame
                it_0->second.pre_integration = nullptr;
 
                // Remove all frames older than the oldest frame in the window
                for (map<double, ImageFrame>::iterator it = all_image_frame.begin(); it != it_0; ++it)
                {
                    if (it->second.pre_integration)
                        delete it->second.pre_integration;
                    it->second.pre_integration = NULL;
                }

                // Erase the oldest frame and all prior frames from the map
                all_image_frame.erase(all_image_frame.begin(), it_0);
                all_image_frame.erase(t_0);

            }
            // Perform specific cleanup for the oldest frame (adjust feature depths)
            slideWindowOld();
        }
    }
    // Case 2: Marginalize the second-newest frame (when current frame is not a keyframe)
    else
    {
        // Only slide when the window is full
        if (frame_count == WINDOW_SIZE)
        {
            // Merge IMU data from the newest frame into the second-newest frame
            for (unsigned int i = 0; i < dt_buf[frame_count].size(); i++)
            {
                double tmp_dt = dt_buf[frame_count][i];
                Vector3d tmp_linear_acceleration = linear_acceleration_buf[frame_count][i];
                Vector3d tmp_angular_velocity = angular_velocity_buf[frame_count][i];

                // Add IMU data to the second-newest frame's pre-integrator
                pre_integrations[frame_count - 1]->push_back(tmp_dt, tmp_linear_acceleration, tmp_angular_velocity);

                // Append IMU data to the second-newest frame's buffers
                dt_buf[frame_count - 1].push_back(tmp_dt);
                linear_acceleration_buf[frame_count - 1].push_back(tmp_linear_acceleration);
                angular_velocity_buf[frame_count - 1].push_back(tmp_angular_velocity);
            }

            // Update the second-newest frame with the newest frame's states
            Headers[frame_count - 1] = Headers[frame_count];
            Ps[frame_count - 1] = Ps[frame_count];
            Vs[frame_count - 1] = Vs[frame_count];
            Rs[frame_count - 1] = Rs[frame_count];
            Bas[frame_count - 1] = Bas[frame_count];
            Bgs[frame_count - 1] = Bgs[frame_count];

            // Reinitialize pre-integrator for the new last frame (now empty)
            delete pre_integrations[WINDOW_SIZE];
            pre_integrations[WINDOW_SIZE] = new IntegrationBase{acc_0, gyr_0, Bas[WINDOW_SIZE], Bgs[WINDOW_SIZE]};

            // Clear IMU buffers for the new last frame
            dt_buf[WINDOW_SIZE].clear();
            linear_acceleration_buf[WINDOW_SIZE].clear();
            angular_velocity_buf[WINDOW_SIZE].clear();

            // Perform specific cleanup for the second-newest frame
            slideWindowNew();
        }
    }
}

// real marginalization is removed in solve_ceres()
/**
 * @brief Handles sliding window by removing the second-newest frame (non-keyframe)
 *        Updates feature manager to remove features associated with the front frame.
 */
void Estimator::slideWindowNew()
{
    sum_of_front++; // Increment counter for front marginalizations
    // Remove features observed in the front frame (second-newest frame)
    f_manager.removeFront(frame_count);
}
// real marginalization is removed in solve_ceres()
/**
 * @brief Handles sliding window by removing the oldest frame (keyframe)
 *        Adjusts feature depths based on the marginalized frame's pose if in non-linear mode.
 */
void Estimator::slideWindowOld()
{
    sum_of_back++; // Increment counter for back marginalizations

    // Determine if depth adjustment is needed (only in non-linear optimization mode)
    bool shift_depth = solver_flag == NON_LINEAR ? true : false;
    if (shift_depth)
    {
        Matrix3d R0, R1; // Rotations of the marginalized frame and new first frame (camera coords)
        Vector3d P0, P1;// Positions of the marginalized frame and new first frame (camera coords)
        // Convert IMU poses to camera poses using extrinsics
        R0 = back_R0 * ric[0];  // Rotation of marginalized frame (camera)
        R1 = Rs[0] * ric[0];    // Rotation of new first frame (camera)
        P0 = back_P0 + back_R0 * tic[0];  // Position of marginalized frame (camera)
        P1 = Ps[0] + Rs[0] * tic[0];      // Position of new first frame (camera)
        
        // Remove the oldest frame's features and adjust their depths using pose transformation
        f_manager.removeBackShiftDepth(R0, P0, R1, P1);
    }
    else
    {
        // In initial mode: simply remove features associated with the oldest frame
        f_manager.removeBack();
    }
}

/**
 * @brief Sets relocalization information to correct drift using a matched frame from the global map
 * @param _frame_stamp Timestamp of the relocalization frame
 * @param _frame_index Global index of the relocalization frame
 * @param _match_points Matched 2D-3D features between current frame and relocalization frame
 * @param _relo_t Translation of the relocalization frame in global map
 * @param _relo_r Rotation of the relocalization frame in global map
 */
void Estimator::setReloFrame(double _frame_stamp, int _frame_index, vector<Vector3d> &_match_points, Vector3d _relo_t, Matrix3d _relo_r)
{
    relo_frame_stamp = _frame_stamp;  // Store timestamp of the relocalization frame
    relo_frame_index = _frame_index;  // Store global index of the relocalization frame
    match_points.clear();
    match_points = _match_points;     // Store matched features (3D: x,y,feature_id)
    prev_relo_t = _relo_t;            // Store previous relocalization translation
    prev_relo_r = _relo_r;            // Store previous relocalization rotation

    // Find the local index of the relocalization frame in the current window
    for(int i = 0; i < WINDOW_SIZE; i++)
    {
        if(relo_frame_stamp == Headers[i].stamp.toSec())
        {
            relo_frame_local_index = i; // Record local index in the window
            relocalization_info = 1; // Flag: valid relocalization info available

            // Copy the pose of the relocalization frame from current optimization parameters
            for (int j = 0; j < SIZE_POSE; j++)
                relo_Pose[j] = para_Pose[i][j];
        }
    }
}

