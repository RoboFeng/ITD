#include <IntensityTD.hpp>
#include <iostream>
#include <fstream>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

typedef Eigen::Array<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> PoseArray;

std::string bag_path, pose_path, cloud_topic;
int width, height;
int kf_frame_gap;
int lc_mode = 0; // 0:itdicp 1:icp 2:itd
float run_freq;  // The frequency of program execution
shared_ptr<IntensityTD> intensity_td;
shared_ptr<ITDVis> itd_vis;
shared_ptr<ITDLog> itd_log;

// Factor graph optimization, saving the pose and timestamp of each frame
gtsam::Values initial;
gtsam::NonlinearFactorGraph graph;
vector<Eigen::Affine3d> pose_vec;
vector<Eigen::Affine3d> pose_vec_aftopt;
vector<double> pose_timestamp;

// Capture keyboard termination signal
bool flg_exit = false;
void KillHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
}
void readLidarPoses(std::string pose_path, PoseArray &lidar_pose)
{
    std::vector<double> pose_tmp; // Used to cache all data
    std::ifstream fin(pose_path);
    if (fin)
    {
        int line_num = 0;
        std::string line;
        while (getline(fin, line)) // Automatically remove line breaks
        {
            std::stringstream linestream(line);
            std::string out;
            while (getline(linestream, out, ' '))
            {
                pose_tmp.emplace_back(stod(out));
            }

            line_num++;
        }
        lidar_pose = Eigen::Map<PoseArray>(pose_tmp.data(), line_num, int(pose_tmp.size() / line_num));
    }
    else
    {
        ROS_ERROR("Can not find ouster_traj.txt!");
    }
}

// Return its pose index ID based on timestamp
Eigen::Index getIdFromTimestamp(double ts, PoseArray &pose)
{
    Eigen::Array<double, Eigen::Dynamic, 1> ts_diff;
    ts_diff = (pose.col(0) - ts).abs();
    Eigen::Index idx0, idx1;
    ts_diff.minCoeff(&idx0, &idx1);
    if (ts_diff[idx0] > 0.1)
        return -1;
    else
        return idx0;
}

void trans2LidarFrame(const PointCloudOuster::Ptr &ptr_ouster, PointCloudXYZI::Ptr &ptr, cv::Mat &inten_img)
{
    // Rearrange point clouds in image order
    ptr->resize(height * width);
    inten_img = cv::Mat(height, width, cv_pixel_type, cv::Scalar(0));

    int pixel_shift_by_row[4] = {48, 32, 16, 0}; // 2048

    for (size_t v = 0; v < height; v++)
        for (size_t u = 0; u < width; u++)
        {
            const int uu = (u + width - pixel_shift_by_row[v % 4]) % width;
            const int ori_idx = v * width + uu;
            const int idx = v * width + u;
            const auto &pt = ptr_ouster->points[ori_idx];
            ptr->points[idx].x = pt.x;
            ptr->points[idx].y = pt.y;
            ptr->points[idx].z = pt.z;
            ptr->points[idx].intensity = pt.reflectivity;
            inten_img.at<pixel_type>(v, u) = pt.reflectivity;
        }
}

void update_poses(const gtsam::Values &estimates,
                  std::vector<Eigen::Affine3d> &poses)
{
    assert(estimates.size() == poses.size());

    poses.clear();

    for (int i = 0; i < estimates.size(); ++i)
    {
        auto est = estimates.at<gtsam::Pose3>(i);
        Eigen::Affine3d est_affine3d(est.matrix());
        poses.push_back(est_affine3d);
    }
}

int main(int argc, char **argv)
{
    Eigen::initParallel();
    ros::init(argc, argv, "demo_fusionportable_opt");
    // Avoid PCL Warnings
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    ros::NodeHandle nh;

    intensity_td = std::make_shared<IntensityTD>(nh);
    itd_vis = std::make_shared<ITDVis>(nh);
    itd_log = std::make_shared<ITDLog>(nh);

    nh.param<float>("run_freq", run_freq, 10.0);
    nh.param<std::string>("bag_path", bag_path, "~/Fusionportable/20220216_corridor_day.bag");
    nh.param<std::string>("pose_path", pose_path, "~/Fusionportable/pose.txt");
    nh.param<std::string>("cloud_topic", cloud_topic, "/os_cloud_node/points");
    nh.param<int>("width", width, 2048);
    nh.param<int>("height", height, 128);
    nh.param<int>("kf_frame_gap", kf_frame_gap, 10);
    nh.param<int>("lc_mode", lc_mode, 0);

    /*--------------------------------------------------Read LiDAR pose--------------------------------------------------*/
    // Read LiDAR pose, use hash storage for easy reading
    PoseArray lidar_pose;
    readLidarPoses(pose_path, lidar_pose);
    /*--------------------------------------------------Read ROS package--------------------------------------------------*/
    std::cout << "Loading Bag from Path: " << bag_path << std::endl;
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);
    std::cout << "Loading Clouds from Topic: " << cloud_topic << std::endl;
    rosbag::View view(bag, rosbag::TopicQuery({cloud_topic}));

    /*--------------------------------------------------Initialize factor graph related parameters--------------------------------------------------*/

    // https://github.com/TixiaoShan/LIO-SAM/blob/6665aa0a4fcb5a9bb3af7d3923ae4a035b489d47/src/mapOptmization.cpp#L1385
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
        gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8)
                .finished()); // rad*rad, meter*meter

    double loopNoiseScore = 1e-1;
    gtsam::Vector robustNoiseVector6(
        6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
        loopNoiseScore, loopNoiseScore, loopNoiseScore;
    gtsam::noiseModel::Base::shared_ptr robustLoopNoise =
        gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    gtsam::ISAM2 isam(parameters);

    // Wait for 2 seconds to enter the main loop
    sleep(2);
    /*--------------------------------------------------!Main loop--------------------------------------------------*/
    int frame_id = 0;
    int kf_id = 0;
    bool first_flg = true;
    vector<cv::Mat> kf_imgs;                                          // Collect image data within keyframes
    PointCloudCorner::Ptr corners_kf_collect(new PointCloudCorner()); // Collect corner data within keyframes for constructing ITD descriptors
    PointCloudXYZI::Ptr feat_kf_collect(new PointCloudXYZI());        // Collect point cloud data within keyframes for ICP

    signal(SIGINT, KillHandle);
    ros::Rate ros_loop(run_freq);
    bool ros_status = ros::ok();
    for (rosbag::MessageInstance const m : view)
    {
        if (flg_exit || !ros_status)
            break;
        /*************************************************Read raw data******************************************************/
        if (m.getTopic() != cloud_topic)
        {
            continue;
        }
        sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (msg == NULL)
            continue;
        PointCloudOuster::Ptr ptr_ouster(new PointCloudOuster);
        double ros_timestamp = msg->header.stamp.toSec();
        pcl::fromROSMsg(*msg, *ptr_ouster);
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();

        // Read poses
        int query_id = getIdFromTimestamp(ros_timestamp, lidar_pose);
        if (query_id != -1)
        {
            auto pose_curr = lidar_pose.row(query_id);
            Eigen::Quaterniond r(pose_curr[7], pose_curr[4], pose_curr[5], pose_curr[6]);
            Eigen::Vector3d t(pose_curr[1], pose_curr[2], pose_curr[3]);
            pose.translate(t);
            pose.rotate(r);
        }
        else
            continue;

        // raw data
        PointCloudXYZI::Ptr ptr_feat(new PointCloudXYZI());
        cv::Mat inten_img; // Intensity image, used for corner detection
        trans2LidarFrame(ptr_ouster, ptr_feat, inten_img);

        /*************************************************Subframe data processing******************************************************/
        // Automatic exposure
        intensity_td->autoExposure(inten_img, first_flg);
        // Remove image stripe noise
        intensity_td->removeStripNoise(inten_img);
        // Extracting corner points from images
        PointCloudCorner::Ptr corners(new PointCloudCorner());
        intensity_td->detectCorners(ptr_feat, inten_img, frame_id % kf_frame_gap, corners);
        // Collect corner points of sub frames
        pcl::transformPointCloud(*corners, *corners, pose.matrix()); // Transform all corner points to the world coordinate system
        *corners_kf_collect += *corners;
        // Collect images of sub frames
        kf_imgs.emplace_back(inten_img);

        // Publish the point cloud, image, and pose of the current frame
        itd_vis->pubIntenImg(inten_img, ros_timestamp);
        PointCloudXYZI::Ptr ptr_feat_world(new PointCloudXYZI());
        pcl::transformPointCloud(*ptr_feat, *ptr_feat_world, pose.matrix()); // Transform the LiDAR point to the world coordinate system
        *feat_kf_collect += *ptr_feat_world;
        itd_vis->pubPclWorld(ptr_feat_world, ros_timestamp);
        itd_vis->pubOdomPath(pose, ros_timestamp);

        /*************************************************Factor graph optimization******************************************************/
        pose_vec.push_back(pose);
        pose_vec_aftopt.push_back(pose);
        pose_timestamp.push_back(ros_timestamp);

        if (!frame_id)
        {
            graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                0, gtsam::Pose3(pose.matrix()), odometryNoise));
        }
        else
        {
            auto prev_pose = gtsam::Pose3(pose_vec[frame_id - 1].matrix());
            auto curr_pose = gtsam::Pose3(pose.matrix());
            graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                frame_id - 1, frame_id, prev_pose.between(curr_pose),
                odometryNoise));
        }
        initial.insert(frame_id, gtsam::Pose3(pose.matrix()));

        /*************************************************Keyframe data processing******************************************************/
        bool isloop = false;
        if ((frame_id % kf_frame_gap) == 0 && (frame_id > 0))
        {
            // Define the consumption time variable (time ms)
            double keypoints_time = 0.0;
            double build_time = 0.0;
            double addtree_time = 0.0;
            double candidate_time = 0.0;
            double vertify_time = 0.0;

            // Generate key points from collected corner points
            PointCloudGmm::Ptr keypoints_kf(new PointCloudGmm());
            auto t1 = std::chrono::high_resolution_clock::now();
            intensity_td->generateKeypoints(corners_kf_collect, kf_imgs, keypoints_kf);
            auto t2 = std::chrono::high_resolution_clock::now();
            PointCloudXYZI::Ptr keypoints_world(new PointCloudXYZI());
            pcl::copyPointCloud(*keypoints_kf, *keypoints_world);
            itd_vis->pubKeypoints(keypoints_world, ros_timestamp);
            pcl::transformPointCloud(*keypoints_kf, *keypoints_kf, pose.inverse().matrix()); // Transform all corner points to the LiDAR coordinate system

            PointCloudItdesc::Ptr itds_kf(new PointCloudItdesc());
            auto t3 = std::chrono::high_resolution_clock::now();
            intensity_td->generateItdesc(kf_id, keypoints_kf, itds_kf);
            auto t4 = std::chrono::high_resolution_clock::now();

            PointCloudXYZI::Ptr feat_kf(new PointCloudXYZI());
            pcl::transformPointCloud(*feat_kf_collect, *feat_kf, pose.inverse().matrix()); // Transform the collected LiDAR points to the LiDAR coordinate system

            // Build keyframes
            ITDFrame *itd_frame = new ITDFrame();
            itd_frame->index = kf_id;
            itd_frame->frame_id = frame_id;
            itd_frame->time_stamp = ros_timestamp;
            itd_frame->keypoints = keypoints_kf;
            itd_frame->itds_vec = itds_kf;
            if (lc_mode == 2)
                feat_kf->clear();
            else
                itd_frame->points_ori = feat_kf;
            itd_frame->intensity_img = inten_img.clone();

            // Perform loop detection
            std::pair<int, double> loop_result;
            Eigen::Affine3d loop_transform;
            std::vector<std::pair<PointItdesc, PointItdesc>> loop_itd_pair;
            isloop = intensity_td->detectLoop(itd_frame, loop_result, loop_transform, loop_itd_pair, candidate_time, vertify_time);

            if (loop_result.first > 0)
                itd_log->logDumpPredictScore(itd_frame->index, intensity_td->getITDFrame(loop_result.first)->index,
                                             itd_frame->time_stamp, intensity_td->getITDFrame(loop_result.first)->time_stamp, loop_result.second, 0.0);

            // Add keyframes and descriptors to the database
            auto t5 = std::chrono::high_resolution_clock::now();
            intensity_td->addITDFrame(itd_frame);
            auto t6 = std::chrono::high_resolution_clock::now();

            keypoints_time = time_inc(t2, t1);
            build_time = time_inc(t4, t3);
            addtree_time = time_inc(t6, t5);
            itd_log->logDumpTimeCost(itd_frame->time_stamp, keypoints_time, build_time, addtree_time, candidate_time, vertify_time);

            if (isloop)
            {
                // Retrieve the matched historical frames
                ITDFrame *itd_frame_loop = intensity_td->getITDFrame(loop_result.first);
                int src_frame = frame_id;
                int tar_frame = itd_frame_loop->frame_id;

                itd_vis->pubLoop(pose_vec[src_frame], pose_vec[tar_frame], ros_timestamp);
                itd_vis->pubMatchedItds(*itd_frame, *itd_frame_loop, pose_vec[src_frame], pose_vec[tar_frame], loop_itd_pair, ros_timestamp);
                itd_vis->pubMatchedImg(*itd_frame, *itd_frame_loop, loop_itd_pair, ros_timestamp);

                Eigen::Affine3d pose_relative;
                switch (lc_mode)
                {
                case 0:
                    pose_relative = loop_transform.inverse();
                    if (intensity_td->doICPRelative(itd_frame->points_ori, itd_frame_loop->points_ori, pose_relative))
                        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            tar_frame, src_frame,
                            gtsam::Pose3(pose_relative.matrix()),
                            robustLoopNoise));
                    break;
                case 1:
                    pose_relative = Eigen::Affine3d::Identity();
                    if (intensity_td->doICPRelative(itd_frame->points_ori, itd_frame_loop->points_ori, pose_relative))
                        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            tar_frame, src_frame,
                            gtsam::Pose3(pose_relative.matrix()),
                            robustLoopNoise));
                    break;
                case 2:
                    pose_relative = loop_transform.inverse();
                    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                        tar_frame, src_frame,
                        gtsam::Pose3(pose_relative.matrix()),
                        robustLoopNoise));
                    break;
                default:
                    ROS_ERROR("wrong lc_mode!");
                }
            }

            // Clear keyframe data
            corners_kf_collect->clear();
            feat_kf_collect->clear();
            kf_imgs.clear();
            kf_id++;
        }
        first_flg = false;
        frame_id++;

        // Update factor graph
        isam.update(graph, initial);
        isam.update();

        if (isloop)
        {
            isam.update();
            isam.update();
            isam.update();
            isam.update();
            isam.update();
        }

        graph.resize(0);
        initial.clear();

        gtsam::Values curr_estimate;
        curr_estimate = isam.calculateEstimate();
        update_poses(curr_estimate, pose_vec_aftopt);
        itd_vis->pubOdomPathAftopt(pose_vec_aftopt, ros_timestamp);

        ros_loop.sleep();
    }

    // Save the updated poses to a file
    FILE *fp_traj;
    std::string file_name = "Log/traj_aftopt.txt";
    std::string traj_dir = ROOT_DIR + file_name;
    fp_traj = fopen(traj_dir.c_str(), "w");
    assert(pose_timestamp.size() == pose_vec_aftopt.size());
    for (size_t i = 0; i < pose_timestamp.size(); i++)
    {
        fprintf(fp_traj, "%s,", std::to_string(pose_timestamp[i]).c_str());
        Eigen::Quaterniond q_(pose_vec_aftopt[i].rotation());
        Eigen::Vector3d t_(pose_vec_aftopt[i].translation());

        fprintf(fp_traj, "%f,", t_.x());
        fprintf(fp_traj, "%f,", t_.y());
        fprintf(fp_traj, "%f,", t_.z());

        fprintf(fp_traj, "%f,", q_.x());
        fprintf(fp_traj, "%f,", q_.y());
        fprintf(fp_traj, "%f,", q_.z());
        fprintf(fp_traj, "%f", q_.w());
        fprintf(fp_traj, "\r\n");
        fflush(fp_traj);
    }
    cout << "Traj file save to " << traj_dir << endl;

    return 0;
}