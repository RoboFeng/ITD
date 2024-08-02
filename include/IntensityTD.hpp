#pragma once
#include <ros/ros.h>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <condition_variable>
#include <unordered_map>

// eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

// ros
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// pcl
#define PCL_NO_PRECOMPILE
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

// cv2
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include <AutoExposure/image_processing.h>
#include <ikd-Tree/ikd_Tree.h>
#include <mypoint_type.h>
#include <gmm.hpp>
#include <voxel_hash.hpp>

using pixel_type = uint8_t;	  // Data type of intensity image
#define cv_pixel_type CV_8UC1 // Opencv data type of intensity image
typedef Eigen::Matrix<double, 3, Eigen::Dynamic> PointArray;

struct ITDFrame
{
	int index;						// kf_id
	int frame_id;					// frame_id
	double time_stamp;				// time_stamp
	Eigen::Affine3d lidar_pose;		// lidar pose
	PointCloudItdesc::Ptr itds_vec; // IntensityTD descriptor
	PointCloudGmm::Ptr keypoints;	// keypoints
	PointCloudXYZI::Ptr points_ori; // points_ori
	cv::Mat intensity_img;          // intensity_img
};

class IntensityTD
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	IntensityTD(ros::NodeHandle nh);
	~IntensityTD(){};

	/*************************Functions for Subframes*****************************/
	void fillVoid(cv::Mat &img);
	void autoExposure(cv::Mat &img, const bool first_flag);
	void removeStripNoise(cv::Mat &img);
	void detectCorners(const PointCloudXYZI::Ptr &ptr, const cv::Mat &inten_img, const int img_idx, PointCloudCorner::Ptr &corners);

	/*************************Functions for keyframes*****************************/
	void generateKeypoints(const PointCloudCorner::Ptr &corners, const vector<cv::Mat> &kf_imgs, PointCloudGmm::Ptr &keypoints); // Input: collected corner points output: keypoints (in world coordinate system)
	void generateItdesc(const int kfid, const PointCloudGmm::Ptr &keypoints, PointCloudItdesc::Ptr &itds_vec);					 // Input: keypoints (in keyframe LiDAR coordinate system) output: ITD descriptor (in keyframe LiDAR coordinate system)
	void addITDFrame(ITDFrame *&itd_frame);
	ITDFrame *getITDFrame(int index);

	bool detectLoop(ITDFrame *&itd_frame, std::pair<int, double> &loop_result,
					Eigen::Affine3d &loop_transform,
					std::vector<std::pair<PointItdesc, PointItdesc>> &loop_itd_pair,
					double &candidate_time, double &vertify_time); // Detect loop
	bool doICPRelative(const PointCloudXYZI::Ptr pcl_src, const PointCloudXYZI::Ptr pcl_tar, Eigen::Affine3d &pose_relative);

private:
	// LiDAR related parameters
	int height;								// Horizontal resolution
	int width;								// Number of lines
	ouster::viz::AutoExposure intensity_ae; // ae
	cv::Mat highpass_fir, lowpass_fir;

	int kf_frame_gap;
	float corner_fast_thres, corner_voxel_size, corner_nms_radius;
	float corner_blind_min, corner_blind_max;
	int descriptor_near_num, min_matched_num, min_loop_search_gap, kf_candidate_num, near_candidate_num;
	float descriptor_min_len, descriptor_max_len, rough_dis_threshold, descriptor_match_dis, relative_dis_threshold;

	vector<ITDFrame *> itdframe_list;
	KD_TREE<PointItdesc> itd_db; // ikdtree as database

	bool calItdescCell(const PointCloudCorner::Ptr &corners_in, const vector<cv::Mat> &kf_imgs, PointGmm &kp_out);
	float calKeypointMatchRatio(const PointGmm &curr_ptgmm, const PointGmm &prev_ptgmm);								 // Vertex GMM similarity estimation
	void solveSVD(const PointArray &points_ori, const PointArray &points_tar, Eigen::Vector3d &t, Eigen::Matrix3d &rot); // SVD
};

// Visualize point clouds, images, ITD, etc
class ITDVis
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ITDVis(ros::NodeHandle nh);
	// Publish odometer and trajectory
	void pubOdomPath(const Eigen::Affine3d &pose_in, double time_stamp);
	// Publish optimized trajectory of factor graph
	void pubOdomPathAftopt(const vector<Eigen::Affine3d> &poses_in, double time_stamp);
	// Publish point cloud
	void pubPclWorld(const PointCloudXYZI::Ptr &pcl_in, double time_stamp);
	// Publish keypoints
	void pubKeypoints(const PointCloudXYZI::Ptr &kps_in, double time_stamp);
	// Publish matched ITD
	void pubMatchedItds(const ITDFrame &itd_frame_curr, const ITDFrame &itd_frame_loop,
						const Eigen::Affine3d &pose_curr, const Eigen::Affine3d &pose_loop,
						const std::vector<std::pair<PointItdesc, PointItdesc>> &loop_itd_pair, double time_stamp);
	// Publish detected loop constraints
	void pubLoop(const Eigen::Affine3d &pose_curr, const Eigen::Affine3d &pose_loop, double time_stamp);
	// Publish intensity image
	void pubIntenImg(const cv::Mat img_in, double time_stamp);
	// Publish matched image
	void pubMatchedImg(const ITDFrame &itd_frame_curr, const ITDFrame &itd_frame_loop,
					   const std::vector<std::pair<PointItdesc, PointItdesc>> &loop_itd_pair, double time_stamp);

private:
	// Publish odometer
	ros::Publisher odom_publisher;
	// Publish trajectory
	nav_msgs::Path path;
	ros::Publisher path_publisher;
	// Publish optimized odometer and trajectory
	ros::Publisher odom_aftopt_publisher;
	ros::Publisher path_aftopt_publisher;
	// Publish point cloud
	int vis_filter_num = 4;
	ros::Publisher pcl_publisher;
	// Publish keypoints
	ros::Publisher keypoints_publisher;
	// Publish matched ITD
	ros::Publisher itd_publisher;
	// Publish detected loop constraints
	float vis_loop_constraints_scale;
	ros::Publisher loop_publisher; // Visual scaling factor, adjusted according to the trajectory display situation. ous18:(6.0 lw:4.0) ous22:(2.0 lw:1.5) fs(1.0 lw:0.5)
	visualization_msgs::MarkerArray LoopMarkerArray;
	// Publish intensity image
	image_transport::Publisher intenimg_publisher;
	// Publish matched image
	int height;								// Horizontal resolution
	int width;								// Number of lines
	Eigen::Vector2d project3Dto2D(Eigen::Vector3d point3d);
	image_transport::Publisher mimg_publisher;
};

class ITDLog
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	ITDLog(ros::NodeHandle nh);

	void logDumpPredictScore(int id1, int id2, double t1, double t2, float s1, float s2);
	void logDumpTimeCost(double timestamp, double t1, double t2, double t3, double t4, double t5);

private:
	// Save the score of loop detection for each keyframe
	bool log_predict_score_en;
	std::string log_predict_score_path;
	FILE *fp_predict_score;
	// Save the time consumption of each part of the algorithm
	bool log_time_cost_en;
	std::string log_time_cost_path;
	FILE *fp_time_cost;
};

static double time_inc(std::chrono::_V2::system_clock::time_point &t_end,
					   std::chrono::_V2::system_clock::time_point &t_begin)
{
	return std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_begin).count() * 1000;
}