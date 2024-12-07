#include <IntensityTD.hpp>

// 定义ITD显示颜色（山茶红）
#define CURR_ITD_R 237
#define CURR_ITD_G 85
#define CURR_ITD_B 106

// 定义ITD显示颜色（麦秆黄）
#define LOOP_ITD_R 248
#define LOOP_ITD_G 223
#define LOOP_ITD_B 112

// 定义连接线颜色（纯绿）
#define STLINE_R 0
#define STLINE_G 255
#define STLINE_B 0

ITDVis::ITDVis(ros::NodeHandle nh)
{
    nh.param<int>("width", width, 1024);
    nh.param<int>("height", height, 128);
    nh.param<int>("vis_filter_num", vis_filter_num, 4);
    nh.param<float>("vis_loop_constraints_scale", vis_loop_constraints_scale, 0.5);

    odom_publisher = nh.advertise<nav_msgs::Odometry>("/itd/pose", 100000);               // 发布优化前的位姿
    path_publisher = nh.advertise<nav_msgs::Path>("/itd/path", 100000);                   // 发布优化前的轨迹
    odom_aftopt_publisher = nh.advertise<nav_msgs::Odometry>("/itd/pose_aftopt", 100000); // 发布优化后的位姿
    path_aftopt_publisher = nh.advertise<nav_msgs::Path>("/itd/path_aftopt", 100000);     // 发布优化后的轨迹
    pcl_publisher = nh.advertise<sensor_msgs::PointCloud2>("/itd/points_world", 10);      // 发布注册到世界坐标系下的点云
    keypoints_publisher = nh.advertise<sensor_msgs::PointCloud2>("/itd/keypoints", 10);   // 发布注册到世界坐标系下的关键点
    image_transport::ImageTransport it1(nh);
    intenimg_publisher = it1.advertise("/itd/intensity_img", 100); // 发布强度图像

    itd_publisher = nh.advertise<visualization_msgs::MarkerArray>("/itd/matched_itds", 10);              // 发布匹配到的ITD
    loop_publisher = nh.advertise<visualization_msgs::MarkerArray>("/itd/loop_closure_constraints", 10); // 发布回环检测结果
    image_transport::ImageTransport it2(nh);
    mimg_publisher = it2.advertise("/itd/matched_img", 100); // 发布回环检测到的图像
}

void ITDVis::pubOdomPath(const Eigen::Affine3d &pose_in, double time_stamp)
{
    // 发布里程计位姿
    nav_msgs::Odometry pose_odom;
    Eigen::Quaterniond q_(pose_in.rotation());
    pose_odom.header.stamp = ros::Time().fromSec(time_stamp);
    pose_odom.header.frame_id = "camera_init";
    pose_odom.child_frame_id = "body";
    pose_odom.pose.pose.position.x = pose_in.translation().x();
    pose_odom.pose.pose.position.y = pose_in.translation().y();
    pose_odom.pose.pose.position.z = pose_in.translation().z();
    pose_odom.pose.pose.orientation.x = q_.x();
    pose_odom.pose.pose.orientation.y = q_.y();
    pose_odom.pose.pose.orientation.z = q_.z();
    pose_odom.pose.pose.orientation.w = q_.w();
    odom_publisher.publish(pose_odom);

    // 发布TF
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose_odom.pose.pose.position.x,
                                    pose_odom.pose.pose.position.y,
                                    pose_odom.pose.pose.position.z));
    q.setW(pose_odom.pose.pose.orientation.w);
    q.setX(pose_odom.pose.pose.orientation.x);
    q.setY(pose_odom.pose.pose.orientation.y);
    q.setZ(pose_odom.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, pose_odom.header.stamp, "camera_init", "body"));

    // 发布轨迹
    path.header.stamp = ros::Time().fromSec(time_stamp);
    path.header.frame_id = "camera_init";
    geometry_msgs::PoseStamped msg_pose_odom;
    msg_pose_odom.header = pose_odom.header;
    msg_pose_odom.pose = pose_odom.pose.pose;
    path.poses.push_back(msg_pose_odom);
    path_publisher.publish(path);
}

void ITDVis::pubOdomPathAftopt(const vector<Eigen::Affine3d> &poses_in, double time_stamp)
{
    // 发布里程计位姿
    Eigen::Affine3d pose_in = poses_in.back();
    Eigen::Quaterniond q_(pose_in.rotation());
    nav_msgs::Odometry pose_odom_aftopt;
    pose_odom_aftopt.header.stamp = ros::Time().fromSec(time_stamp);
    pose_odom_aftopt.header.frame_id = "camera_init";
    pose_odom_aftopt.child_frame_id = "body_aftopt";
    pose_odom_aftopt.pose.pose.position.x = pose_in.translation().x();
    pose_odom_aftopt.pose.pose.position.y = pose_in.translation().y();
    pose_odom_aftopt.pose.pose.position.z = pose_in.translation().z();
    pose_odom_aftopt.pose.pose.orientation.x = q_.x();
    pose_odom_aftopt.pose.pose.orientation.y = q_.y();
    pose_odom_aftopt.pose.pose.orientation.z = q_.z();
    pose_odom_aftopt.pose.pose.orientation.w = q_.w();
    odom_aftopt_publisher.publish(pose_odom_aftopt);

    // 发布TF
    static tf::TransformBroadcaster br_aftopt;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(pose_odom_aftopt.pose.pose.position.x,
                                    pose_odom_aftopt.pose.pose.position.y,
                                    pose_odom_aftopt.pose.pose.position.z));
    q.setW(pose_odom_aftopt.pose.pose.orientation.w);
    q.setX(pose_odom_aftopt.pose.pose.orientation.x);
    q.setY(pose_odom_aftopt.pose.pose.orientation.y);
    q.setZ(pose_odom_aftopt.pose.pose.orientation.z);
    transform.setRotation(q);
    br_aftopt.sendTransform(tf::StampedTransform(transform, pose_odom_aftopt.header.stamp, "camera_init", "body_aftopt"));

    // 发布轨迹
    nav_msgs::Path path_aftopt;
    path_aftopt.header.stamp = ros::Time().fromSec(time_stamp);
    path_aftopt.header.frame_id = "camera_init";
    // 遍历每个位姿，并索引对应的关键帧时间戳
    for (int i = 0; i < poses_in.size(); i++)
    {
        Eigen::Affine3d pose_ = poses_in[i];
        Eigen::Quaterniond qt_(pose_in.rotation());
        geometry_msgs::PoseStamped msg_pose_aftopt;
        msg_pose_aftopt.header.frame_id = "camera_init";
        msg_pose_aftopt.pose.position.x = pose_.translation().x();
        msg_pose_aftopt.pose.position.y = pose_.translation().y();
        msg_pose_aftopt.pose.position.z = pose_.translation().z();
        msg_pose_aftopt.pose.orientation.x = qt_.x();
        msg_pose_aftopt.pose.orientation.y = qt_.y();
        msg_pose_aftopt.pose.orientation.z = qt_.z();
        msg_pose_aftopt.pose.orientation.w = qt_.w();
        path_aftopt.poses.push_back(msg_pose_aftopt);
    }
    path_aftopt_publisher.publish(path_aftopt);
}

// 发布点云
void ITDVis::pubPclWorld(const PointCloudXYZI::Ptr &pcl_in, double time_stamp)
{
    // 对点云降采样进行显示
    PointCloudXYZI::Ptr pcl_dw(new PointCloudXYZI());
    for (int i = 0; i < pcl_in->points.size(); i++)
    {
        if (i % vis_filter_num == 0)
            pcl_dw->points.push_back(pcl_in->points[i]);
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pcl_dw, msg);
    msg.header.stamp = ros::Time().fromSec(time_stamp);
    msg.header.frame_id = "camera_init";
    pcl_publisher.publish(msg);
}

// 发布关键点
void ITDVis::pubKeypoints(const PointCloudXYZI::Ptr &kps_in, double time_stamp)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*kps_in, msg);
    msg.header.stamp = ros::Time().fromSec(time_stamp);
    msg.header.frame_id = "camera_init";
    keypoints_publisher.publish(msg);
}

// 发布匹配的ITD
void ITDVis::pubMatchedItds(const ITDFrame &itd_frame_curr, const ITDFrame &itd_frame_loop,
                            const Eigen::Affine3d &pose_curr, const Eigen::Affine3d &pose_loop,
                            const std::vector<std::pair<PointItdesc, PointItdesc>> &loop_itd_pair, double time_stamp)
{
    // 定义要发布的消息
    visualization_msgs::MarkerArray ma_line;
    visualization_msgs::Marker m_line;
    m_line.type = visualization_msgs::Marker::LINE_LIST;
    m_line.action = visualization_msgs::Marker::ADD;
    m_line.ns = "lines";
    // Don't forget to set the alpha!
    m_line.scale.x = 0.25;
    m_line.pose.orientation.w = 1.0;
    m_line.header.frame_id = "camera_init";
    m_line.header.stamp = ros::Time().fromSec(time_stamp);
    m_line.id = 0;

    for (auto &mp : loop_itd_pair)
    {
        geometry_msgs::Point p;

        // 提取所有顶点
        auto &p1a = itd_frame_curr.keypoints->points[mp.first.va_idx];
        Eigen::Vector3d p1a_t(p1a.x, p1a.y, p1a.z);
        auto &p1b = itd_frame_curr.keypoints->points[mp.first.vb_idx];
        Eigen::Vector3d p1b_t(p1b.x, p1b.y, p1b.z);
        auto &p1c = itd_frame_curr.keypoints->points[mp.first.vc_idx];
        Eigen::Vector3d p1c_t(p1c.x, p1c.y, p1c.z);
        auto &p2a = itd_frame_loop.keypoints->points[mp.second.va_idx];
        Eigen::Vector3d p2a_t(p2a.x, p2a.y, p2a.z);
        auto &p2b = itd_frame_loop.keypoints->points[mp.second.vb_idx];
        Eigen::Vector3d p2b_t(p2b.x, p2b.y, p2b.z);
        auto &p2c = itd_frame_loop.keypoints->points[mp.second.vc_idx];
        Eigen::Vector3d p2c_t(p2c.x, p2c.y, p2c.z);

        /********************显示curr的ITD********************/
        // 将点转换到世界坐标系
        p1a_t = pose_curr * p1a_t;
        p1b_t = pose_curr * p1b_t;
        p1c_t = pose_curr * p1c_t;

        m_line.color.a = 0.9;
        m_line.points.clear();
        m_line.color.r = CURR_ITD_R / 255.0;
        m_line.color.g = CURR_ITD_G / 255.0;
        m_line.color.b = CURR_ITD_B / 255.0;

        // side AB
        p.x = p1a_t.x();
        p.y = p1a_t.y();
        p.z = p1a_t.z();
        m_line.points.push_back(p);
        p.x = p1b_t.x();
        p.y = p1b_t.y();
        p.z = p1b_t.z();
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        // side BC
        p.x = p1b_t.x();
        p.y = p1b_t.y();
        p.z = p1b_t.z();
        m_line.points.push_back(p);
        p.x = p1c_t.x();
        p.y = p1c_t.y();
        p.z = p1c_t.z();
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        // side AC
        p.x = p1a_t.x();
        p.y = p1a_t.y();
        p.z = p1a_t.z();
        m_line.points.push_back(p);
        p.x = p1c_t.x();
        p.y = p1c_t.y();
        p.z = p1c_t.z();
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();

        /********************显示loop的ITD********************/
        // 将点转换到世界坐标系
        p2a_t = pose_loop * p2a_t;
        p2b_t = pose_loop * p2b_t;
        p2c_t = pose_loop * p2c_t;

        m_line.points.clear();
        m_line.color.r = LOOP_ITD_R / 255.0;
        m_line.color.g = LOOP_ITD_G / 255.0;
        m_line.color.b = LOOP_ITD_B / 255.0;

        // side AB
        p.x = p2a_t.x();
        p.y = p2a_t.y();
        p.z = p2a_t.z();
        m_line.points.push_back(p);
        p.x = p2b_t.x();
        p.y = p2b_t.y();
        p.z = p2b_t.z();
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        // side BC
        p.x = p2b_t.x();
        p.y = p2b_t.y();
        p.z = p2b_t.z();
        m_line.points.push_back(p);
        p.x = p2c_t.x();
        p.y = p2c_t.y();
        p.z = p2c_t.z();
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
        // side AC
        p.x = p2a_t.x();
        p.y = p2a_t.y();
        p.z = p2a_t.z();
        m_line.points.push_back(p);
        p.x = p2c_t.x();
        p.y = p2c_t.y();
        p.z = p2c_t.z();
        m_line.points.push_back(p);
        ma_line.markers.push_back(m_line);
        m_line.id++;
        m_line.points.clear();
    }

    int marker_num = ma_line.markers.size();

    // 删除marker
    for (int j = 0; j < marker_num; j++)
    {
        m_line.color.a = 0.00;
        ma_line.markers.push_back(m_line);
        m_line.id++;
    }

    itd_publisher.publish(ma_line);
    m_line.id = 0;
    ma_line.markers.clear();
}

// 发布检测到的回环约束
void ITDVis::pubLoop(const Eigen::Affine3d &pose_curr, const Eigen::Affine3d &pose_loop, double time_stamp)
{
    static int loop_node_id = 0;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "camera_init"; // camera_init
    markerNode.header.stamp = ros::Time().fromSec(time_stamp);
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = loop_node_id;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 2 * vis_loop_constraints_scale;
    markerNode.scale.y = 2 * vis_loop_constraints_scale;
    markerNode.scale.z = 2 * vis_loop_constraints_scale;
    markerNode.color.r = 0.0 / 255.0;
    markerNode.color.g = 255.0 / 255.0;
    markerNode.color.b = 0.0 / 255.0;
    markerNode.color.a = 1;
    loop_node_id++;

    // 闭环边
    static int loop_edge_id = 0;
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = "camera_init";
    markerEdge.header.stamp = ros::Time().fromSec(time_stamp);
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = loop_edge_id;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.6 * vis_loop_constraints_scale;
    markerEdge.color.r = STLINE_R / 255.0;
    markerEdge.color.g = STLINE_G / 255.0;
    markerEdge.color.b = STLINE_B / 255.0;
    markerEdge.color.a = 1;
    loop_edge_id++;

    geometry_msgs::Point p;
    p.x = pose_curr.translation().x();
    p.y = pose_curr.translation().y();
    p.z = pose_curr.translation().z();
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);
    p.x = pose_loop.translation().x();
    p.y = pose_loop.translation().y();
    p.z = pose_loop.translation().z();
    markerNode.points.push_back(p);
    markerEdge.points.push_back(p);

    LoopMarkerArray.markers.push_back(markerNode);
    LoopMarkerArray.markers.push_back(markerEdge);
    loop_publisher.publish(LoopMarkerArray);
}

// 发布强度图像
void ITDVis::pubIntenImg(const cv::Mat img_in, double time_stamp)
{
    cv::Mat img_show;
    cv::cvtColor(img_in, img_show, CV_GRAY2RGB);
    // 发布图像
    sensor_msgs::ImagePtr msg;
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_show).toImageMsg();
    intenimg_publisher.publish(msg);
}

Eigen::Vector2d ITDVis::project3Dto2D(Eigen::Vector3d point3d)
{
    const float PI_INV = 1.0f / M_PI;
    const float beam_angle_up = 21.25 * M_PI / 180;
    const float beam_angle_down = 21.38 * M_PI / 180;
    const float BEAM_ANGLE_INV = 1 / (beam_angle_up + beam_angle_down);

    Eigen::Vector2d point2d;
    // u
    point2d[0] = 0.5 * width * (1 - PI_INV * atan2f(point3d[1], point3d[0])) + 12.0; // 1024:12 2048:24

    if (point2d[0] > width)
        point2d[0] -= width;
    if (point2d[0] < 0)
        point2d[0] += width;
    // v
    const float L = sqrtf(point3d[0] * point3d[0] + point3d[1] * point3d[1]);
    const float range = sqrtf(point3d[2] * point3d[2] + L * L);
    const float alt_angle = asinf(point3d[2] / range);

    point2d[1] = (beam_angle_up - alt_angle) * BEAM_ANGLE_INV * height;
    return point2d;
}

// 发布匹配的图像
void ITDVis::pubMatchedImg(const ITDFrame &itd_frame_curr, const ITDFrame &itd_frame_loop,
                           const std::vector<std::pair<PointItdesc, PointItdesc>> &loop_itd_pair, double time_stamp)
{
    // 将两帧图片拼接起来
    cv::Mat img_show;
    cv::vconcat(itd_frame_curr.intensity_img, itd_frame_loop.intensity_img, img_show);
    cv::cvtColor(img_show, img_show, CV_GRAY2RGB);

    // 遍历STD匹配对index，并索引相应数据进行显示
    for (auto &mp : loop_itd_pair)
    {
        // STD在两张图像上都能显示全才进行显示
        bool need_show = true;
        vector<cv::Point2f> curr_pts2d;
        vector<cv::Point2f> prev_pts2d;
        curr_pts2d.resize(3);
        prev_pts2d.resize(3);

        for (int idx = 0; idx < 3; idx++)
        {
            // 获取curr_kf的角点投影
            auto &curr_kp = itd_frame_curr.keypoints->points[mp.first.v_idx[idx]];
            Eigen::Vector3d curr_pt3d(curr_kp.x, curr_kp.y, curr_kp.z);
            Eigen::Vector2d curr_pt2d = project3Dto2D(curr_pt3d);
            if (curr_pt2d[1] < 0 || curr_pt2d[1] > height - 1)
            {
                need_show = false;
                break;
            }
            cv::Point2f curr_pt2d_show(floor(curr_pt2d[0]), floor(curr_pt2d[1]));
            curr_pts2d[idx] = curr_pt2d_show;

            // 获取prev_kf的角点投影
            auto &prev_kp = itd_frame_loop.keypoints->points[mp.second.v_idx[idx]];
            Eigen::Vector3d prev_pt3d(prev_kp.x, prev_kp.y, prev_kp.z);
            Eigen::Vector2d prev_pt2d = project3Dto2D(prev_pt3d);
            if (prev_pt2d[1] < 0 || prev_pt2d[1] > height - 1)
            {
                need_show = false;
                break;
            }
            cv::Point2f prev_pt2d_show(floor(prev_pt2d[0]), floor(prev_pt2d[1]));
            // 因为拼接图像所以垂直方向坐标加height
            prev_pt2d_show.y += height;
            prev_pts2d[idx] = prev_pt2d_show;
        }

        if (need_show)
        {
            // 绘制curr_kf的描述子
            cv::line(img_show, curr_pts2d[0], curr_pts2d[1], cv::Scalar(CURR_ITD_B, CURR_ITD_G, CURR_ITD_R), 2);
            cv::line(img_show, curr_pts2d[1], curr_pts2d[2], cv::Scalar(CURR_ITD_B, CURR_ITD_G, CURR_ITD_R), 2);
            cv::line(img_show, curr_pts2d[2], curr_pts2d[0], cv::Scalar(CURR_ITD_B, CURR_ITD_G, CURR_ITD_R), 2);

            // 绘制prev_kf的描述子
            cv::line(img_show, prev_pts2d[0], prev_pts2d[1], cv::Scalar(LOOP_ITD_B, LOOP_ITD_G, LOOP_ITD_R), 2);
            cv::line(img_show, prev_pts2d[1], prev_pts2d[2], cv::Scalar(LOOP_ITD_B, LOOP_ITD_G, LOOP_ITD_R), 2);
            cv::line(img_show, prev_pts2d[2], prev_pts2d[0], cv::Scalar(LOOP_ITD_B, LOOP_ITD_G, LOOP_ITD_R), 2);

            // 绘制描述子之间的连接线
            cv::line(img_show, curr_pts2d[0], prev_pts2d[0], cv::Scalar(STLINE_B, STLINE_G, STLINE_R), 1);
            cv::line(img_show, curr_pts2d[1], prev_pts2d[1], cv::Scalar(STLINE_B, STLINE_G, STLINE_R), 1);
            cv::line(img_show, curr_pts2d[2], prev_pts2d[2], cv::Scalar(STLINE_B, STLINE_G, STLINE_R), 1);
        }
    }

    // 在图像上显示kf标号
    int text_height_pos = 25;
    double text_scale = 0.8;
    int text_thickness = 1;
    cv::Scalar text_color = cv::Scalar(255, 255, 255);
    cv::putText(img_show, "kf-id: " + to_string(itd_frame_curr.index),
                cv::Point2f(5, text_height_pos),
                cv::FONT_HERSHEY_SIMPLEX, text_scale, text_color, text_thickness);
    cv::putText(img_show, "kf-id: " + to_string(itd_frame_loop.index),
                cv::Point2f(5, text_height_pos + height),
                cv::FONT_HERSHEY_SIMPLEX, text_scale, text_color, text_thickness);

    // 发布图像
    sensor_msgs::ImagePtr mimg_msg;
    mimg_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_show).toImageMsg();
    mimg_publisher.publish(mimg_msg);
}