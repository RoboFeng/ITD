#include <IntensityTD.hpp>

// Read parameters from ROS
IntensityTD::IntensityTD(ros::NodeHandle nh)
{
    nh.param<int>("width", width, 1024);
    nh.param<int>("height", height, 128);
    nh.param<int>("kf_frame_gap", kf_frame_gap, 10);
    vector<double> hpf;
    vector<double> lpf;
    nh.param<vector<double>>("highpass_fir", hpf, vector<double>());
    nh.param<vector<double>>("lowpass_fir", lpf, vector<double>());
    highpass_fir = cv::Mat(hpf).clone();
    lowpass_fir = cv::Mat(lpf).clone();

    nh.param<float>("corner_fast_thres", corner_fast_thres, 110.0);
    nh.param<float>("corner_blind_min", corner_blind_min, 2.0);
    nh.param<float>("corner_blind_max", corner_blind_max, 50.0);
    nh.param<float>("corner_voxel_size", corner_voxel_size, 0.3);
    nh.param<float>("corner_nms_radius", corner_nms_radius, 1.0);

    nh.param<int>("descriptor_near_num", descriptor_near_num, 10);
    nh.param<float>("descriptor_min_len", descriptor_min_len, 2.0);
    nh.param<float>("descriptor_max_len", descriptor_max_len, 20.0);

    nh.param<int>("min_loop_search_gap", min_loop_search_gap, 50);
    nh.param<int>("kf_candidate_num", kf_candidate_num, 10);
    nh.param<int>("near_candidate_num", near_candidate_num, 10);
    nh.param<int>("min_matched_num", min_matched_num, 10);
    nh.param<float>("rough_dis_threshold", rough_dis_threshold, 0.007);
    nh.param<float>("descriptor_match_dis", descriptor_match_dis, 1.0);
    nh.param<float>("relative_dis_threshold", relative_dis_threshold, 10.0);
}

// Using local median to fill image gaps
void IntensityTD::fillVoid(cv::Mat &img)
{
    for (size_t v = 1; v < img.rows - 1; v++)
        for (size_t u = 1; u < img.cols - 1; u++)
        {
            if (img.at<pixel_type>(v, u) == 0)
            {
                vector<pixel_type> near_pixels;
                // Save the pixel values of the current pixel neighborhood
                for (int nv = -1; nv <= 1; nv++)
                    for (int nu = -1; nu <= 1; nu++)
                        near_pixels.emplace_back(img.at<pixel_type>(v + nv, u + nu));
                sort(near_pixels.begin(), near_pixels.end());
                img.at<pixel_type>(v, u) = near_pixels[4]; // Fill in the median value
            }
        }
}

// Automatic exposure of images (ouster)
void IntensityTD::autoExposure(cv::Mat &img, const bool first_flag)
{
    auto image_map = Eigen::Map<img_t<pixel_type>>(
        (pixel_type *)img.data, height, width);
    img_t<double> image_eigen(height, width);
    for (size_t v = 0; v < height; v++)
        for (size_t u = 0; u < width; u++)
        {
            image_eigen(v, u) = img.at<pixel_type>(v, u);
        }
    intensity_ae(image_eigen, first_flag);
    image_map = (image_eigen * 255.0).cast<pixel_type>();
}

void IntensityTD::removeStripNoise(cv::Mat &img)
{
    cv::Mat img_tmp = img.clone();
    cv::filter2D(img_tmp, img_tmp, -1, highpass_fir);
    cv::filter2D(img_tmp, img_tmp, -1, lowpass_fir);
    img = img - img_tmp;
}

// Detecting corner points in images
void IntensityTD::detectCorners(const PointCloudXYZI::Ptr &ptr, const cv::Mat &inten_img, const int img_idx, PointCloudCorner::Ptr &corners)
{
    vector<cv::Point2f> good_pt2;
    std::vector<cv::KeyPoint> keypoints;
    cv::AGAST(inten_img, keypoints, corner_fast_thres);

    good_pt2.reserve(keypoints.size());
    for (auto &kp : keypoints)
    {
        good_pt2.emplace_back(kp.pt);
    }
    corners->clear();
    corners->resize(good_pt2.size());

    int n = 0;
    for (auto pt2 : good_pt2)
    {
        // Determine whether the extracted feature points are located at the boundaries of the image
        if (pt2.x < 10 || pt2.x > width - 10 || pt2.y < 5 || pt2.y > height - 5)
            continue;
        const int idx = round(pt2.y) * width + round(pt2.x);
        auto &point_sel = ptr->points[idx];
        float range = sqrtf(point_sel.x * point_sel.x + point_sel.y * point_sel.y + point_sel.z * point_sel.z);

        if (range < corner_blind_min || range > corner_blind_max)
            continue;

        corners->points[n].x = point_sel.x;
        corners->points[n].y = point_sel.y;
        corners->points[n].z = point_sel.z;
        corners->points[n].t = point_sel.curvature;
        corners->points[n].imgidx = img_idx;
        corners->points[n].u = round(pt2.x);
        corners->points[n].v = round(pt2.y);
        n++;
    }
    corners->resize(n);
}

// Calculate representative points based on the corner points in each voxel
bool IntensityTD::calItdescCell(const PointCloudCorner::Ptr &corners_in, const vector<cv::Mat> &kf_imgs, PointGmm &kp_out)
{
    std::vector<float> refs;
    int corners_size = corners_in->points.size();
    refs.reserve(corners_size * 9);

    kp_out.x = 0;
    kp_out.y = 0;
    kp_out.z = 0;
    // Calculate the coordinates of a point (average of points within the voxel)
    for (int idx = 0; idx < corners_size; idx++)
    {
        auto &corner_ = corners_in->points[idx];
        kp_out.x += corner_.x;
        kp_out.y += corner_.y;
        kp_out.z += corner_.z;
        for (int pixu = corner_.u - 1; pixu < corner_.u + 2; pixu++)
            for (int pixv = corner_.v - 1; pixv < corner_.v + 2; pixv++)
            {
                float pix_value = kf_imgs[corner_.imgidx].at<pixel_type>(pixv, pixu);
                refs.emplace_back(pix_value);
            }
    }
    if (refs.empty())
        return false;

    kp_out.x = kp_out.x / corners_size;
    kp_out.y = kp_out.y / corners_size;
    kp_out.z = kp_out.z / corners_size;

    GaussianMixture gmm;
    gmm.fit(refs);

    kp_out.refmean[0] = gmm.means_(0);
    kp_out.refmean[1] = gmm.means_(1);
    kp_out.refmean[2] = gmm.means_(2);
    kp_out.refvar[0] = gmm.covariances_(0);
    kp_out.refvar[1] = gmm.covariances_(1);
    kp_out.refvar[2] = gmm.covariances_(2);
    kp_out.refweight[0] = gmm.weights_(0);
    kp_out.refweight[1] = gmm.weights_(1);
    kp_out.refweight[2] = gmm.weights_(2);
    // Calculate the decision factor (the more specific the point, the better)
    kp_out.refquality = 0.0f;
    for (int i = 0; i < 3; i++)
    {
        if (isnan(kp_out.refvar[i]))
        {
            kp_out.refquality += 1.0f;
        }
        else
        {
            kp_out.refquality += kp_out.refweight[i] * sqrtf(kp_out.refvar[i]);
        }
    }
    kp_out.refquality = kp_out.refquality * (1.5f - corners_size / kf_frame_gap);
    return true;
}

void IntensityTD::generateKeypoints(const PointCloudCorner::Ptr &corners, const vector<cv::Mat> &kf_imgs, PointCloudGmm::Ptr &keypoints)
{
    std::unordered_map<VOXEL_LOC, PointCloudCorner::Ptr> voxel_kf_map; // Create voxel maps that collect key points
    float voxel_size = corner_voxel_size;

    // Add all points to voxel map
    for (int i = 0; i < corners->size(); i++)
    {
        auto &p_c = corners->points[i];

        float loc_xyz[3];
        loc_xyz[0] = p_c.x < 0 ? p_c.x / voxel_size - 1 : p_c.x / voxel_size;
        loc_xyz[1] = p_c.y < 0 ? p_c.y / voxel_size - 1 : p_c.y / voxel_size;
        loc_xyz[2] = p_c.z < 0 ? p_c.z / voxel_size - 1 : p_c.z / voxel_size;

        VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                           (int64_t)loc_xyz[2]);
        auto iter = voxel_kf_map.find(position);
        if (iter != voxel_kf_map.end())
        {
            voxel_kf_map[position]->points.push_back(p_c);
        }
        else
        {
            PointCloudCorner::Ptr corners_ptr = boost::make_shared<PointCloudCorner>();
            corners_ptr->points.push_back(p_c);
            voxel_kf_map[position] = corners_ptr;
        }
    }

    vector<PointCloudCorner::Ptr> corners_vec;
    corners_vec.reserve(voxel_kf_map.size());
    for (auto iter = voxel_kf_map.begin(); iter != voxel_kf_map.end(); ++iter)
    {
        if (iter->second->points.size() > 3)
        {
            corners_vec.emplace_back(iter->second);
        }
    }

    keypoints->clear();
    keypoints->resize(corners_vec.size());

#ifdef MP_EN
    omp_set_num_threads(4);
#pragma omp parallel for
#endif
    for (int i = 0; i < corners_vec.size(); i++)
    {
        PointGmm kp_cell;
        calItdescCell(corners_vec[i], kf_imgs, kp_cell);
        keypoints->points[i] = kp_cell;
    }

    // cout << "******************" << endl;
    // cout << "dw corners_size_before:" << keypoints.size() << endl;

    // Build a kd tree map, traverse the corners in space again, and perform non maximum suppression
    PointCloudGmm::Ptr pcl_tmp(new PointCloudGmm());
    pcl::copyPointCloud(*keypoints, *pcl_tmp);
    keypoints->clear();
    keypoints->reserve(pcl_tmp->points.size());

    pcl::KdTreeFLANN<PointGmm> kdtree_nms;
    kdtree_nms.setInputCloud(pcl_tmp);
    for (int i = 0; i < pcl_tmp->points.size(); i++)
    {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        // Start radius search
        if (kdtree_nms.radiusSearch(pcl_tmp->points[i], corner_nms_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            bool add_flag = true;
            for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
            {
                if (pcl_tmp->points[i].refquality > pcl_tmp->points[pointIdxRadiusSearch[j]].refquality)
                {
                    add_flag = false;
                }
            }
            if (add_flag)
                keypoints->points.push_back(pcl_tmp->points[i]);
        }
        else
        {
            keypoints->points.push_back(pcl_tmp->points[i]);
        }
    }
    // cout << "dw corners_size:" << keypoints.size() << endl;
}

void IntensityTD::generateItdesc(const int kfid, const PointCloudGmm::Ptr &keypoints, PointCloudItdesc::Ptr &itds_vec)
{
    itds_vec->clear();
    int near_num = descriptor_near_num;
    double max_dis_threshold = descriptor_max_len;
    double min_dis_threshold = descriptor_min_len;

    std::unordered_map<STDesc_LOC, bool> feat_map;
    pcl::KdTreeFLANN<PointGmm>::Ptr kd_tree(
        new pcl::KdTreeFLANN<PointGmm>);
    kd_tree->setInputCloud(keypoints);
    std::vector<int> pointIdxNKNSearch(near_num);         // Index of nearest neighbor points
    std::vector<float> pointNKNSquaredDistance(near_num); // The distance between nearest neighbors

    // Search N nearest keypoints to form stds.
    for (size_t i = 0; i < keypoints->size(); i++)
    {
        PointGmm searchPoint = keypoints->points[i];
        if (kd_tree->nearestKSearch(searchPoint, near_num, pointIdxNKNSearch,
                                    pointNKNSquaredDistance) > 0)
        {
            for (int m = 1; m < near_num - 1; m++)
            {
                for (int n = m + 1; n < near_num; n++)
                {
                    PointGmm p1 = searchPoint;
                    PointGmm p2 = keypoints->points[pointIdxNKNSearch[m]];
                    PointGmm p3 = keypoints->points[pointIdxNKNSearch[n]];
                    double a = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) +
                                    pow(p1.z - p2.z, 2));
                    double b = sqrt(pow(p1.x - p3.x, 2) + pow(p1.y - p3.y, 2) +
                                    pow(p1.z - p3.z, 2));
                    double c = sqrt(pow(p3.x - p2.x, 2) + pow(p3.y - p2.y, 2) +
                                    pow(p3.z - p2.z, 2));
                    if (a > max_dis_threshold || b > max_dis_threshold ||
                        c > max_dis_threshold || a < min_dis_threshold ||
                        b < min_dis_threshold || c < min_dis_threshold)
                    {
                        continue;
                    }
                    // re-range the vertex by the side length
                    double temp;
                    Eigen::Vector3i l1, l2, l3;
                    Eigen::Vector3i l_temp;
                    l1 << 1, 2, 0;
                    l2 << 1, 0, 3;
                    l3 << 0, 2, 3;
                    if (a > b)
                    {
                        temp = a;
                        a = b;
                        b = temp;
                        l_temp = l1;
                        l1 = l2;
                        l2 = l_temp;
                    }
                    if (b > c)
                    {
                        temp = b;
                        b = c;
                        c = temp;
                        l_temp = l2;
                        l2 = l3;
                        l3 = l_temp;
                    }
                    if (a > b)
                    {
                        temp = a;
                        a = b;
                        b = temp;
                        l_temp = l1;
                        l1 = l2;
                        l2 = l_temp;
                    }
                    // check augnmentation
                    pcl::PointXYZ d_p;
                    d_p.x = a * 1000;
                    d_p.y = b * 1000;
                    d_p.z = c * 1000;
                    STDesc_LOC position((int64_t)d_p.x, (int64_t)d_p.y, (int64_t)d_p.z);
                    auto iter = feat_map.find(position);
                    if (iter == feat_map.end())
                    {
                        uint16_t va_idx;
                        uint16_t vb_idx;
                        uint16_t vc_idx;
                        if (l1[0] == l2[0])
                        {
                            va_idx = i;
                        }
                        else if (l1[1] == l2[1])
                        {
                            va_idx = pointIdxNKNSearch[m];
                        }
                        else
                        {
                            va_idx = pointIdxNKNSearch[n];
                        }
                        if (l1[0] == l3[0])
                        {
                            vb_idx = i;
                        }
                        else if (l1[1] == l3[1])
                        {
                            vb_idx = pointIdxNKNSearch[m];
                        }
                        else
                        {
                            vb_idx = pointIdxNKNSearch[n];
                        }
                        if (l2[0] == l3[0])
                        {
                            vc_idx = i;
                        }
                        else if (l2[1] == l3[1])
                        {
                            vc_idx = pointIdxNKNSearch[m];
                        }
                        else
                        {
                            vc_idx = pointIdxNKNSearch[n];
                        }
                        PointItdesc single_descriptor;
                        single_descriptor.x = a;
                        single_descriptor.y = b;
                        single_descriptor.z = c;
                        single_descriptor.kfid = kfid;
                        single_descriptor.v_idx[0] = va_idx;
                        single_descriptor.v_idx[1] = vb_idx;
                        single_descriptor.v_idx[2] = vc_idx;
                        feat_map[position] = true;
                        itds_vec->push_back(single_descriptor);
                    }
                }
            }
        }
    }
}

void IntensityTD::addITDFrame(ITDFrame *&itd_frame)
{
    itdframe_list.emplace_back(itd_frame);
    if (itd_db.Root_Node == nullptr)
    {
        if (itd_frame->itds_vec->points.size() > 5)
        {
            itd_db.Build(itd_frame->itds_vec->points);
        }
    }
    else
        itd_db.Add_Points(itd_frame->itds_vec->points, false);
}

ITDFrame *IntensityTD::getITDFrame(int index)
{
    if (index >= itdframe_list.size())
    {
        ROS_ERROR("wrong itdframe query index!");
        exit(EXIT_FAILURE);
    }
    return itdframe_list[index];
}

float IntensityTD::calKeypointMatchRatio(const PointGmm &curr_ptgmm, const PointGmm &prev_ptgmm)
{
    float match_ratio = 0.0f;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
            float refweight_mlt = curr_ptgmm.refweight[i] * prev_ptgmm.refweight[j];
            if (refweight_mlt < 0.001f)
                continue;

            float max_thres = sqrtf(fmaxf(curr_ptgmm.refvar[i], prev_ptgmm.refvar[j]));
            if (fabsf(curr_ptgmm.refmean[i] - prev_ptgmm.refmean[j]) < (max_thres > 3.0f ? max_thres : 3.0f))
                match_ratio += refweight_mlt;
        }
    return match_ratio;
}

void IntensityTD::solveSVD(const PointArray &points_ori, const PointArray &points_tar, Eigen::Vector3d &t, Eigen::Matrix3d &rot)
{
    Eigen::Matrix<double, 3, 1> mean_ori = points_ori.rowwise().mean();
    Eigen::Matrix<double, 3, 1> mean_tar = points_tar.rowwise().mean();
    Eigen::Matrix3d covariance = (points_ori.colwise() - mean_ori) * (points_tar.colwise() - mean_tar).transpose();
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(covariance, Eigen::ComputeThinU |
                                                          Eigen::ComputeThinV);
    Eigen::Matrix3d V = svd.matrixV();
    Eigen::Matrix3d U = svd.matrixU();
    rot = V * U.transpose();
    if (rot.determinant() < 0)
    {
        Eigen::Matrix3d K;
        K << 1, 0, 0, 0, 1, 0, 0, 0, -1;
        rot = V * K * U.transpose();
    }
    t = -rot * mean_ori + mean_tar;
}

bool IntensityTD::detectLoop(ITDFrame *&itd_frame, std::pair<int, double> &loop_result,
                             Eigen::Affine3d &loop_transform,
                             std::vector<std::pair<PointItdesc, PointItdesc>> &loop_itd_pair,
                             double &candidate_time, double &vertify_time)
{
    loop_result.first = -1;
    loop_result.second = -1;
    if (itd_frame->index < min_loop_search_gap)
        return false;
    vector<vector<std::pair<PointItdesc, PointItdesc>>> match_candidates; // Create an index to collect candidate matching descriptors
    match_candidates.resize(itdframe_list.size());
    double match_array[MAX_FRAME_N] = {0}; // Voting Results

    int curr_kfid = itd_frame->index;
    PointCloudItdesc::Ptr &itds_vec = itd_frame->itds_vec;

    auto t1 = std::chrono::high_resolution_clock::now();

    vector<vector<std::pair<PointItdesc, PointItdesc>>> cell_candidates_vec;
    cell_candidates_vec.resize(itds_vec->points.size());

#ifdef MP_EN
    omp_set_num_threads(6);
#pragma omp parallel for
#endif
    for (size_t i = 0; i < itds_vec->points.size(); i++)
    {
        auto &curr_std = itds_vec->points[i];
        Eigen::Vector3d curr_sidelen(curr_std.x, curr_std.y, curr_std.z);
        double dis = curr_sidelen.norm() * rough_dis_threshold;
        PointVectorItdesc nearest_stds;
        itd_db.Radius_Search(curr_std, dis, nearest_stds);
        // vector<float> stdSearchSqDis(2 * near_candidate_num);
        // itd_db.Nearest_Search(curr_std, 2 * near_candidate_num, nearest_stds, stdSearchSqDis, dis);
        // Collect scores for the current nearest neighbor search
        vector<std::pair<float, PointItdesc>> cell_candidates;
        for (int j = 0; j < nearest_stds.size(); j++)
        {
            int prev_kfid = nearest_stds[j].kfid;
            if (curr_kfid - prev_kfid < min_loop_search_gap)
                continue;
            bool match_success = true;
            float ref_match_score = 0.0f;
            ITDFrame *prev_kf = itdframe_list[prev_kfid];
            for (int idx = 0; idx < 3; idx++)
            {
                // Extract the corresponding three vertices
                auto &curr_ptgmm = itd_frame->keypoints->points[curr_std.v_idx[idx]];
                auto &prev_ptgmm = prev_kf->keypoints->points[nearest_stds[j].v_idx[idx]];
                float ref_match_ratio = calKeypointMatchRatio(curr_ptgmm, prev_ptgmm);
                // cout << "ref_match_ratio:" << ref_match_ratio << endl;
                // As long as one vertex is not correctly matched, it is considered a match failure
                if (ref_match_ratio < 0.001f)
                {
                    match_success = false;
                    break;
                }
                ref_match_score += ref_match_ratio;
            }
            if (!match_success)
                continue;
            cell_candidates.emplace_back(std::pair<float, PointItdesc>(ref_match_score, nearest_stds[j]));
        }
        sort(cell_candidates.begin(), cell_candidates.end(), [&](std::pair<float, PointItdesc> &a, std::pair<float, PointItdesc> &b)
             { return a.first > b.first; });

        int picked = 0;
        for (auto &ccd : cell_candidates)
        {
            std::pair<PointItdesc, PointItdesc> mp_;
            mp_.first = curr_std;
            mp_.second = ccd.second;
            cell_candidates_vec[i].emplace_back(mp_);
            picked++;
            if (picked > near_candidate_num)
                break;
        }
    }

    // Vote for the currently matched frame
    for (int i = 0; i < itds_vec->points.size(); i++)
    {
        if (!cell_candidates_vec[i].empty())
        {
            for (auto &ccd : cell_candidates_vec[i])
            {
                match_array[ccd.second.kfid] += 1;
                match_candidates[ccd.second.kfid].emplace_back(ccd);
            }
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    float best_match_error = 0.0f;
    int best_vote_num = 0;
    int best_prev_index = -1;
    vector<std::pair<PointItdesc, PointItdesc>> best_itdpairs;
    Eigen::Affine3d best_transform;

    for (int cnt = 0; cnt < kf_candidate_num; cnt++)
    {
        double max_vote = 1;
        int max_vote_index = -1;
        // Find the frame with the highest number of votes
        for (int i = 0; i < MAX_FRAME_N; i++)
        {
            if (match_array[i] > max_vote)
            {
                max_vote = match_array[i];
                max_vote_index = i;
            }
        }
        if (max_vote_index >= 0 && max_vote >= 5 && max_vote > best_vote_num)
        {
            match_array[max_vote_index] = 0; // Clear the number of votes to obtain other suboptimal matching frames

            if (!match_candidates[max_vote_index].empty())
            {
                // Verify the current candidate matching frame
                ITDFrame *prev_kf = itdframe_list[max_vote_index];

                // Place all candidate feature subsets in an array one by one according to the index
                PointArray points_ori; // prev
                PointArray points_tar; // curr
                points_ori.resize(3, match_candidates[max_vote_index].size() * 3);
                points_tar.resize(3, match_candidates[max_vote_index].size() * 3);
                for (int i = 0; i < match_candidates[max_vote_index].size(); i++)
                {
                    for (int idx = 0; idx < 3; idx++)
                    {
                        auto &pt_ori = prev_kf->keypoints->points[match_candidates[max_vote_index][i].second.v_idx[idx]];
                        auto &pt_tar = itd_frame->keypoints->points[match_candidates[max_vote_index][i].first.v_idx[idx]];
                        points_ori.block<3, 1>(0, 3 * i + idx) << pt_ori.x, pt_ori.y, pt_ori.z;
                        points_tar.block<3, 1>(0, 3 * i + idx) << pt_tar.x, pt_tar.y, pt_tar.z;
                    }
                }

                vector<vector<int>> vote_list_vec;
                vote_list_vec.resize(match_candidates[max_vote_index].size());
#ifdef MP_EN
                omp_set_num_threads(3);
#pragma omp parallel for 
#endif
                for (int iter = 0; iter < match_candidates[max_vote_index].size(); iter++)
                {
                    Eigen::Vector3d t;
                    Eigen::Matrix3d rot;
                    vector<int> vote_list;
                    vote_list.reserve(match_candidates[max_vote_index].size());

                    solveSVD(points_ori.block<3, 3>(0, iter * 3), points_tar.block<3, 3>(0, iter * 3), t, rot);

                    for (int i = 0; i < match_candidates[max_vote_index].size(); i++)
                    {
                        if (((points_tar.block<3, 3>(0, i * 3) - ((rot * points_ori.block<3, 3>(0, i * 3)).colwise() + t)).colwise().norm()).norm() < descriptor_match_dis)
                        {
                            vote_list.emplace_back(i);
                        }
                    }
                    vote_list_vec[iter].swap(vote_list);
                }

                vector<int> best_vote_list;
                int best_vote_num_vec = 0;
                for (int i = 0; i < vote_list_vec.size(); i++)
                {
                    if (vote_list_vec[i].size() > best_vote_num_vec)
                    {
                        best_vote_list.swap(vote_list_vec[i]);
                        best_vote_num_vec = vote_list_vec[i].size();
                    }
                }

                // TODO：Check the distribution of matched feature points in space to prevent excessive global matching errors caused by local matching
                if (best_vote_list.size() > best_vote_num)
                {
                    // Collect the corresponding relationships and corner indices of refstd features
                    vector<int> match_pt_idx;
                    vector<std::pair<PointItdesc, PointItdesc>> match_itdpairs;
                    match_itdpairs.reserve(best_vote_list.size());
                    match_pt_idx.reserve(best_vote_list.size() * 3);
                    for (auto &bvl : best_vote_list)
                    {
                        match_pt_idx.emplace_back(3 * bvl);
                        match_pt_idx.emplace_back(3 * bvl + 1);
                        match_pt_idx.emplace_back(3 * bvl + 2);
                        match_itdpairs.emplace_back(match_candidates[max_vote_index][bvl]);
                    }

                    Eigen::Vector3d best_t;
                    Eigen::Matrix3d best_rot;
                    solveSVD(points_ori(Eigen::all, match_pt_idx), points_tar(Eigen::all, match_pt_idx), best_t, best_rot);

                    // Evaluate the score after matching
                    float match_error = ((points_tar(Eigen::all, match_pt_idx) - ((best_rot * points_ori(Eigen::all, match_pt_idx)).colwise() + best_t)).colwise().norm()).norm() / best_vote_list.size();

                    // Calculate the relative position of the solution. If it is greater than a certain distance, it is considered an invalid loop because pose estimation may not be accurate
                    double relative_t = best_t.norm();
                    if (relative_t <= relative_dis_threshold) // Outdoor is 10, indoor is 5
                    {
                        best_prev_index = prev_kf->index;
                        best_itdpairs.swap(match_itdpairs);
                        best_match_error = match_error;
                        best_vote_num = best_vote_list.size();
                        best_transform = Eigen::Affine3d::Identity();
                        best_transform.translate(best_t);
                        best_transform.rotate(best_rot);
                    }
                }
            }
        }
        else
            break;
    }

    auto t3 = std::chrono::high_resolution_clock::now();
    std::cout << "[Time] candidate selector: " << time_inc(t2, t1)
              << " ms, candidate verify: " << time_inc(t3, t2) << "ms"
              << std::endl;

    candidate_time = time_inc(t2, t1);
    vertify_time = time_inc(t3, t2);

    loop_result.first = best_prev_index;
    loop_result.second = static_cast<double>(best_vote_num);

    match_candidates.clear();
    static int total_loop_num = 0;
    if (best_vote_num > min_matched_num && best_match_error < 0.5f)
    {
        total_loop_num++;
        loop_transform = best_transform;
        loop_itd_pair.swap(best_itdpairs);

        cout << "Find loop-> [" << setw(4) << setfill(' ') << itd_frame->index << "---"
             << setw(4) << setfill(' ') << best_prev_index << "] ****** "
             << " Match error: " << setprecision(2) << best_match_error << " ****** "
             << " Total loop: " << setw(4) << setfill(' ') << total_loop_num << endl;
        return true;
    }
    else
        return false;
}

bool IntensityTD::doICPRelative(const PointCloudXYZI::Ptr pcl_src, const PointCloudXYZI::Ptr pcl_tar, Eigen::Affine3d &pose_relative)
{
    PointCloudXYZI::Ptr pcl_src_dw(new PointCloudXYZI());
    PointCloudXYZI::Ptr pcl_tar_dw(new PointCloudXYZI());
    pcl::VoxelGrid<PointXYZI> downSizeFilterICP;
    downSizeFilterICP.setLeafSize(0.5, 0.5, 0.5);
    downSizeFilterICP.setInputCloud(pcl_src);
    downSizeFilterICP.filter(*pcl_src_dw);
    downSizeFilterICP.setInputCloud(pcl_tar);
    downSizeFilterICP.filter(*pcl_tar_dw);

    pcl::GeneralizedIterativeClosestPoint<PointXYZI, PointXYZI> gicp;
    gicp.setInputSource(pcl_src_dw);
    gicp.setInputTarget(pcl_tar_dw);
    gicp.setMaximumIterations(100);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setEuclideanFitnessEpsilon(1e-6);
    // gicp.setRANSACIterations(0);
    // // gicp.setMaxCorrespondenaceDistance(150);

    PointCloudXYZI Final;
    gicp.align(Final, pose_relative.matrix().cast<float>());

    pose_relative.matrix() = gicp.getFinalTransformation().cast<double>();
    cout << gicp.getFitnessScore() << " | " << gicp.hasConverged() << endl;
    cout << pose_relative.matrix() << endl;
    // if (gicp.hasConverged() == false || gicp.getFitnessScore() > 0.5)
    if (gicp.hasConverged() == false)
        return false;
    else
        return true;
}