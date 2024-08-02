#ifndef MYPOINT_TYPE_H
#define MYPOINT_TYPE_H
#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

// Redefining the point cloud format of ouster, excerpted from the official library of ouster_ros
namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

// Directly obtain corner points from the image, save the position of the corner points in the image to index the patch
namespace corner
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        uint16_t v;     // Image v coordinate
        uint16_t u;     // Image u coordinate
        uint8_t imgidx; // Image index
        uint32_t t;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(corner::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint16_t, v, v)
    (std::uint16_t, u, u)
    (std::uint8_t, imgidx, imgidx)
    (std::uint32_t, t, t)
)

//Customize point cloud data containing reflectance gmm information
namespace gmm
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float refquality;
        union EIGEN_ALIGN16 {
            float refmean[3];
            struct{
                float refmean1;
                float refmean2;
                float refmean3;
            };
        };
        union EIGEN_ALIGN16 {
            float refvar[3];
            struct{
                float refvar1;
                float refvar2;
                float refvar3;
            };
        };
        union EIGEN_ALIGN16 {
            float refweight[3];
            struct{
                float refweight1;
                float refweight2;
                float refweight3;
            };
        };
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(gmm::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, refquality, refquality)
    (float, refmean1, refmean1)
    (float, refmean2, refmean2)
    (float, refmean3, refmean3)
    (float, refvar1, refvar1)
    (float, refvar2, refvar2)
    (float, refvar3, refvar3)
    (float, refweight1, refweight1)
    (float, refweight2, refweight2)
    (float, refweight3, refweight3)
)

//Custom descriptor
namespace itdesc
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        uint16_t kfid;
        union EIGEN_ALIGN16 {
            uint16_t v_idx[3];
            struct{
                uint16_t va_idx;
                uint16_t vb_idx;
                uint16_t vc_idx;
            };
        };
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(itdesc::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint16_t, kfid, kfid)
    (std::uint16_t, va_idx, va_idx)
    (std::uint16_t, vb_idx, vb_idx)
    (std::uint16_t, vc_idx, vc_idx)
)

typedef pcl::PointXYZINormal PointXYZI;
typedef pcl::PointCloud<PointXYZI> PointCloudXYZI;

typedef ouster_ros::Point PointOuster;                 
typedef pcl::PointCloud<PointOuster> PointCloudOuster; 

typedef corner::Point PointCorner;                     
typedef pcl::PointCloud<PointCorner> PointCloudCorner;
typedef std::vector<PointCorner, Eigen::aligned_allocator<PointCorner>> PointVectorCorner;

typedef gmm::Point PointGmm;                     
typedef pcl::PointCloud<PointGmm> PointCloudGmm; 

typedef itdesc::Point PointItdesc;                     
typedef pcl::PointCloud<PointItdesc> PointCloudItdesc; 
typedef std::vector<PointItdesc, Eigen::aligned_allocator<PointItdesc>> PointVectorItdesc;

#endif