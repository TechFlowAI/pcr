#ifndef POINT_CLOUD_REGISTRATION_POINT_TYPES_H_
#define POINT_CLOUD_REGISTRATION_POINT_TYPES_H_

#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using PointType = pcl::PointXYZ;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

using NormalPointType = pcl::Normal;
using NormalCloudType = pcl::PointCloud<NormalPointType>;
using NormalCloudPtr = NormalCloudType::Ptr;

using FPFHType = pcl::FPFHSignature33;
using FPFHCloudType = pcl::PointCloud<FPFHType>;
using FPFHCloudPtr = FPFHCloudType::Ptr;

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using VecXd = Eigen::Matrix<double, Eigen::Dynamic, 1>;

using Vec3f = Eigen::Vector3f;
using VecXf = Eigen::Matrix<float, Eigen::Dynamic, 1>;

using Mat2d = Eigen::Matrix2d;
using Mat3d = Eigen::Matrix3d;
using Mat4d = Eigen::Matrix4d;
using Mat3Xd = Eigen::Matrix<double, 3, Eigen::Dynamic>;
using MatXd = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

using Mat2Xi = Eigen::Matrix<int, 2, Eigen::Dynamic>;

#endif