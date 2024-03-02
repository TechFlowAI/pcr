#ifndef POINT_CLOUD_REGISTRATION_FPFH_MANAGER_H_
#define POINT_CLOUD_REGISTRATION_FPFH_MANAGER_H_

#include "common/point_types.h"
#include "common/tic_toc.h"

#include "feature_matcher.h"

#include "fpfh.h"

#include <iostream>
#include <stdexcept>

namespace pcr {
class FPFHManager {
   public:
    FPFHManager(double nromal_radius, double fpfh_radius);

    FPFHManager();

    ~FPFHManager();

    void SetFeaturePair(CloudPtr source_cloud, CloudPtr target_cloud);

    void ClearInputs();

    Eigen::Matrix<double, 3, Eigen::Dynamic> GetSourceMatched();

    Eigen::Matrix<double, 3, Eigen::Dynamic> GetTargetMatched();

    Eigen::Matrix<double, 3, Eigen::Dynamic> GetTargetNormals();

    FPFHCloudType GetSourceDescriptor();

    FPFHCloudType GetTargetDescriptor();

    PointCloudType GetSourceKeyPointCloud();

    PointCloudType GetTargetKeyPointCloud();

   private:
    FPFHCloudPtr source_descriptors_;
    FPFHCloudPtr target_descriptors_;

    double normal_radius_;
    double fpfh_radius_;

    std::vector<std::pair<int, int>> corr_;  // Correspondence

    Eigen::Matrix<double, 3, Eigen::Dynamic> source_matched_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> target_matched_;
    Eigen::Matrix<double, 3, Eigen::Dynamic> target_normals_;

    PointCloudType source_matched_pcl_;
    PointCloudType target_matched_pcl_;
};
}  // namespace pcr
#endif