#ifndef POINT_CLOUD_REGISTRATION_FPFH_H_
#define POINT_CLOUD_REGISTRATION_FPFH_H_

#include "common/point_types.h"

namespace pcr {
class FPFHEstimation {
   public:
    FPFHEstimation();

    FPFHCloudPtr ComputeFPFHFeatures(const CloudPtr &input_cloud, double normal_search_radius = 0.03,
                                     double fpfh_search_radius = 0.05);

    FPFHCloudPtr ComputeFPFHFeatures(const CloudPtr &input_cloud, NormalCloudType &normals,
                                     double normal_search_radius = 0.03, double fpfh_search_radius = 0.05);

    inline pcl::FPFHEstimationOMP<PointType, NormalPointType, FPFHType>::Ptr GetImplPointer() const {
        return fpfh_estimation_;
    }
    void Compute(FPFHCloudType &output_cloud);

    void SetInputCloud(CloudPtr input_cloud);

    void SetInputNormals(NormalCloudPtr input_normals);

    void SetSearchMethod(pcl::search::KdTree<PointType>::Ptr search_method);

    void SetRadiusSearch(double search_radius);

   private:
    pcl::FPFHEstimationOMP<PointType, NormalPointType, FPFHType>::Ptr fpfh_estimation_;
};
}  // namespace pcr

#endif