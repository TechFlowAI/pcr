#include "fpfh.h"

namespace pcr {
FPFHEstimation::FPFHEstimation() : fpfh_estimation_(new pcl::FPFHEstimationOMP<PointType, pcl::Normal, FPFHType>()) {}

FPFHCloudPtr FPFHEstimation::ComputeFPFHFeatures(const CloudPtr &input_cloud, double normal_search_radius,
                                                 double fpfh_search_radius) {
    // define some intermediate variables
    NormalCloudPtr estimated_normals(new NormalCloudType());
    FPFHCloudPtr descriptors(new FPFHCloudType());

    // estimate normals using pcl function
    pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(input_cloud);
    normal_estimator.setRadiusSearch(normal_search_radius);
    pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
    normal_estimator.setSearchMethod(kdtree);
    normal_estimator.compute(*estimated_normals);

    // estimate FPFH using pcl function
    SetInputCloud(input_cloud);
    SetInputNormals(estimated_normals);
    SetSearchMethod(kdtree);
    SetRadiusSearch(fpfh_search_radius);
    Compute(*descriptors);

    return descriptors;
}

FPFHCloudPtr FPFHEstimation::ComputeFPFHFeatures(const CloudPtr &input_cloud, NormalCloudType &normals,
                                                 double normal_search_radius, double fpfh_search_radius) {
    // define some intermediate variables
    NormalCloudPtr estimated_normals(new NormalCloudType());
    FPFHCloudPtr descriptors(new FPFHCloudType());

    // estimate normals using pcl function
    pcl::NormalEstimationOMP<PointType, pcl::Normal> normal_estimator;
    normal_estimator.setInputCloud(input_cloud);
    normal_estimator.setRadiusSearch(normal_search_radius);
    pcl::search::KdTree<PointType>::Ptr kdtree(new pcl::search::KdTree<PointType>);
    normal_estimator.setSearchMethod(kdtree);
    normal_estimator.compute(*estimated_normals);

    normals = *estimated_normals;

    // estimate FPFH using pcl function
    SetInputCloud(input_cloud);
    SetInputNormals(estimated_normals);
    SetSearchMethod(kdtree);
    SetRadiusSearch(fpfh_search_radius);
    Compute(*descriptors);

    return descriptors;
}

void FPFHEstimation::Compute(FPFHCloudType &output_cloud) { fpfh_estimation_->compute(output_cloud); }

void FPFHEstimation::SetInputCloud(CloudPtr input_cloud) { fpfh_estimation_->setInputCloud(input_cloud); }

void FPFHEstimation::SetInputNormals(NormalCloudPtr input_normals) { fpfh_estimation_->setInputNormals(input_normals); }

void FPFHEstimation::SetSearchMethod(pcl::search::KdTree<PointType>::Ptr search_method) {
    fpfh_estimation_->setSearchMethod(search_method);
}

void FPFHEstimation::SetRadiusSearch(double search_radius) { fpfh_estimation_->setRadiusSearch(search_radius); }

}  // namespace pcr