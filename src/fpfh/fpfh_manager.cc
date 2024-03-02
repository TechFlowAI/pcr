#include "fpfh_manager.h"

#include "common/utility.h"

namespace pcr {
FPFHManager::FPFHManager(double nromal_radius, double fpfh_radius)
    : normal_radius_(nromal_radius), fpfh_radius_(fpfh_radius) {
    source_descriptors_.reset(new FPFHCloudType());
    target_descriptors_.reset(new FPFHCloudType());
}

FPFHManager::FPFHManager() {
    source_descriptors_.reset(new FPFHCloudType());
    target_descriptors_.reset(new FPFHCloudType());
}

FPFHManager::~FPFHManager() {
    normal_radius_ = 0.5;
    fpfh_radius_ = 0.6;
    source_descriptors_.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
    target_descriptors_.reset(new pcl::PointCloud<pcl::FPFHSignature33>());
}

void FPFHManager::SetFeaturePair(CloudPtr source_cloud, CloudPtr target_cloud) {
    if (normal_radius_ > fpfh_radius_) {
        std::cout << "normal_radius = " << normal_radius_ << "; "
                  << "fpfh_radius = " << fpfh_radius_ << std::endl;
        throw std::invalid_argument("[FPFHManager]: Normal radius should be lower than fpfh radius");
    }

    pcl::PointCloud<pcl::Normal>::Ptr source_normals_raw(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr target_normals_raw(new pcl::PointCloud<pcl::Normal>());

    TicToc computer_fpfh_time;
    FPFHEstimation fpfh_estimator;
    source_descriptors_ =
        fpfh_estimator.ComputeFPFHFeatures(source_cloud, *source_normals_raw, normal_radius_, fpfh_radius_);
    target_descriptors_ =
        fpfh_estimator.ComputeFPFHFeatures(target_cloud, *target_normals_raw, normal_radius_, fpfh_radius_);
    std::cout << "compute fpfh descriptors costs: " << computer_fpfh_time.Toc() << " ms" << std::endl;;

    TicToc compute_fpfh_corr_time;
    Matcher matcher;
    corr_ = matcher.CalculateCorrespondences(*source_cloud, *target_cloud, *source_descriptors_, *target_descriptors_,
                                             true, true, true, 0.95);
    std::cout << "compute fpfh correspondences cost: " << compute_fpfh_corr_time.Toc() << " ms" << std::endl;

    source_matched_.resize(3, corr_.size());
    target_matched_.resize(3, corr_.size());
    target_normals_.resize(3, corr_.size());

    for (int i = 0; i < corr_.size(); ++i) {
        auto src_idx = corr_[i].first;
        auto tgt_idx = corr_[i].second;
        source_matched_.col(i) << source_cloud->points[src_idx].x, source_cloud->points[src_idx].y,
            source_cloud->points[src_idx].z;
        target_matched_.col(i) << target_cloud->points[tgt_idx].x, target_cloud->points[tgt_idx].y,
            target_cloud->points[tgt_idx].z;

        target_normals_.col(i) << target_normals_raw->points[tgt_idx].normal_x,
            target_normals_raw->points[tgt_idx].normal_y, target_normals_raw->points[tgt_idx].normal_z;
    }

    Eigen2Pcl(source_matched_, source_matched_pcl_);
    Eigen2Pcl(target_matched_, target_matched_pcl_);
}

void FPFHManager::ClearInputs() {
    corr_.clear();
    source_descriptors_->clear();
    target_descriptors_->clear();
}

Eigen::Matrix<double, 3, Eigen::Dynamic> FPFHManager::GetSourceMatched() { return source_matched_; }

Eigen::Matrix<double, 3, Eigen::Dynamic> FPFHManager::GetTargetMatched() { return target_matched_; }

Eigen::Matrix<double, 3, Eigen::Dynamic> FPFHManager::GetTargetNormals() { return target_normals_; }

FPFHCloudType FPFHManager::GetSourceDescriptor() { return *source_descriptors_; }

FPFHCloudType FPFHManager::GetTargetDescriptor() { return *target_descriptors_; }

PointCloudType FPFHManager::GetSourceKeyPointCloud() { return source_matched_pcl_; }

PointCloudType FPFHManager::GetTargetKeyPointCloud() { return target_matched_pcl_; }

}  // namespace pcr