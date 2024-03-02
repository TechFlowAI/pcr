#include <iostream>

#include <pcl/io/pcd_io.h>

#include "common/point_types.h"
#include "common/utility.h"
#include "fpfh/fpfh_manager.h"

using namespace pcr;
int main(int argc, char **argv) {
    CloudPtr source_cloud(new PointCloudType());
    CloudPtr target_cloud(new PointCloudType());
    if (pcl::io::loadPCDFile("../data/source.pcd", *source_cloud) == -1) {
        PCL_ERROR("couldn't load source point cloud \n");
        return -1;
    }
    if (pcl::io::loadPCDFile("../data/target.pcd", *target_cloud) == -1) {
        PCL_ERROR("couldn't load target point cloud \n");
        return -1;
    }

    FPFHManager fpfhmanager(0.5, 0.75);
    // voxelize
    CloudPtr source_feature(new PointCloudType());
    CloudPtr target_feature(new PointCloudType());
    double voxel_size = 1.0;
    Voxelize<PointType>(source_cloud, source_feature, voxel_size);
    Voxelize<PointType>(target_cloud, target_feature, voxel_size);

    fpfhmanager.SetFeaturePair(source_feature, target_feature);

    // Eigen type
    //    Eigen::Matrix<double, 3, Eigen::Dynamic> src_matched = fpfhmanager.getSrcMatched();
    //    Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_matched = fpfhmanager.getTgtMatched();

    // PCL type
    CloudPtr source_matched(new PointCloudType());
    CloudPtr target_matched(new PointCloudType());
    *source_matched = fpfhmanager.GetSourceKeyPointCloud();
    *target_matched = fpfhmanager.GetTargetKeyPointCloud();

    std::cout << "source_matched size = " << source_matched->points.size() << std::endl;
    std::cout << "target_matched size = " << target_matched->points.size() << std::endl;

    return 0;
}