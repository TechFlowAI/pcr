#ifndef POINT_CLOUD_REGISTRATION_H_
#define POINT_CLOUD_REGISTRATION_H_

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace pcr {
template <typename T>
void Eigen2Pcl(const Eigen::Matrix<double, 3, Eigen::Dynamic> &src, pcl::PointCloud<T> &cloud) {
    int num_pc = src.cols();
    T pt_tmp;
    if (!cloud.empty())
        cloud.clear();
    for (int i = 0; i < num_pc; ++i) {
        pt_tmp.x = src(0, i);
        pt_tmp.y = src(1, i);
        pt_tmp.z = src(2, i);
        cloud.points.emplace_back(pt_tmp);
    }
}

template <typename T>
void Voxelize(typename pcl::PointCloud<T>::Ptr input_cloud, typename pcl::PointCloud<T>::Ptr output_cloud,
              double voxel_size) {
    pcl::VoxelGrid<T> voxel_filter;
    voxel_filter.setInputCloud(input_cloud);
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.filter(*output_cloud);
}
}  // namespace pcr

#endif