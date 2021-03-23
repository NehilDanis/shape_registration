#ifndef PREPROCESSING_HPP
#define PREPROCESSING_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

namespace Preprocessing {

/**
 * @brief pass_through_filter
 * @param input_cloud
 * @param x_min_val
 * @param x_max_val
 * @param y_min_val
 * @param y_max_val
 * @param z_min_val
 * @param z_max_val
 * @return filtered cloud
 */
PointCloudT::Ptr pass_through_filter(PointCloudT::Ptr &input_cloud, float x_min_val, float x_max_val, float y_min_val, float y_max_val, float z_min_val, float z_max_val);

/**
 * @brief voxel_grid_downsampling
 * @param input_cloud
 * @param voxel_size
 * @return
 */
PointCloudT::Ptr voxel_grid_downsampling(PointCloudT::Ptr &input_cloud, float voxel_size);

/**
 * @brief statistical_filtering
 * @param input_cloud
 * @return
 */
PointCloudT::Ptr statistical_filtering(PointCloudT::Ptr &input_cloud);

/**
 * @brief extract_plane
 * @param input_cloud
 * @param threshold
 * @return
 */
PointCloudT::Ptr extract_plane(PointCloudT::Ptr &input_cloud, double threshold);


}


#endif // PREPROCESSING_HPP
