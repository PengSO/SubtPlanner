#pragma once

#include "global_repositioning/orientation_search.h"

#include "voxel_map/voxel_map.h"
#include "cognitive_graph_model/params.h"

#include "pcl_ros/point_cloud.h"
#include <pcl/search/impl/search.hpp>

#include <eigen3/Eigen/Dense>
#include <string>

#include "sensor_msgs/PointCloud2.h"

enum RepositioningRegionType { kPca = 0, kMvbb, kAabb };

class RepositioningRegion {
  public:
  RepositioningRegion(VoxelMap* voxel_map)
      : voxel_map_(voxel_map){
        // current_local_pcl_ = new pcl::PointCloud<pcl::PointXYZI>();
      }
  ~RepositioningRegion();

  void computeBounds(Eigen::Vector3d& min_val, Eigen::Vector3d& max_val, const Eigen::Vector3d& offset,
                     const Eigen::Matrix3d& rot_w2b, const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);
  double computeVolume(const Eigen::Matrix3d& rot_w2b, const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud);
  bool rotationsSafetyCheck(Eigen::Matrix3d& rot_w2b, Eigen::Vector3d& eig_val);

  void computeVarianceInReferenceFrame(Eigen::Vector3d& variance, const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud, const Eigen::Matrix3d& rot_w2b);

  void constructBoundingBox(const Eigen::Vector3d& pos,
                                  Eigen::Vector3d& min_val,
                                  Eigen::Vector3d& max_val,
                                  Eigen::Vector3d& rotations,
                                  Eigen::Vector3d& mean_val,
                                  Eigen::Vector3d& std_val);

  bool loadParams(std::string ns);
  pcl::PointCloud<pcl::PointXYZI>::Ptr getLocalPCL() { return current_local_pcl_; }

  private:
  VoxelMap* voxel_map_;
  OrientationSearch orientation_search_;

  RepositioningRegionType type_;
  double local_pointcloud_range_;
  double bounding_box_size_max_;
  double distribution_scaling_max_;
  double voxel_filter_leaf_size_;

  // Updated after constructBoundingBox runs
  pcl::PointCloud<pcl::PointXYZI>::Ptr current_local_pcl_;
};
