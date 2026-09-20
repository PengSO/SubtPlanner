#include "voxel_map/voxel_map.h"

VoxelMap::VoxelMap(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
  : nh_(nh), nh_private_(nh_private) 
{
  voxblox_backend_ = std::make_shared<VoxbloxMapBackend<VoxbloxMapServer, VoxbloxMapVoxel>>(nh, nh_private);
}

double VoxelMap::getResolution()
{
  return voxblox_backend_->getResolution();
}

bool VoxelMap::getStatus()
{
  return voxblox_backend_->getStatus();
}

VoxelStatus VoxelMap::getVoxelStatus(const Eigen::Vector3d& position)
{
  return voxblox_backend_->getVoxelStatus(position);
}

float VoxelMap::getVoxelDistance(const Eigen::Vector3d& center)
{
  return voxblox_backend_->getVoxelDistance(center);
}

double VoxelMap::getPointDistance(const Eigen::Vector3d& point)
{
  return voxblox_backend_->getPointDistance(point);
}

Eigen::Vector3d VoxelMap::getPointGradient(const Eigen::Vector3d& point)
{
  return voxblox_backend_->getPointGradient(point);
}

VoxelStatus VoxelMap::getRayStatus(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test,
                                  bool stop_at_unknown_voxel)
{
  return voxblox_backend_->getRayStatus(view_point,
                                  voxel_to_test,
                                  stop_at_unknown_voxel);
}

VoxelStatus VoxelMap::getRayStatus(const Eigen::Vector3d& view_point,
                                  const Eigen::Vector3d& voxel_to_test,
                                  bool stop_at_unknown_voxel,
                                  Eigen::Vector3d& end_voxel,
                                  double& tsdf_dist)
{
  return voxblox_backend_->getRayStatus(view_point,
                                  voxel_to_test,
                                  stop_at_unknown_voxel,
                                  end_voxel,
                                  tsdf_dist);
}

VoxelStatus VoxelMap::getBoxStatus(const Eigen::Vector3d& center,
                                  const Eigen::Vector3d& size,
                                  bool stop_at_unknown_voxel)
{
  return voxblox_backend_->getBoxStatus(center, size, stop_at_unknown_voxel);
}

VoxelStatus VoxelMap::getPathStatus(const Eigen::Vector3d& start,
                                  const Eigen::Vector3d& end,
                                  const Eigen::Vector3d& box_size,
                                  bool stop_at_unknown_voxel)
{
  return voxblox_backend_->getPathStatus(start, end, box_size, stop_at_unknown_voxel);
}

bool VoxelMap::augmentFreeBox(const Eigen::Vector3d& position,
                            const Eigen::Vector3d& box_size)
{
  return voxblox_backend_->augmentFreeBox(position, box_size);
}

void VoxelMap::augmentFreeFrustum()
{
  voxblox_backend_->augmentFreeFrustum();
}

void VoxelMap::getFreeSpacePointCloud(std::vector<Eigen::Vector3d> multiray_endpoints, StateVec state,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  voxblox_backend_->getFreeSpacePointCloud(multiray_endpoints, state,
    cloud);
}

void VoxelMap::getScanStatus(
    Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
    std::tuple<int, int, int>& gain_log,
    std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
    SensorParamsBase& sensor_params)
{
  voxblox_backend_->getScanStatus( pos, multiray_endpoints,
    gain_log, voxel_log, sensor_params);
}

void VoxelMap::extractLocalMap(const Eigen::Vector3d& center,
                              const Eigen::Vector3d& bounding_box_size,
                              std::vector<Eigen::Vector3d>& occupied_voxels,
                              std::vector<Eigen::Vector3d>& free_voxels)
{
  voxblox_backend_->extractLocalMap(center,
                              bounding_box_size, occupied_voxels, free_voxels);
}

void VoxelMap::extractLocalMapAlongAxis(
    const Eigen::Vector3d& center, const Eigen::Vector3d& axis,
    const Eigen::Vector3d& bounding_box_size,
    std::vector<Eigen::Vector3d>& occupied_voxels,
    std::vector<Eigen::Vector3d>& free_voxels)
{
  voxblox_backend_->extractLocalMapAlongAxis(
    center, axis,
    bounding_box_size,
    occupied_voxels,
    free_voxels);
}

void VoxelMap::getLocalPointcloud(const Eigen::Vector3d& center, const double& range,
                        const double& yaw,
                        pcl::PointCloud<pcl::PointXYZI>& pcl,
                        bool include_unknown_voxels)
{
  voxblox_backend_->getLocalPointcloud(center, range,
                        yaw, pcl, include_unknown_voxels);
}

void VoxelMap::getLocalPointcloud(const Eigen::Vector3d& center, const double& range,
                        const double& yaw, const Eigen::Vector2d& z_limits,
                        pcl::PointCloud<pcl::PointXYZI>& pcl,
                        bool include_unknown_voxels)
{
  voxblox_backend_->getLocalPointcloud(center, range,
                        yaw, z_limits, pcl,
                        include_unknown_voxels);
}

void VoxelMap::annotateCameraVoxels(Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints)
{
  voxblox_backend_->annotateCameraVoxels(pos, multiray_endpoints);
}

void VoxelMap::getCameraScanStatus(Eigen::Vector3d& pos, std::vector<Eigen::Vector3d>& multiray_endpoints,
  std::tuple<int, int, int>& gain_log,
  std::vector<std::pair<Eigen::Vector3d, VoxelStatus>>& voxel_log,
  SensorParamsBase& sensor_params)
{
  voxblox_backend_->getCameraScanStatus(pos, multiray_endpoints,
          gain_log, voxel_log, sensor_params);
}

void VoxelMap::getCameraScanStatus(StateVec& state, SensorParamsBase& sensor_params, std::vector<VoxelLog> &out_logs)
{
  voxblox_backend_->getCameraScanStatus(state, sensor_params, out_logs);
}

void VoxelMap::resetMap()
{
  voxblox_backend_->resetMap();
}

void VoxelMap::setRaycastingParams(bool nonuniform_ray_cast,
                          double ray_cast_step_size_multiplier)
{
  voxblox_backend_->setRaycastingParams(nonuniform_ray_cast,
                          ray_cast_step_size_multiplier);
}

void VoxelMap::setRobotRadius(double robot_radius)
{
  voxblox_backend_->setRobotRadius(robot_radius);
}

void VoxelMap::setBoxCheckMethod(int m)
{
  voxblox_backend_->setBoxCheckMethod(m);
}

void VoxelMap::setLineCheckMethod(int m)
{
  voxblox_backend_->setLineCheckMethod(m);
}
