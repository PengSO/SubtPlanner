#pragma once

#include "global_repositioning/nelder_mead_rot.h"

#include "voxel_map/voxel_map.h"
#include "planner_common/params.h"

#include "pcl_ros/point_cloud.h"
#include <pcl/search/impl/search.hpp>

#include <boost/function.hpp>
#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

#include "sensor_msgs/PointCloud2.h"

enum RepositioningRegionType { kPca = 0, kMvbb, kAabb };

struct OrientationSearchParams {
  int max_iterations = 100;
  int max_iter_nm = 20;
  int population_size = 10;
  double tol = 1e-2;
  int tol_iter = 5;
};

class Individual {
  public:
  Individual(){};
  Individual(bool random);
  Eigen::Matrix3d& operator[](int i) { return chromosome_[i]; };
  double calculateFitness(const boost::function<double(const Eigen::Matrix3d)>& objFun);

  void mutation(const boost::function<double(const Eigen::Matrix3d)>& objFun,
                NelderMeadRot<Eigen::Matrix3d>& nealder_mead);

  double getFitness() const { return fitness_; };

  int getFitnessIndex() const { return fitness_idx_; };
  Eigen::Matrix3d& getFittestGene() { return chromosome_[fitness_idx_]; };

  private:
  double fitness_;
  int fitness_idx_;
  std::vector<Eigen::Matrix3d> chromosome_;
};

class OrientationSearch {
  public:
  OrientationSearch(){};
  OrientationSearch(OrientationSearchParams params) : params_(params), nealder_mead_(params.max_iter_nm){};

  void setParams(OrientationSearchParams params) { params_ = params; };

  void initPopulation(std::vector<Individual>& population);
  double calculateFitness(std::vector<Individual>& population);
  void sortIndividuals(std::vector<Individual>& population);
  void crossover(std::vector<Individual>& population);
  void mutation(std::vector<Individual>& population);

  Eigen::Matrix3d runOrientationSearch(const boost::function<double(const Eigen::Matrix3d)> objFun);

  private:
  boost::function<double(const Eigen::Matrix3d)> objFun_;
  std::vector<Individual> population_;
  OrientationSearchParams params_;
  NelderMeadRot<Eigen::Matrix3d> nealder_mead_;
};

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
