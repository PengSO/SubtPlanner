#include "global_repositioning/repositioning_region.h"

#include <limits>

#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>

#include "planner_common/terminal_log.h"

Individual::Individual(bool random)
{
  for (int i = 0; i < 4; i++) {
    if (random) {
      Eigen::Matrix3d gene = Eigen::Matrix3d::Random(3, 3);
      gene = gene.householderQr().householderQ();
      chromosome_.push_back(gene);
    }
  }
}

double Individual::calculateFitness(const boost::function<double(const Eigen::Matrix3d)>& objFun)
{
  fitness_ = std::numeric_limits<double>::max();
  fitness_idx_ = 0;

  int idx = 0;
  for (auto& gene : chromosome_) {
    double fitness_gene = objFun(gene);
    if (fitness_gene < fitness_) {
      fitness_ = fitness_gene;
      fitness_idx_ = idx;
    }

    idx = idx + 1;
  }
  return fitness_;
}

void Individual::mutation(const boost::function<double(const Eigen::Matrix3d)>& objFun,
                          NelderMeadRot<Eigen::Matrix3d>& nealder_mead)
{
  nealder_mead.solve(objFun, chromosome_);
}

void OrientationSearch::initPopulation(std::vector<Individual>& population)
{
  population.clear();
  const bool random = true;

  for (int i = 0; i < params_.population_size; i++) {
    population.push_back(Individual(random));
  }
}

double OrientationSearch::calculateFitness(std::vector<Individual>& population)
{
  double best_fitness = std::numeric_limits<double>::max();
  for (auto& individual : population) {
    double fitness = individual.calculateFitness(objFun_);
    if (fitness < best_fitness)
      best_fitness = fitness;
  }
  return best_fitness;
}

void OrientationSearch::sortIndividuals(std::vector<Individual>& population)
{
  stable_sort(population.begin(), population.end(),
              [](const Individual& i1, const Individual& i2)
              {
                return i1.getFitness() < i2.getFitness();
              });
}

void OrientationSearch::crossover(std::vector<Individual>& population)
{
  int population_size_half_c = ceil(params_.population_size / 2);
  int population_size_half_f = floor(params_.population_size / 2);

  std::vector<Individual> pop1;
  std::vector<Individual> pop2;
  std::vector<Individual> pop3;
  std::vector<Individual> pop4;

  for (int i = 0; i < population_size_half_f; i++) {
    int idx1 = rand() % population_size_half_c;
    int idx2 = rand() % population_size_half_c;
    pop1.push_back(population[idx1]);
    pop2.push_back(population[idx2]);
  }

  for (int i = 0; i < population_size_half_c; i++) {
    int idx3 = rand() % population_size_half_f;
    int idx4 = rand() % population_size_half_f;
    pop3.push_back(population[idx3]);
    pop4.push_back(population[idx4]);
  }

  for (int i = 0; i < population_size_half_f; i++) {
    double cutoff = 0.5 + 0.1 * (pop1[i].getFitness() <= pop2[i].getFitness()) -
                    0.1 * (pop1[i].getFitness() >= pop2[i].getFitness());
    for (int j = 0; j < 4; j++) {
      if ((double)rand() / RAND_MAX < cutoff)
        population[i][j] = pop1[i][j];
      else
        population[i][j] = pop2[i][j];
    }
  }

  for (int i = 0; i < population_size_half_c; i++) {
    int i2 = population_size_half_f + i;
    double cutoff = 0.5 + 0.1 * (pop3[i].getFitness() <= pop4[i].getFitness()) -
                    0.1 * (pop3[i].getFitness() >= pop4[i].getFitness());
    for (int j = 0; j < 4; j++) {
      Eigen::Matrix3d mat = cutoff * pop3[i][j] + (1 - cutoff) * pop4[i][j];
      mat = mat.householderQr().householderQ();
      population[i2][j] = mat;
    }
  }
}

void OrientationSearch::mutation(std::vector<Individual>& population)
{
  for (int i = 0; i < params_.population_size; i++) {
    population[i].mutation(objFun_, nealder_mead_);
  }
}

Eigen::Matrix3d OrientationSearch::runOrientationSearch(
    const boost::function<double(const Eigen::Matrix3d)> objFun)
{
  objFun_ = objFun;

  initPopulation(population_);

  double best_fitness = std::numeric_limits<double>::max();
  double old_fitness;
  int tol_counter = 0;

  for (int iter = 0; iter < params_.max_iterations; iter++) {
    old_fitness = best_fitness;
    best_fitness = calculateFitness(population_);

    sortIndividuals(population_);

    if (abs(best_fitness - old_fitness) < params_.tol) {
      tol_counter = tol_counter + 1;
      if (tol_counter >= params_.tol_iter) {
        break;
      }
    }
    else {
      tol_counter = 0;
    }

    crossover(population_);
    mutation(population_);
  }

  return population_[0].getFittestGene();
}

void RepositioningRegion::computeBounds(Eigen::Vector3d& min_val, Eigen::Vector3d& max_val,
                                const Eigen::Vector3d& offset, const Eigen::Matrix3d& rot_w2b,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  double inf = std::numeric_limits<double>::max();

  min_val = Eigen::Vector3d::Ones() * inf;
  max_val = -Eigen::Vector3d::Ones() * inf;

  for (auto& p : pointcloud->points) {
    Eigen::Vector3d point_w(p.x, p.y, p.z);
    Eigen::Vector3d point_b = rot_w2b * (point_w - offset);

    for (int i = 0; i < 3; i++) {
      if (point_b[i] < min_val[i]) min_val[i] = point_b[i];
      if (point_b[i] > max_val[i]) max_val[i] = point_b[i];
    }
  }
}

double RepositioningRegion::computeVolume(const Eigen::Matrix3d& rot_w2b,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud)
{
  Eigen::Vector3d min_val;
  Eigen::Vector3d max_val;

  computeBounds(min_val, max_val, Eigen::Vector3d::Zero().eval(), rot_w2b, pointcloud);

  return (max_val[0] - min_val[0]) * (max_val[1] - min_val[1]) * (max_val[2] - min_val[2]);
}

bool RepositioningRegion::rotationsSafetyCheck(Eigen::Matrix3d& rot_w2b, Eigen::Vector3d& eig_val)
{
  if (abs(rot_w2b(2, 0)) > 0.8) {
    // new x ← old y
    // new y ← old z
    // new z ← old x
    Eigen::Matrix3d rot_swap;
    rot_swap << 0, 1, 0, 
                0, 0, 1, 
                1, 0, 0;

    rot_w2b = rot_swap * rot_w2b;
    eig_val = rot_swap * eig_val;
    return true;
  }
  return false;
}

void RepositioningRegion::computeVarianceInReferenceFrame(Eigen::Vector3d& variance, 
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud, 
                                const Eigen::Matrix3d& rot_w2b) 
{
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.topLeftCorner(3, 3) = rot_w2b.cast<float>();

    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  
    pcl::transformPointCloud(*pointcloud, *transformed_cloud, transform);

    Eigen::Vector4f centroid_out;
    pcl::compute3DCentroid(*transformed_cloud, centroid_out);

    Eigen::Matrix3f covariance_matrix;
    computeCovarianceMatrix(*transformed_cloud, centroid_out, covariance_matrix); 

    variance << covariance_matrix(0,0), covariance_matrix(1,1), covariance_matrix(2,2);

    variance = variance / (pointcloud->size() - 1.0);
}

void RepositioningRegion::constructBoundingBox(const Eigen::Vector3d& pos,
                                        Eigen::Vector3d& min_val,
                                        Eigen::Vector3d& max_val,
                                        Eigen::Vector3d& rotations,
                                        Eigen::Vector3d& mean_val,
                                        Eigen::Vector3d& std_val)
{

  ros::Time getlocalpcl_time;

  pcl::PointCloud<pcl::PointXYZI>* local_cloud = new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud_ptr(local_cloud);

  START_TIMER(getlocalpcl_time);
  voxel_map_->getLocalPointcloud(pos, local_pointcloud_range_, 0, *local_cloud, false);
  ROS_INFO_STREAM_COND(
      global_verbosity >= Verbosity::DEBUG,
      planner_log::stage(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "Local point-cloud extraction for branch region: " +
                             planner_log::num(GET_ELAPSED_TIME(getlocalpcl_time)) +
                             " s"));

  pcl::PointCloud<pcl::PointXYZI>* local_filtered = new pcl::PointCloud<pcl::PointXYZI>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_filtered_ptr(local_filtered);

  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(local_cloud_ptr);
  voxel_filter.setLeafSize(voxel_filter_leaf_size_, voxel_filter_leaf_size_, voxel_filter_leaf_size_);
  voxel_filter.filter(*local_filtered);

  current_local_pcl_ = local_filtered_ptr;

  Eigen::Matrix3d rot_w2b;
  Eigen::Vector3d variance;
  
  if (type_ == RepositioningRegionType::kPca) {
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(local_filtered_ptr);
    rot_w2b = pca.getEigenVectors().cast<double>();

    rot_w2b = rot_w2b / rot_w2b.determinant();
    rot_w2b.transposeInPlace(); 

    double cloud_size = local_filtered->size();
    variance = pca.getEigenValues().cast<double>()/(cloud_size - 1);
    variance.cwiseAbs();
  }

  else if (type_ == RepositioningRegionType::kMvbb) {
    pcl::PointCloud<pcl::PointXYZI>* chull_points = new pcl::PointCloud<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr chull_ptr(chull_points);

    pcl::ConvexHull<pcl::PointXYZI> chull;
    chull.setInputCloud(local_cloud_ptr);
    chull.reconstruct(*chull_points);

    boost::function<double(const Eigen::Matrix3d&)> objFun(
      boost::bind(&RepositioningRegion::computeVolume, this, _1, chull_ptr));

    rot_w2b = orientation_search_.runOrientationSearch(objFun);

    computeVarianceInReferenceFrame(variance, local_filtered_ptr, rot_w2b);
  }

  else if (type_ == RepositioningRegionType::kAabb) {
    rot_w2b = Eigen::Matrix3d::Identity();
    computeVarianceInReferenceFrame(variance, local_filtered_ptr, rot_w2b);
  }

  else {
    ROS_WARN_STREAM(planner_log::tag(
        planner_log::Channel::kGlobal, "Global Repositioning",
        "Invalid branch-region bounding box type; using AABB."));
    rot_w2b = Eigen::Matrix3d::Identity();
    computeVarianceInReferenceFrame(variance, local_filtered_ptr, rot_w2b);
  }

  rotationsSafetyCheck(rot_w2b, variance);

  Eigen::Vector4f centroid_out;
  pcl::compute3DCentroid(*local_filtered, centroid_out);

  Eigen::Vector3d pos_c = centroid_out.head(3).cast<double>();
  Eigen::Vector3d offset = rot_w2b*(pos_c - pos);
  
  Eigen::Vector3d sigma = variance.cwiseSqrt();
  double sigma_max = sigma.maxCoeff();
  ROS_INFO_STREAM_COND(
      global_verbosity >= Verbosity::DEBUG,
      planner_log::stage(planner_log::Channel::kGlobal,
                         "Global Repositioning",
                         "Branch-region PCA sigma: " +
                             planner_log::num(sigma[0]) + ", " +
                             planner_log::num(sigma[1]) + ", " +
                             planner_log::num(sigma[2])));

  min_val = offset - sigma/sigma_max*bounding_box_size_max_/2;
  max_val = offset + sigma/sigma_max*bounding_box_size_max_/2;

  rotations = rot_w2b.eulerAngles(2, 1, 0);

  std_val = sigma/sigma_max*distribution_scaling_max_;
  mean_val = Eigen::Vector3d::Zero();
}

bool RepositioningRegion::loadParams(std::string ns)
{
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;

  std::string parse_str;

  param_name = ns + "/type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kPca"))
    type_ = RepositioningRegionType::kPca;
  else if (!parse_str.compare("kMvbb"))
    type_ = RepositioningRegionType::kMvbb;
  else if (!parse_str.compare("kAabb"))
    type_ = RepositioningRegionType::kAabb;
  else {
    ROSPARAM_ERROR(param_name);
    return false;
  }

  param_name = ns + "/local_pointcloud_range";
  if (!ros::param::get(param_name, local_pointcloud_range_)) {
    local_pointcloud_range_ = 15.0;
    ROSPARAM_WARN(param_name, local_pointcloud_range_);
  }

  param_name = ns + "/bounding_box_size_max";
  if (!ros::param::get(param_name, bounding_box_size_max_)) {
    bounding_box_size_max_ = 30.0;
    ROSPARAM_WARN(param_name, bounding_box_size_max_);
  }

  param_name = ns + "/distribution_scaling_max";
  if (!ros::param::get(param_name, distribution_scaling_max_)) {
    distribution_scaling_max_ = 10.0;
    ROSPARAM_WARN(param_name, distribution_scaling_max_);
  }

  param_name = ns + "/voxel_filter_leaf_size";
  if (!ros::param::get(param_name, voxel_filter_leaf_size_)) {
    voxel_filter_leaf_size_ = 1.0;
    ROSPARAM_WARN(param_name, voxel_filter_leaf_size_);
  }

  if (type_ == RepositioningRegionType::kMvbb) {
    OrientationSearchParams param_vals;

    param_name = ns + "/max_iterations";
    if (!ros::param::get(param_name, param_vals.max_iterations)) {
      param_vals.max_iterations = 100;
      ROSPARAM_WARN(param_name, param_vals.max_iterations);
    }

    param_name = ns + "/max_iter_nelder_mead";
    if (!ros::param::get(param_name, param_vals.max_iter_nm)) {
      param_vals.max_iter_nm = 20;
      ROSPARAM_WARN(param_name, param_vals.max_iter_nm);
    }

    param_name = ns + "/population_size";
    if (!ros::param::get(param_name, param_vals.population_size)) {
      param_vals.population_size = 10;
      ROSPARAM_WARN(param_name, param_vals.population_size);
    }

    param_name = ns + "/tol_termination";
    if (!ros::param::get(param_name, param_vals.tol)) {
      param_vals.tol = 1e-2;
      ROSPARAM_WARN(param_name, param_vals.tol);
    }

    param_name = ns + "/tol_iter";
    if (!ros::param::get(param_name, param_vals.tol_iter)) {
      param_vals.tol_iter = 5;
      ROSPARAM_WARN(param_name, param_vals.tol_iter);
    }

    orientation_search_.setParams(param_vals);
  }

  return true;
}
