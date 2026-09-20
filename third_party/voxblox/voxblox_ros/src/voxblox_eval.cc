//=============================================================================================================
//=============================================================================================================

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>
#include <pcl/conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <deque>

#include <voxblox/core/esdf_map.h>
#include <voxblox/core/occupancy_map.h>
#include <voxblox/core/tsdf_map.h>
#include <voxblox/integrator/esdf_integrator.h>
#include <voxblox/integrator/occupancy_integrator.h>
#include <voxblox/integrator/tsdf_integrator.h>
#include <voxblox/io/layer_io.h>
#include <voxblox/io/mesh_ply.h>
#include <voxblox/mesh/mesh_integrator.h>

#include "voxblox_ros/mesh_vis.h"
#include "voxblox_ros/ptcloud_vis.h"

namespace voxblox {

class VoxbloxEvaluator {
 public:
  VoxbloxEvaluator(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);
  void evaluate();
  void visualize();
  bool shouldExit() const { return !visualize_; }

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  bool visualize_;
  bool recolor_by_error_;
  ColorMode color_mode_;
  std::string frame_id_;

  Transformation T_V_G_;

  ros::Publisher mesh_pub_;
  ros::Publisher gt_ptcloud_pub_;

  std::shared_ptr<Layer<TsdfVoxel>> tsdf_layer_;
  pcl::PointCloud<pcl::PointXYZRGB> gt_ptcloud_;

  Interpolator<TsdfVoxel>::Ptr interpolator_;

  std::shared_ptr<MeshLayer> mesh_layer_;
  std::shared_ptr<MeshIntegrator<TsdfVoxel>> mesh_integrator_;
};

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
VoxbloxEvaluator::VoxbloxEvaluator(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      visualize_(true),
      recolor_by_error_(false),
      frame_id_("world") {

  nh_private_.param("visualize", visualize_, visualize_);
  nh_private_.param("recolor_by_error", recolor_by_error_, recolor_by_error_);
  nh_private_.param("frame_id", frame_id_, frame_id_);

  XmlRpc::XmlRpcValue T_V_G_xml;
  if (nh_private_.getParam("T_V_G", T_V_G_xml)) {
    kindr::minimal::xmlRpcToKindr(T_V_G_xml, &T_V_G_);
    bool invert_static_tranform = false;
    nh_private_.param("invert_T_V_G", invert_static_tranform, invert_static_tranform);
    if (invert_static_tranform) {
      T_V_G_ = T_V_G_.inverse();
    }
  }

  std::string voxblox_file_path, gt_file_path;
  CHECK(nh_private_.getParam("voxblox_file_path", voxblox_file_path))
      << "No voxblox_file_path specified.";
  CHECK(nh_private_.getParam("gt_file_path", gt_file_path))
      << "No gt_file_path specified.";

  CHECK(io::LoadLayer<TsdfVoxel>(voxblox_file_path, &tsdf_layer_))
      << "Cannot load " << voxblox_file_path;

  pcl::PLYReader ply_reader;
  CHECK_EQ(ply_reader.read(gt_file_path, gt_ptcloud_), 0)
      << "Cannot load pointcloud from " << gt_file_path;

  interpolator_.reset(new Interpolator<TsdfVoxel>(tsdf_layer_.get()));

  if (visualize_) {
    mesh_pub_ = nh_private_.advertise<visualization_msgs::MarkerArray>("mesh", 1, true);
    gt_ptcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(
        "gt_ptcloud", 1, true);

    std::string color_mode("color");
    nh_private_.param("color_mode", color_mode, color_mode);
    if (color_mode == "color")
      color_mode_ = ColorMode::kColor;
    else if (color_mode == "height")
      color_mode_ = ColorMode::kHeight;
    else if (color_mode == "normals")
      color_mode_ = ColorMode::kNormals;
    else if (color_mode == "lambert")
      color_mode_ = ColorMode::kLambert;
    else
      color_mode_ = ColorMode::kGray;
  }

}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void VoxbloxEvaluator::evaluate() {
  pcl::transformPointCloud(gt_ptcloud_, gt_ptcloud_, T_V_G_.getTransformationMatrix());

  uint64_t total_evaluated_voxels = 0;
  uint64_t unknown_voxels = 0;
  uint64_t outside_truncation_voxels = 0;

  double truncation_distance = 2 * tsdf_layer_->voxel_size();

  double mse = 0.0;

  for (pcl::PointCloud<pcl::PointXYZRGB>::const_iterator it = gt_ptcloud_.begin();
       it != gt_ptcloud_.end(); ++it) {
    Point point(it->x, it->y, it->z);

    FloatingPoint distance = 0.0;
    float weight = 0.0;
    bool valid = false;

    const float min_weight = 0.01;
    const bool interpolate = true;

    if (!interpolator_->getNearestDistanceAndWeight(point, &distance, &weight)) {
      unknown_voxels++;
    } else if (weight <= min_weight) {
      unknown_voxels++;
    } else if (distance >= truncation_distance) {
      outside_truncation_voxels++;
      mse += truncation_distance * truncation_distance;
      valid = true;
    } else {
      interpolator_->getDistance(point, &distance, interpolate);
      mse += distance * distance;
      valid = true;
    }

    if (valid && visualize_ && recolor_by_error_) {
      Layer<TsdfVoxel>::BlockType::Ptr block_ptr = tsdf_layer_->getBlockPtrByCoordinates(point);
      if (block_ptr != nullptr) {
        TsdfVoxel& voxel = block_ptr->getVoxelByCoordinates(point);
        voxel.color = grayColorMap(std::fabs(distance) / truncation_distance);
      }
    }

    total_evaluated_voxels++;
  }

  double rms = sqrt(mse / (total_evaluated_voxels - unknown_voxels));

  std::cout << "================ Voxblox Evaluation ================\n"
            << "RMSE: " << rms << "\n"
            << "Evaluated voxels: " << total_evaluated_voxels << "\n"
            << "Unknown voxels: " << unknown_voxels
            << " (" << static_cast<double>(unknown_voxels)/total_evaluated_voxels << ")\n"
            << "Outside truncation voxels: " << outside_truncation_voxels
            << " (" << static_cast<double>(outside_truncation_voxels)/total_evaluated_voxels << ")\n"
            << "==================================================\n";

  if (visualize_) {
    visualize();
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void VoxbloxEvaluator::visualize() {
  MeshIntegratorConfig mesh_config;
  mesh_layer_.reset(new MeshLayer(tsdf_layer_->block_size()));
  mesh_integrator_.reset(new MeshIntegrator<TsdfVoxel>(
      mesh_config, tsdf_layer_.get(), mesh_layer_.get()));

  constexpr bool only_mesh_updated_blocks = false;
  constexpr bool clear_updated_flag = true;
  mesh_integrator_->generateMesh(only_mesh_updated_blocks, clear_updated_flag);

  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(1);
  marker_array.markers[0].header.frame_id = frame_id_;
  fillMarkerWithMesh(mesh_layer_, color_mode_, &marker_array.markers[0]);
  mesh_pub_.publish(marker_array);

  gt_ptcloud_.header.frame_id = frame_id_;
  gt_ptcloud_pub_.publish(gt_ptcloud_);

}

}  // namespace voxblox

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_evaluator_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::VoxbloxEvaluator eval(nh, nh_private);
  eval.evaluate();

  if (!eval.shouldExit()) {
    ros::spin();
  }
  return 0;
}
