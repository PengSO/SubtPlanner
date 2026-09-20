#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace gazebo {

class GazeboRosPointCloud : public SensorPlugin {
 public:
  GazeboRosPointCloud() = default;
  ~GazeboRosPointCloud() override = default;

  void Load(sensors::SensorPtr parent, sdf::ElementPtr sdf) override {
    parent_ray_sensor_ = std::dynamic_pointer_cast<sensors::RaySensor>(parent);
    if (!parent_ray_sensor_) {
      gzerr << "[gazebo_ros_pointcloud_plugin] Parent sensor is not a RaySensor.\n";
      return;
    }

    if (!ros::isInitialized()) {
      gzerr << "[gazebo_ros_pointcloud_plugin] ROS is not initialized.\n";
      return;
    }

    robot_namespace_ = sdf->HasElement("robotNamespace")
                           ? sdf->Get<std::string>("robotNamespace")
                           : std::string();
    topic_name_ = sdf->HasElement("topicName") ? sdf->Get<std::string>("topicName")
                                               : std::string("pointcloud");
    frame_name_ = sdf->HasElement("frameName") ? sdf->Get<std::string>("frameName")
                                               : parent_ray_sensor_->Name();
    gaussian_noise_ = sdf->HasElement("gaussianNoise")
                          ? sdf->Get<double>("gaussianNoise")
                          : 0.0;

    rosnode_.reset(new ros::NodeHandle(robot_namespace_));
    pointcloud_pub_ = rosnode_->advertise<sensor_msgs::PointCloud2>(topic_name_, 1);

    new_laser_scans_connection_ =
        parent_ray_sensor_->LaserShape()->ConnectNewLaserScans(
            boost::bind(&GazeboRosPointCloud::OnNewLaserScans, this));
    parent_ray_sensor_->SetActive(true);

    gzdbg << "[gazebo_ros_pointcloud_plugin] Publishing " << topic_name_
          << " in namespace " << robot_namespace_ << " with frame " << frame_name_
          << "\n";
  }

 private:
  void OnNewLaserScans() {
    if (!pointcloud_pub_) {
      return;
    }

    const int horizontal_count = parent_ray_sensor_->RangeCount();
    const int vertical_count = parent_ray_sensor_->VerticalRangeCount();
    if (horizontal_count <= 0 || vertical_count <= 0) {
      return;
    }

    const double h_min = parent_ray_sensor_->AngleMin().Radian();
    const double h_max = parent_ray_sensor_->AngleMax().Radian();
    const double v_min = parent_ray_sensor_->VerticalAngleMin().Radian();
    const double v_max = parent_ray_sensor_->VerticalAngleMax().Radian();
    const double h_step =
        horizontal_count > 1 ? (h_max - h_min) / (horizontal_count - 1) : 0.0;
    const double v_step =
        vertical_count > 1 ? (v_max - v_min) / (vertical_count - 1) : 0.0;
    const double range_min = parent_ray_sensor_->RangeMin();
    const double range_max = parent_ray_sensor_->RangeMax();

    struct Point {
      float x;
      float y;
      float z;
      float intensity;
    };
    std::vector<Point> points;
    points.reserve(static_cast<size_t>(horizontal_count) *
                   static_cast<size_t>(vertical_count));

    for (int v = 0; v < vertical_count; ++v) {
      const double v_angle = v_min + v_step * static_cast<double>(v);
      const double cos_v = std::cos(v_angle);
      const double sin_v = std::sin(v_angle);
      for (int h = 0; h < horizontal_count; ++h) {
        const unsigned int index =
            static_cast<unsigned int>(v * horizontal_count + h);
        double range = parent_ray_sensor_->Range(index);
        if (gaussian_noise_ > 0.0) {
          range += noise_(rng_) * gaussian_noise_;
        }
        if (!std::isfinite(range) || range < range_min || range > range_max) {
          continue;
        }

        const double h_angle = h_min + h_step * static_cast<double>(h);
        const double xy = range * cos_v;
        points.push_back(Point{static_cast<float>(xy * std::cos(h_angle)),
                               static_cast<float>(xy * std::sin(h_angle)),
                               static_cast<float>(range * sin_v), 1.0f});
      }
    }

    sensor_msgs::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = ros::Time::now();
    cloud_msg.header.frame_id = frame_name_;
    cloud_msg.height = 1;
    cloud_msg.width = static_cast<uint32_t>(points.size());

    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32,
                                  "intensity", 1,
                                  sensor_msgs::PointField::FLOAT32);
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");
    for (const Point& point : points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      *iter_i = point.intensity;
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_i;
    }

    cloud_msg.is_dense = true;
    pointcloud_pub_.publish(cloud_msg);
  }

  sensors::RaySensorPtr parent_ray_sensor_;
  event::ConnectionPtr new_laser_scans_connection_;
  std::unique_ptr<ros::NodeHandle> rosnode_;
  ros::Publisher pointcloud_pub_;
  std::string robot_namespace_;
  std::string topic_name_;
  std::string frame_name_;
  double gaussian_noise_ = 0.0;
  std::default_random_engine rng_;
  std::normal_distribution<double> noise_{0.0, 1.0};
};

GZ_REGISTER_SENSOR_PLUGIN(GazeboRosPointCloud)

}  // namespace gazebo
