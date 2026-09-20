//=============================================================================================================
//=============================================================================================================

#include "voxblox/utils/camera_model.h"

namespace voxblox {

//=============================================================================================================
//=============================================================================================================

/**
 */
void Plane::setFromPoints(const Point& p1, const Point& p2, const Point& p3) {
  Point p1p2 = p2 - p1;
  Point p1p3 = p3 - p1;

  Point cross = p1p2.cross(p1p3);
  normal_ = cross.normalized();

  distance_ = normal_.dot(p1);
}

/**
 */
void Plane::setFromDistanceNormal(const Point& normal, double distance) {
  normal_ = normal;
  distance_ = distance;
}

/**
 */
bool Plane::isPointInside(const Point& point) const {
  if (point.dot(normal_) >= distance_) {
    return true;
  }
  return false;
}

//=============================================================================================================
//=============================================================================================================

/**
 */
void CameraModel::setIntrinsicsFromFocalLength(
    const Eigen::Matrix<FloatingPoint, 2, 1>& resolution, double focal_length,
    double min_distance, double max_distance) {
  double horizontal_fov = 2 * std::atan(resolution.x() / (2 * focal_length));
  double vertical_fov = 2 * std::atan(resolution.y() / (2 * focal_length));

  setIntrinsicsFromFoV(horizontal_fov, vertical_fov, min_distance, max_distance);
}

/**
 */
void CameraModel::setIntrinsicsFromFoV(double horizontal_fov,
                                       double vertical_fov, double min_distance,
                                       double max_distance) {
  corners_C_.clear();
  corners_C_.reserve(8);

  double tan_half_h_fov = std::tan(horizontal_fov / 2.0);
  double tan_half_v_fov = std::tan(vertical_fov / 2.0);

  corners_C_.emplace_back(Point(min_distance,
                                min_distance * tan_half_h_fov,
                                min_distance * tan_half_v_fov));
  corners_C_.emplace_back(Point(min_distance,
                                min_distance * tan_half_h_fov,
                                -min_distance * tan_half_v_fov));
  corners_C_.emplace_back(Point(min_distance,
                                -min_distance * tan_half_h_fov,
                                -min_distance * tan_half_v_fov));
  corners_C_.emplace_back(Point(min_distance,
                                -min_distance * tan_half_h_fov,
                                min_distance * tan_half_v_fov));

  corners_C_.emplace_back(Point(max_distance,
                                max_distance * tan_half_h_fov,
                                max_distance * tan_half_v_fov));
  corners_C_.emplace_back(Point(max_distance,
                                max_distance * tan_half_h_fov,
                                -max_distance * tan_half_v_fov));
  corners_C_.emplace_back(Point(max_distance,
                                -min_distance * tan_half_h_fov,
                                -max_distance * tan_half_v_fov));
  corners_C_.emplace_back(Point(max_distance,
                                -max_distance * tan_half_h_fov,
                                max_distance * tan_half_v_fov));

  initialized_ = true;
}

/**
 */
void CameraModel::setExtrinsics(const Transformation& T_C_B) {
  T_C_B_ = T_C_B;
}

/**
 */
Transformation CameraModel::getCameraPose() const {
  return T_G_C_;
}

/**
 */
Transformation CameraModel::getBodyPose() const {
  return T_G_C_ * T_C_B_;
}

/**
 */
void CameraModel::setCameraPose(const Transformation& cam_pose) {
  T_G_C_ = cam_pose;
  calculateBoundingPlanes();
}

/**
 */
void CameraModel::setBodyPose(const Transformation& body_pose) {
  setCameraPose(body_pose * T_C_B_.inverse());
}

/**
 */
void CameraModel::calculateBoundingPlanes() {
  if (!initialized_) return;

  CHECK_EQ(corners_C_.size(), 8u);
  if (bounding_planes_.empty()) {
    bounding_planes_.resize(6);
  }

  AlignedVector<Point> corners_G(corners_C_.size());

  for (size_t i = 0; i < corners_C_.size(); ++i) {
    corners_G[i] = T_G_C_ * corners_C_[i];
  }

  bounding_planes_[0].setFromPoints(corners_G[0], corners_G[2], corners_G[1]);
  bounding_planes_[1].setFromPoints(corners_G[4], corners_G[5], corners_G[6]);
  bounding_planes_[2].setFromPoints(corners_G[3], corners_G[6], corners_G[2]);
  bounding_planes_[3].setFromPoints(corners_G[0], corners_G[5], corners_G[4]);
  bounding_planes_[4].setFromPoints(corners_G[3], corners_G[4], corners_G[7]);
  bounding_planes_[5].setFromPoints(corners_G[2], corners_G[6], corners_G[5]);

  aabb_min_.setConstant(std::numeric_limits<double>::max());
  aabb_max_.setConstant(std::numeric_limits<double>::lowest());

  for (int i = 0; i < 3; i++) {
    for (size_t j = 0; j < corners_G.size(); j++) {
      aabb_min_(i) = std::min(aabb_min_(i), corners_G[j](i));
      aabb_max_(i) = std::max(aabb_max_(i), corners_G[j](i));
    }
  }
}

/**
 */
void CameraModel::getAabb(Point* aabb_min, Point* aabb_max) const {
  *aabb_min = aabb_min_;
  *aabb_max = aabb_max_;
}

/**
 */
bool CameraModel::isPointInView(const Point& point) const {
  for (size_t i = 0; i < bounding_planes_.size(); i++) {
    if (!bounding_planes_[i].isPointInside(point)) {
      return false;
    }
  }
  return true;
}

/**
 */
void CameraModel::getBoundingLines(AlignedVector<Point>* lines) const {
  CHECK_NOTNULL(lines);
  lines->clear();
  lines->reserve(24);

  AlignedVector<Point> corners_G(corners_C_.size());
  for (size_t i = 0; i < corners_C_.size(); ++i) {
    corners_G[i] = T_G_C_ * corners_C_[i];
  }

  lines->push_back(corners_G[0]); lines->push_back(corners_G[1]);
  lines->push_back(corners_G[1]); lines->push_back(corners_G[2]);
  lines->push_back(corners_G[2]); lines->push_back(corners_G[3]);
  lines->push_back(corners_G[3]); lines->push_back(corners_G[0]);

  lines->push_back(corners_G[4]); lines->push_back(corners_G[5]);
  lines->push_back(corners_G[5]); lines->push_back(corners_G[6]);
  lines->push_back(corners_G[6]); lines->push_back(corners_G[7]);
  lines->push_back(corners_G[7]); lines->push_back(corners_G[4]);

  lines->push_back(corners_G[0]); lines->push_back(corners_G[4]);
  lines->push_back(corners_G[1]); lines->push_back(corners_G[5]);
  lines->push_back(corners_G[2]); lines->push_back(corners_G[6]);
  lines->push_back(corners_G[3]); lines->push_back(corners_G[7]);
}

/**
 */
void CameraModel::getFarPlanePoints(AlignedVector<Point>* points) const {
  CHECK_NOTNULL(points);
  points->clear();
  points->reserve(3);

  points->push_back(T_G_C_ * corners_C_[4]);
  points->push_back(T_G_C_ * corners_C_[5]);
  points->push_back(T_G_C_ * corners_C_[6]);
}

}  // namespace voxblox
