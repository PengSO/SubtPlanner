//=============================================================================================================
//=============================================================================================================

#include "voxblox/core/tsdf_map.h"

namespace voxblox {

//=========================================================================================
//=========================================================================================
unsigned int TsdfMap::coordPlaneSliceGetDistanceWeight(
    unsigned int free_plane_index, double free_plane_val,
    EigenDRef<Eigen::Matrix<double, 3, Eigen::Dynamic>>& positions,
    Eigen::Ref<Eigen::VectorXd> distances, Eigen::Ref<Eigen::VectorXd> weights,
    unsigned int max_points) const {
  BlockIndexList blocks;
  tsdf_layer_->getAllAllocatedBlocks(&blocks);

  const size_t vps = tsdf_layer_->voxels_per_side();
  const size_t num_voxels_per_block = vps * vps * vps;

  bool did_all_fit = true;
  unsigned int count = 0;

  for (const BlockIndex& index : blocks) {
    const Block<TsdfVoxel>& block = tsdf_layer_->getBlockByIndex(index);

    if (!block.has_data()) {
      continue;
    }

    const Point origin = block.origin();
    if (std::abs(origin(free_plane_index) - free_plane_val) >
        block.block_size()) {
      continue;
    }

    for (size_t linear_index = 0; linear_index < num_voxels_per_block;
         ++linear_index) {
      const Point coord = block.computeCoordinatesFromLinearIndex(linear_index);
      const TsdfVoxel& voxel = block.getVoxelByLinearIndex(linear_index);
      
      if (std::abs(coord(free_plane_index) - free_plane_val) <=
          block.voxel_size()) {
        const double distance = voxel.distance;
        const double weight = voxel.weight;

        if (count < positions.cols()) {
          positions.col(count) =
              Eigen::Vector3d(coord.x(), coord.y(), coord.z());
        } else {
          did_all_fit = false;
        }

        if (count < distances.size() && count < weights.size()) {
          distances(count) = distance;
          weights(count) = weight;
        } else {
          did_all_fit = false;
        }

        count++;
        if (count >= max_points) {
          return count;
        }
      }
    }
  }

  if (!did_all_fit) {
    throw std::runtime_error(std::string("Unable to store ") +
                             std::to_string(count) + " values.");
  }

  return count;
}

//=========================================================================================
//=========================================================================================
std::string TsdfMap::Config::print() const {
  std::stringstream ss;
  // clang-format off
  ss << "====================== TSDF Map Config ========================\n";
  ss << " - tsdf_voxel_size:               " << tsdf_voxel_size << "\n";
  ss << " - tsdf_voxels_per_side:          " << tsdf_voxels_per_side << "\n";
  ss << "==============================================================\n";
  // clang-format on
  return ss.str();
}

}  // namespace voxblox
