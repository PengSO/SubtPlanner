//=============================================================================================================
//=============================================================================================================

#include <limits>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <voxblox/mesh/mesh_utils.h>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

unsigned int VoxbloxMeshVisual::instance_counter_ = 0;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
VoxbloxMeshVisual::VoxbloxMeshVisual(Ogre::SceneManager* scene_manager,
                                     Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_ = parent_node->createChildSceneNode();
  instance_number_ = instance_counter_++;
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
VoxbloxMeshVisual::~VoxbloxMeshVisual() {
  for (std::pair<const voxblox::BlockIndex, Ogre::ManualObject*> ogre_object :
       object_map_) {
    scene_manager_->destroyManualObject(ogre_object.second);
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void VoxbloxMeshVisual::setMessage(const voxblox_msgs::Mesh::ConstPtr& msg) {
  for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
    const voxblox::BlockIndex index(mesh_block.index[0], mesh_block.index[1],
                                    mesh_block.index[2]);

    size_t vertex_index = 0u;
    voxblox::Mesh mesh;
    mesh.vertices.reserve(mesh_block.x.size());
    mesh.indices.reserve(mesh_block.x.size());

    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      constexpr float point_conv_factor =
          2.0f / std::numeric_limits<uint16_t>::max();
      const float mesh_x =
          (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
           static_cast<float>(index[0])) *
          msg->block_edge_length;
      const float mesh_y =
          (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
           static_cast<float>(index[1])) *
          msg->block_edge_length;
      const float mesh_z =
          (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
           static_cast<float>(index[2])) *
          msg->block_edge_length;

      mesh.indices.push_back(vertex_index++);
      mesh.vertices.emplace_back(mesh_x, mesh_y, mesh_z);
    }

    mesh.normals.reserve(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); i += 3) {
      const voxblox::Point dir0 = mesh.vertices[i] - mesh.vertices[i + 1];
      const voxblox::Point dir1 = mesh.vertices[i] - mesh.vertices[i + 2];
      const voxblox::Point normal = dir0.cross(dir1).normalized();

      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
      mesh.normals.push_back(normal);
    }

    mesh.colors.reserve(mesh.vertices.size());
    const bool has_color = mesh_block.x.size() == mesh_block.r.size();
    for (size_t i = 0; i < mesh_block.x.size(); ++i) {
      voxblox::Color color;
      if (has_color) {
        color.r = mesh_block.r[i];
        color.g = mesh_block.g[i];
        color.b = mesh_block.b[i];
      } else {
        color.r = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].x() * 0.5f + 0.5f);
        color.g = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].y() * 0.5f + 0.5f);
        color.b = std::numeric_limits<uint8_t>::max() *
                  (mesh.normals[i].z() * 0.5f + 0.5f);
      }
      color.a = std::numeric_limits<uint8_t>::max();
      mesh.colors.push_back(color);
    }

    voxblox::Mesh connected_mesh;
    voxblox::createConnectedMesh(mesh, &connected_mesh);

    Ogre::ManualObject* ogre_object;
    const voxblox::AnyIndexHashMapType<
        Ogre::ManualObject*>::type::const_iterator it = object_map_.find(index);

    if (it != object_map_.end()) {
      if (mesh_block.x.size() == 0) {
        scene_manager_->destroyManualObject(it->second);
        object_map_.erase(it);
        continue;
      }
      ogre_object = it->second;
      ogre_object->clear();
    } else {
      std::string object_name = std::to_string(index.x()) + " " +
                                std::to_string(index.y()) + " " +
                                std::to_string(index.z()) + " " +
                                std::to_string(instance_number_);
      ogre_object = scene_manager_->createManualObject(object_name);
      object_map_.insert(std::make_pair(index, ogre_object));
      frame_node_->attachObject(ogre_object);
    }

    ogre_object->estimateVertexCount(connected_mesh.vertices.size());
    ogre_object->estimateIndexCount(connected_mesh.indices.size());
    ogre_object->begin("BaseWhiteNoLighting",
                       Ogre::RenderOperation::OT_TRIANGLE_LIST);

    for (size_t i = 0; i < connected_mesh.vertices.size(); ++i) {
      ogre_object->position(connected_mesh.vertices[i].x(),
                            connected_mesh.vertices[i].y(),
                            connected_mesh.vertices[i].z());

      ogre_object->normal(connected_mesh.normals[i].x(),
                          connected_mesh.normals[i].y(),
                          connected_mesh.normals[i].z());

      // Brighten the rendered mesh for RViz presentation only. This does not
      // modify the map data; it only lifts dark mesh colors before rendering.
      constexpr float color_conv_factor =
          1.0f / std::numeric_limits<uint8_t>::max();
      constexpr float kDisplayBrightness = 1.35f;
      constexpr float kDisplayAmbient = 0.12f;
      const float red = std::min(
          1.0f, kDisplayAmbient + kDisplayBrightness * color_conv_factor *
                                    static_cast<float>(connected_mesh.colors[i].r));
      const float green = std::min(
          1.0f, kDisplayAmbient + kDisplayBrightness * color_conv_factor *
                                    static_cast<float>(connected_mesh.colors[i].g));
      const float blue = std::min(
          1.0f, kDisplayAmbient + kDisplayBrightness * color_conv_factor *
                                    static_cast<float>(connected_mesh.colors[i].b));
      ogre_object->colour(
          red,
          green,
          blue,
          color_conv_factor * static_cast<float>(connected_mesh.colors[i].a));
    }

    for (const voxblox::VertexIndex index : connected_mesh.indices) {
      ogre_object->index(index);
    }

    ogre_object->end();
  }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void VoxbloxMeshVisual::setFramePosition(const Ogre::Vector3& position) {
  frame_node_->setPosition(position);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void VoxbloxMeshVisual::setFrameOrientation(
    const Ogre::Quaternion& orientation) {
  frame_node_->setOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin
