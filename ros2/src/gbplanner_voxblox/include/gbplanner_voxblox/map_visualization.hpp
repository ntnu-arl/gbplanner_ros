// Copyright (c) 2016, ETHZ ASL. All rights reserved.
// Adapted from voxblox_ros/ptcloud_vis.h, fixed Voxblox commit
// 0ee88c26c203a3fe88e348dbbb552ab431a35352. BSD-3-Clause; LICENSE.voxblox.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstring>

#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voxblox/core/color.h>

#include "gbplanner_voxblox/distance_products.hpp"

namespace gbplanner2 {
namespace map_visualization_detail {

inline sensor_msgs::msg::PointCloud2 emptyRgbCloud() {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.is_bigendian = false;
  // Match PCL PointXYZRGB: homogeneous XYZ occupies 16 bytes, followed by
  // packed RGB and 12 bytes of padding. RGB is a float field carrying bits,
  // not a numeric floating-point color value.
  cloud.point_step = 32;
  const char* names[] = {"x", "y", "z", "rgb"};
  const uint32_t offsets[] = {0, 4, 8, 16};
  for (unsigned i = 0; i < 4; ++i) {
    sensor_msgs::msg::PointField field;
    field.name = names[i];
    field.offset = offsets[i];
    field.datatype = sensor_msgs::msg::PointField::FLOAT32;
    field.count = 1;
    cloud.fields.push_back(field);
  }
  return cloud;
}

inline void appendRgbPoint(const voxblox::Point& center,
                           const voxblox::Color& color,
                           PointcloudBuffer<32>* buffer, bool* is_dense) {
  // Initialize the complete record, including PCL's twelve padding bytes.
  uint8_t record[32]{};
  for (unsigned axis = 0; axis < 3; ++axis) {
    distance_product_detail::writeLittleEndianFloat(center(axis), record + axis * 4);
    if (!std::isfinite(center(axis))) *is_dense = false;
  }
  distance_product_detail::writeLittleEndianFloat(1.f, record + 12);
  // The original helper assigns r/g/b to a default PCL point (alpha=255).
  // Preserve those channel semantics rather than copying the voxel's alpha.
  distance_product_detail::writeLittleEndianWord(
      0xFF000000u | (uint32_t(color.r) << 16) |
      (uint32_t(color.g) << 8) | uint32_t(color.b), record + 16);
  buffer->append(record);
}
}  // namespace map_visualization_detail

// Every allocated block is visited; these products are not clipped to the
// planner's dense window. The caller sets all message headers before publish.
inline sensor_msgs::msg::PointCloud2 createDistancePointcloudFromTsdfLayer(
    const voxblox::Layer<voxblox::TsdfVoxel>& layer) {
  return distance_product_detail::fromLayer(layer,
      [](const voxblox::TsdfVoxel& voxel, const voxblox::Point&) {
        return voxel.weight > 1e-3f;
      });
}

inline sensor_msgs::msg::PointCloud2 createDistancePointcloudFromEsdfLayer(
    const voxblox::Layer<voxblox::EsdfVoxel>& layer) {
  return distance_product_detail::fromLayer(layer,
      [](const voxblox::EsdfVoxel& voxel, const voxblox::Point&) {
        return voxel.observed;
      });
}

inline sensor_msgs::msg::PointCloud2 createSurfacePointcloudFromTsdfLayer(
    const voxblox::Layer<voxblox::TsdfVoxel>& layer) {
  auto cloud = map_visualization_detail::emptyRgbCloud();
  PointcloudBuffer<32> buffer(cloud.data);
  const float surface_distance = layer.voxel_size() * .75f;
  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);
  for (const auto& index : blocks) {
    const auto& block = layer.getBlockByIndex(index);
    for (std::size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      // Surface colors use the original >0 weight cutoff, unlike the
      // distance products' >1e-3 cutoff; the distance boundary is strict.
      if (voxel.weight > 0.f && std::abs(voxel.distance) < surface_distance)
        map_visualization_detail::appendRgbPoint(
            block.computeCoordinatesFromLinearIndex(i), voxel.color, &buffer, &cloud.is_dense);
    }
  }
  cloud.width = buffer.finish();
  cloud.row_step = cloud.width * cloud.point_step;
  return cloud;
}

inline visualization_msgs::msg::MarkerArray createOccupancyBlocksFromTsdfLayer(
    const voxblox::Layer<voxblox::TsdfVoxel>& layer,
    voxblox::FloatingPoint occupied_min_distance = 0.f) {
  visualization_msgs::msg::MarkerArray result;
  result.markers.emplace_back();
  auto& marker = result.markers.back();
  marker.ns = "occupied_voxels";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.scale.x = marker.scale.y = marker.scale.z = layer.voxel_size();
  // ROS 2/RViz adaptation: an identity pose and opaque overall color make
  // the original world-coordinate cubes and per-point colors unambiguous.
  marker.pose.orientation.w = 1.;
  marker.color.a = 1.f;
  voxblox::BlockIndexList blocks;
  layer.getAllAllocatedBlocks(&blocks);
  for (const auto& index : blocks) {
    const auto& block = layer.getBlockByIndex(index);
    for (std::size_t i = 0; i < block.num_voxels(); ++i) {
      const auto& voxel = block.getVoxelByLinearIndex(i);
      if (!(voxel.weight > 1e-3f && voxel.distance <= occupied_min_distance))
        continue;
      const auto center = block.computeCoordinatesFromLinearIndex(i);
      geometry_msgs::msg::Point point;
      point.x = center.x();
      point.y = center.y();
      point.z = center.z();
      marker.points.push_back(point);
      const auto color = voxblox::rainbowColorMap((center.z() + 2.5) / 5.0);
      std_msgs::msg::ColorRGBA rgba;
      rgba.r = color.r / 255.f;
      rgba.g = color.g / 255.f;
      rgba.b = color.b / 255.f;
      rgba.a = color.a / 255.f;
      marker.colors.push_back(rgba);
    }
  }
  // An empty ADD is inconsistently handled by visualizers. Delete our one
  // stable marker explicitly so reset/import of an empty map clears RViz.
  if (marker.points.empty()) marker.action = visualization_msgs::msg::Marker::DELETE;
  return result;
}

}  // namespace gbplanner2
