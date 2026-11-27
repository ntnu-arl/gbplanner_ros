// The MIT License (MIT)
// Copyright (c) 2014 Matthew Klingensmith and Ivan Dryanovski
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// Adapted from voxblox_ros/mesh_vis.h at fixed Voxblox commit
// 0ee88c26c203a3fe88e348dbbb552ab431a35352; original mesh output came from
// OpenChisel (github.com/personalrobotics/OpenChisel).
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <voxblox/core/color.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_msgs/msg/mesh.hpp>

namespace gbplanner2 {

enum class MeshColorMode { Color, Height, Normals, Gray, Lambert, LambertColor };

inline MeshColorMode meshColorModeFromString(const std::string& mode) {
  if (mode.empty() || mode == "color" || mode == "colors") return MeshColorMode::Color;
  if (mode == "height") return MeshColorMode::Height;
  if (mode == "normals") return MeshColorMode::Normals;
  if (mode == "lambert") return MeshColorMode::Lambert;
  if (mode == "lambert_color") return MeshColorMode::LambertColor;
  // Same fallback as upstream; this also gives the local "none" mode gray.
  return MeshColorMode::Gray;
}

namespace mesh_message_detail {
inline std::array<float, 3> normalizedColor(const voxblox::Color& color) {
  return {static_cast<float>(color.r / 255.0),
          static_cast<float>(color.g / 255.0),
          static_cast<float>(color.b / 255.0)};
}

inline std::array<float, 3> vertexColor(const voxblox::Mesh& mesh,
                                       MeshColorMode mode, std::size_t index) {
  switch (mode) {
    case MeshColorMode::Color:
      return normalizedColor(mesh.colors[index]);
    case MeshColorMode::Height: {
      // Original fixed height color scale: -1 m to 10 m.
      const double height = std::min<voxblox::FloatingPoint>(
          std::max<voxblox::FloatingPoint>((mesh.vertices[index].z() + 1.) / 11., 0.), 1.);
      return normalizedColor(voxblox::rainbowColorMap(height));
    }
    case MeshColorMode::Normals:
      return {mesh.normals[index].x() * .5f + .5f,
              mesh.normals[index].y() * .5f + .5f,
              mesh.normals[index].z() * .5f + .5f};
    case MeshColorMode::Gray:
      return {.5f, .5f, .5f};
    case MeshColorMode::Lambert:
    case MeshColorMode::LambertColor: {
      const auto color = mode == MeshColorMode::LambertColor ? mesh.colors[index]
                                                             : voxblox::Color(127, 127, 127);
      const voxblox::Point light1 = voxblox::Point(.8f, -.2f, .7f).normalized();
      const voxblox::Point light2 = voxblox::Point(-.5f, .2f, .2f).normalized();
      const voxblox::Point ambient(.2f, .2f, .2f);
      const voxblox::Point rgb(color.r / 255.0, color.g / 255.0, color.b / 255.0);
      const auto& normal = mesh.normals[index];
      const voxblox::Point shaded = std::max<voxblox::FloatingPoint>(normal.dot(light1), 0.f) * rgb +
                                   std::max<voxblox::FloatingPoint>(normal.dot(light2), 0.f) * rgb + ambient;
      return {std::min(shaded.x(), 1.f), std::min(shaded.y(), 1.f), std::min(shaded.z(), 1.f)};
    }
  }
  throw std::invalid_argument("Unknown mesh color mode");
}

inline void validateMesh(const voxblox::Mesh& mesh, MeshColorMode mode) {
  if (mesh.vertices.size() % 3 != 0)
    throw std::invalid_argument("Mesh transport requires consecutive vertex triples");
  if ((mode == MeshColorMode::Color || mode == MeshColorMode::LambertColor) &&
      mesh.colors.size() != mesh.vertices.size())
    throw std::invalid_argument("Mesh vertex colors are missing");
  // Normals mode transmits no colors: the receiver derives face normals from
  // the triangle vertices and does not need the sender's normals array.
  if ((mode == MeshColorMode::Lambert || mode == MeshColorMode::LambertColor) &&
      mesh.normals.size() != mesh.vertices.size())
    throw std::invalid_argument("Mesh normals are missing for Lambert shading");
}
}  // namespace mesh_message_detail

// Convert the updated block meshes using the original compressed wire layout.
// The caller owns header.frame_id/stamp and reset/late-subscriber publication
// policy. only_updated=false supports a full snapshot without changing flags
// first. This consumes updated flags and removes transmitted empty blocks.
inline void generateVoxbloxMeshMsg(voxblox::MeshLayer* layer, MeshColorMode mode,
                                  voxblox_msgs::msg::Mesh* message,
                                  bool only_updated = true) {
  if (!layer || !message || !std::isfinite(layer->block_size()) || layer->block_size() <= 0.)
    throw std::invalid_argument("Missing mesh layer/message or invalid block size");
  if (mode != MeshColorMode::Color && mode != MeshColorMode::Height &&
      mode != MeshColorMode::Normals && mode != MeshColorMode::Gray &&
      mode != MeshColorMode::Lambert && mode != MeshColorMode::LambertColor)
    throw std::invalid_argument("Unknown mesh color mode");

  voxblox::BlockIndexList indices;
  if (only_updated) layer->getAllUpdatedMeshes(&indices);
  else layer->getAllAllocatedMeshes(&indices);
  voxblox_msgs::msg::Mesh next;
  next.header = message->header;
  next.block_edge_length = layer->block_size();
  next.mesh_blocks.reserve(indices.size());
  for (const auto& index : indices) {
    const auto& mesh = layer->getMeshByIndex(index);
    mesh_message_detail::validateMesh(mesh, mode);
    voxblox_msgs::msg::MeshBlock block;
    for (int axis = 0; axis < 3; ++axis) block.index[axis] = index[axis];
    block.x.reserve(mesh.vertices.size());
    block.y.reserve(mesh.vertices.size());
    block.z.reserve(mesh.vertices.size());
    if (mode != MeshColorMode::Normals) {
      block.r.reserve(mesh.vertices.size());
      block.g.reserve(mesh.vertices.size());
      block.b.reserve(mesh.vertices.size());
    }
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
      // Original Marching Cubes output can extend into the neighboring block;
      // therefore normalize by TWO block lengths. Conversion truncates, despite
      // the old message comments saying "rounded to nearest".
      const voxblox::Point normalized = .5f *
          (layer->block_size_inv() * mesh.vertices[i] - index.cast<voxblox::FloatingPoint>());
      // Keep the fixed upstream producer's range contract, including its
      // squared-norm guard. Throw before clearing any update flags on failure.
      if (!normalized.allFinite() || normalized.squaredNorm() > 1.f ||
          (normalized.array() < 0.f).any())
        throw std::invalid_argument("Mesh vertex outside upstream block encoding range");
      block.x.push_back(static_cast<uint16_t>(65535.f * normalized.x()));
      block.y.push_back(static_cast<uint16_t>(65535.f * normalized.y()));
      block.z.push_back(static_cast<uint16_t>(65535.f * normalized.z()));
      if (mode != MeshColorMode::Normals) {
        const auto rgb = mesh_message_detail::vertexColor(mesh, mode, i);
        for (float channel : rgb)
          if (!std::isfinite(channel) || channel < 0.f || channel > 1.f)
            throw std::invalid_argument("Invalid mesh vertex color");
        block.r.push_back(static_cast<uint8_t>(255.f * rgb[0]));
        block.g.push_back(static_cast<uint8_t>(255.f * rgb[1]));
        block.b.push_back(static_cast<uint8_t>(255.f * rgb[2]));
      }
    }
    next.mesh_blocks.push_back(std::move(block));
  }

  // Commit after all blocks encode successfully. Reusing a message does not
  // append old blocks; a malformed block cannot consume pending updates.
  *message = std::move(next);
  for (const auto& index : indices) {
    auto mesh = layer->getMeshPtrByIndex(index);
    mesh->updated = false;
    if (!mesh->hasVertices()) layer->removeMesh(index);
  }
}

}  // namespace gbplanner2
