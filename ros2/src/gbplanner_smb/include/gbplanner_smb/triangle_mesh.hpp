#pragma once

#include <Eigen/Core>
#include <array>
#include <string>
#include <vector>

namespace gbplanner_smb {

// Vertices already carry the source model's world transform. The file format
// is the deterministic output of tools/convert_edgar_collision.py.
struct TriangleMesh {
  std::vector<Eigen::Vector3d> vertices;
  std::vector<std::array<int, 3>> triangles;
};

TriangleMesh readTriangleMesh(const std::string& path);

}  // namespace gbplanner_smb
