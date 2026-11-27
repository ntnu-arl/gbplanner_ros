#include "gbplanner_smb/triangle_mesh.hpp"

#include <filesystem>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace gbplanner_smb {
TriangleMesh readTriangleMesh(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("Cannot open collision mesh: " + path);
  std::string magic;
  int version = 0;
  long long vertices = 0, triangles = 0;
  if (!(input >> magic >> version >> vertices >> triangles) ||
      magic != "GBPLANNER_TRIANGLE_MESH" || version != 1 || vertices < 3 || triangles < 1)
    throw std::runtime_error("Invalid collision mesh header: " + path);
  // Even the shortest xyz/index record occupies six bytes in this format.
  // Check against file size before allocating for malformed count fields.
  const auto bytes = std::filesystem::file_size(path);
  if (vertices > std::numeric_limits<int>::max() || triangles > std::numeric_limits<int>::max() ||
      static_cast<unsigned long long>(vertices + triangles) > bytes / 6)
    throw std::runtime_error("Invalid collision mesh counts: " + path);
  TriangleMesh mesh;
  mesh.vertices.resize(static_cast<std::size_t>(vertices));
  mesh.triangles.resize(static_cast<std::size_t>(triangles));
  for (auto& point : mesh.vertices)
    if (!(input >> point.x() >> point.y() >> point.z()) || !point.allFinite())
      throw std::runtime_error("Invalid collision mesh vertex: " + path);
  for (auto& face : mesh.triangles) {
    if (!(input >> face[0] >> face[1] >> face[2]))
      throw std::runtime_error("Invalid collision mesh face: " + path);
    for (int index : face)
      if (index < 0 || index >= vertices)
        throw std::runtime_error("Collision mesh index out of range: " + path);
  }
  input >> std::ws;
  if (input.bad() || !input.eof())
    throw std::runtime_error("Unexpected collision mesh trailing data: " + path);
  return mesh;
}
}  // namespace gbplanner_smb
