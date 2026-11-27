// Inspect the actual ignition-common mesh loader without starting Gazebo.
// CSV exports preserve loaded vertex order and triangle winding so the result
// can be compared with the original COLLADA positions and indexed triangles.
#include <ignition/common/Mesh.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/config.hh>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace {
using Vector = ignition::math::Vector3d;

void vectorJson(std::ostream& stream, const Vector& point) {
  stream << '[' << point.X() << ',' << point.Y() << ',' << point.Z() << ']';
}

void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
}  // namespace

int main(int argc, char** argv) {
  try {
    require(argc == 3, "Usage: check_pittsburgh_mesh INPUT.dae OUTPUT_PREFIX");
    const auto input = std::filesystem::absolute(argv[1]);
    const std::filesystem::path output(argv[2]);
    require(std::filesystem::is_regular_file(input), "Input DAE is missing");
    if (!output.parent_path().empty())
      std::filesystem::create_directories(output.parent_path());

    // This is the installed loader used by Fortress, not a private XML mesh
    // reconstruction. Do not apply any inferred Y_UP correction here.
    const auto* mesh = ignition::common::MeshManager::Instance()->Load(input.string());
    require(mesh != nullptr && mesh->SubMeshCount() > 0, "Native mesh load failed");
    std::ofstream vertices(output.string() + "_vertices.csv");
    std::ofstream triangles(output.string() + "_triangles.csv");
    require(vertices.good() && triangles.good(), "Cannot open CSV output");
    vertices << std::setprecision(17) << "submesh,index,x,y,z\n";
    triangles << "submesh,index,a,b,c\n";

    const double infinity = std::numeric_limits<double>::infinity();
    Vector minimum(infinity, infinity, infinity), maximum(-infinity, -infinity, -infinity);
    std::size_t vertex_count = 0, index_count = 0, triangle_count = 0;
    std::size_t repeated_index_triangles = 0, zero_area_triangles = 0;
    std::ostringstream children;
    children << std::setprecision(17) << '[';
    for (unsigned int s = 0; s < mesh->SubMeshCount(); ++s) {
      const auto submesh = mesh->SubMeshByIndex(s).lock();
      require(bool(submesh), "Native submesh is unavailable");
      require(submesh->SubMeshPrimitiveType() == ignition::common::SubMesh::TRIANGLES,
              "Non-triangle native primitive");
      require(submesh->IndexCount() % 3 == 0, "Incomplete native triangle indices");
      for (unsigned int i = 0; i < submesh->VertexCount(); ++i) {
        const auto point = submesh->Vertex(i);
        for (unsigned int axis = 0; axis < 3; ++axis) {
          require(std::isfinite(point[axis]), "Nonfinite native vertex");
          minimum[axis] = std::min(minimum[axis], point[axis]);
          maximum[axis] = std::max(maximum[axis], point[axis]);
        }
        vertices << s << ',' << i << ',' << point.X() << ',' << point.Y() << ',' << point.Z() << '\n';
      }
      for (unsigned int i = 0; i < submesh->IndexCount(); i += 3) {
        std::array<int, 3> indices;
        for (unsigned int k = 0; k < 3; ++k) {
          indices[k] = submesh->Index(i + k);
          require(indices[k] >= 0 && static_cast<unsigned int>(indices[k]) < submesh->VertexCount(),
                  "Out-of-range native triangle index");
        }
        repeated_index_triangles += indices[0] == indices[1] || indices[1] == indices[2] || indices[0] == indices[2];
        const auto a = submesh->Vertex(indices[0]), b = submesh->Vertex(indices[1]), c = submesh->Vertex(indices[2]);
        zero_area_triangles += (b - a).Cross(c - a).SquaredLength() == 0.;
        triangles << s << ',' << i / 3 << ',' << indices[0] << ',' << indices[1] << ',' << indices[2] << '\n';
      }
      vertex_count += submesh->VertexCount();
      index_count += submesh->IndexCount();
      triangle_count += submesh->IndexCount() / 3;
      if (s) children << ',';
      children << "{\"name\":" << std::quoted(submesh->Name())
               << ",\"primitive\":\"TRIANGLES\",\"vertices\":" << submesh->VertexCount()
               << ",\"indices\":" << submesh->IndexCount() << '}';
    }
    children << ']';
    require(vertex_count > 0 && triangle_count > 0, "Empty native geometry");
    require(vertex_count == mesh->VertexCount() && index_count == mesh->IndexCount(),
            "Mesh and submesh counts disagree");
    vertices.close(); triangles.close();
    require(!vertices.fail() && !triangles.fail(), "CSV output failed");

    std::ofstream report(output.string() + ".json");
    require(report.good(), "Cannot open JSON output");
    report << std::setprecision(17)
           << "{\n  \"status\":\"passed\",\n  \"scope\":\"Static native mesh loading only; no SDF placement, raycast, support, Gazebo physics or robot motion\",\n"
           << "  \"loader\":\"ignition::common::MeshManager::Load\",\n"
           << "  \"library_version\":" << std::quoted(IGNITION_COMMON_VERSION_FULL) << ",\n"
           << "  \"input\":" << std::quoted(input.string()) << ",\n"
           << "  \"submesh_count\":" << mesh->SubMeshCount() << ",\n"
           << "  \"vertex_count\":" << vertex_count << ",\n"
           << "  \"index_count\":" << index_count << ",\n"
           << "  \"triangle_count\":" << triangle_count << ",\n"
           << "  \"all_vertices_finite\":true,\n  \"all_indices_valid\":true,\n"
           << "  \"repeated_index_triangles\":" << repeated_index_triangles << ",\n"
           << "  \"zero_area_triangles\":" << zero_area_triangles << ",\n"
           << "  \"vertex_bounds\":{\"min\":";
    vectorJson(report, minimum); report << ",\"max\":"; vectorJson(report, maximum);
    report << "},\n  \"native_reported_bounds\":{\"min\":";
    vectorJson(report, mesh->Min()); report << ",\"max\":"; vectorJson(report, mesh->Max());
    report << "},\n  \"submeshes\":" << children.str() << "\n}\n";
    report.close(); require(!report.fail(), "JSON output failed");
    std::cout << "Loaded " << vertex_count << " vertices and " << triangle_count
              << " triangles with ignition-common " << IGNITION_COMMON_VERSION_FULL << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "Native mesh inspection failed: " << error.what() << '\n';
    return 1;
  }
}
