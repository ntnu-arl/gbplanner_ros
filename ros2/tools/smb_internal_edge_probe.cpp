// Focused standalone checks for the production mesh result and map lifetime.
#include "gbplanner_smb/smb_vehicle.hpp"
#include "mesh_contacts.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btTriangleMesh.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

namespace {
void require(bool value, const char* message) {
  if (!value) throw std::runtime_error(message);
}

struct Surface {
  btTriangleMesh mesh;
  gbplanner_smb::detail::MeshEdgeInfoMap info;
  std::unique_ptr<btBvhTriangleMeshShape> shape;
  std::array<btVector3, 3> face;
  Surface(double fold_z, bool reverse) {
    const btVector3 a(-2, -2, 0), b(2, -2, 0), c(2, 2, 0), d(-2, 2, fold_z);
    face = reverse ? std::array<btVector3, 3>{a, c, b} : std::array<btVector3, 3>{a, b, c};
    mesh.addTriangle(face[0], face[1], face[2], false);
    if (reverse) mesh.addTriangle(a, d, c, false);
    else mesh.addTriangle(a, c, d, false);
    info.setGeometry({a, b, c, d}, reverse
        ? std::vector<std::array<int, 3>>{{{0, 2, 1}}, {{0, 3, 2}}}
        : std::vector<std::array<int, 3>>{{{0, 1, 2}}, {{0, 2, 3}}});
    shape = std::make_unique<btBvhTriangleMeshShape>(&mesh, true);
    shape->setMargin(.001);
    btGenerateInternalEdgeInfo(shape.get(), &info);
  }

  btVector3 contact(btVector3 normal, btVector3 point, bool mesh_first, double other_z = 0.,
                    double raw_depth = -.001, double* corrected_depth = nullptr) {
    btCollisionObject mesh_object, other;
    btSphereShape sphere(.1);
    mesh_object.setCollisionShape(shape.get());
    other.setCollisionShape(&sphere);
    btTransform other_pose;
    other_pose.setIdentity();
    other_pose.setOrigin(btVector3(0, 0, other_z));
    other.setWorldTransform(other_pose);
    btTriangleShape triangle(face[0], face[1], face[2]);
    btCollisionObjectWrapper mesh_root(nullptr, shape.get(), &mesh_object,
                                       mesh_object.getWorldTransform(), -1, -1);
    btCollisionObjectWrapper triangle_wrap(&mesh_root, &triangle, &mesh_object,
                                           mesh_object.getWorldTransform(), 0, 0);
    btCollisionObjectWrapper other_wrap(nullptr, &sphere, &other,
                                        other.getWorldTransform(), -1, -1);
    btPersistentManifold manifold(&other, &mesh_object, 0, .02, .02);
    gbplanner_smb::detail::MeshContactResult result(
        mesh_first ? &triangle_wrap : &other_wrap,
        mesh_first ? &other_wrap : &triangle_wrap);
    result.setPersistentManifold(&manifold);
    if (mesh_first) result.setShapeIdentifiersA(0, 0);
    else result.setShapeIdentifiersB(0, 0);
    normal.normalize();
    result.addContactPoint(normal, point, raw_depth);
    require(manifold.getNumContacts() == 1, "corrected contact was not inserted");
    const auto& contact = manifold.getContactPoint(0);
    require(contact.m_index1 == 0 && contact.m_partId1 == 0, "triangle identifiers lost with reversed wrappers");
    require((contact.getPositionWorldOnA() - contact.getPositionWorldOnB() -
        contact.m_normalWorldOnB * contact.getDistance()).length() < 1e-6,
        "contact anchors do not match normal/depth");
    if (corrected_depth) *corrected_depth = contact.getDistance();
    return contact.m_normalWorldOnB;
  }
};

bool foreignContact(btManifoldPoint&, const btCollisionObjectWrapper*, int, int,
                    const btCollisionObjectWrapper*, int, int) { return false; }

void multipleVehicles() {
  const auto previous = gContactAddedCallback;
  gContactAddedCallback = foreignContact;
  const std::vector<Eigen::Vector3d> vertices{{-4, -4, 0}, {4, -4, 0}, {4, 4, 0}, {-4, 4, 0}};
  const std::vector<std::array<int, 3>> faces{{{0, 1, 2}}, {{0, 2, 3}}};
  gbplanner_smb::VehicleParams params;
  params.add_ground = false;
  params.initial_position = {0, 0, .5};
  auto survivor = std::make_unique<gbplanner_smb::SmbVehicle>(params);
  survivor->addTriangleMesh(vertices, faces);
  {
    gbplanner_smb::SmbVehicle temporary(params);
    temporary.addTriangleMesh(vertices, faces);
    for (int i = 0; i < 500; ++i) {
      survivor->step(.002, .1, 0);
      temporary.step(.002, .1, 0);
    }
    require(gContactAddedCallback == foreignContact, "vehicle replaced process-global callback");
  }
  for (int i = 0; i < 500; ++i) survivor->step(.002, .1, 0);
  require(survivor->state().wheel_contacts == 4, "surviving vehicle lost mesh/map after peer destruction");
  survivor.reset();
  require(gContactAddedCallback == foreignContact, "vehicle destruction replaced foreign callback");
  gContactAddedCallback = previous;
}
}  // namespace

int main() {
  try {
    for (bool reverse : {false, true}) {
      Surface flat(0, reverse);
      for (bool mesh_first : {false, true}) {
        require((flat.contact({.3, -.2, 1}, {0, 0, 0}, mesh_first) - btVector3(0, 0, 1)).length() < 1e-6,
                "coplanar top normal not corrected for winding/wrapper order");
        require((flat.contact({.3, -.2, -1}, {0, 0, 0}, mesh_first) - btVector3(0, 0, -1)).length() < 1e-6,
                "coplanar underside normal flipped for winding/wrapper order");
        require((flat.contact({.3, -.2, -1}, {0, 0, 0}, mesh_first, .101) - btVector3(0, 0, 1)).length() < 1e-6,
                "false underside normal retained for fully upper convex support");
        require((flat.contact({.3, -.2, 1}, {0, 0, 0}, mesh_first, -.101) - btVector3(0, 0, -1)).length() < 1e-6,
                "false upper normal retained for fully lower convex support");
        for (double side : {-1., 1.}) {
          double depth;
          flat.contact({0, 0, btScalar(-side)}, {0, 0, 0}, mesh_first, side * .11, .005, &depth);
          require(std::abs(depth - .009) < 1e-6,
                  "impossible raw anchor converted separation into penetration");
        }
      }
      const btVector3 boundary_normal = btVector3(.3, -.2, 1).normalized();
      require((flat.contact(boundary_normal, {0, -2, 0}, false) - boundary_normal).length() < 1e-6,
              "unshared exterior edge was flattened");
    }
    Surface ridge(-1, false), valley(1, false), shallow_fold(.001, false);
    require(shallow_fold.info.planar_neighbors[0][2] == -1,
            "real shallow fold accepted by exact planar-side correction");
    const auto* ridge_info = ridge.info.find(btHashInt(0));
    const auto* valley_info = valley.info.find(btHashInt(0));
    require(ridge_info && valley_info && std::abs(ridge_info->m_edgeV2V0Angle) > .1 &&
            std::abs(valley_info->m_edgeV2V0Angle) > .1,
            "real folded edge angle was classified as coplanar");
    const btVector3 ridge_middle = (btVector3(0, 0, 1) + btVector3(-.25, .25, 1).normalized()).normalized();
    require((ridge.contact(ridge_middle, {0, 0, 0}, false) - ridge_middle).length() < 1e-5,
            "legitimate convex normal cone was flattened");
    const btVector3 valley_face_normal = btVector3(.25, -.25, 1).normalized();
    require((valley.contact(valley_face_normal, {-1, 1, 1}, false) - valley_face_normal).length() < 1e-6,
            "non-edge folded-face normal was changed");
    multipleVehicles();
    std::cout << "{\"coplanar_winding_wrapper_cases\":24,\"invalid_convex_anchor_separation_preserved\":true,\"false_opposite_side_corrected\":true,\"straddling_support_not_reoriented\":true,\"shallow_fold_not_reoriented\":true,\"underside_preserved\":true,"
                 "\"exterior_boundary_preserved\":true,\"convex_cone_preserved\":true,"
                 "\"fold_angles_preserved\":true,\"multiple_world_lifetime\":true,"
                 "\"global_callback_unchanged\":true,\"passed\":true}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
