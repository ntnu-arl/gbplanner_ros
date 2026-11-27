#pragma once

#include <BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h>
#include <BulletCollision/CollisionDispatch/btCollisionDispatcher.h>
#include <BulletCollision/CollisionDispatch/btInternalEdgeUtility.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>
#include <BulletCollision/CollisionShapes/btCollisionShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btConvexShape.h>
#include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <algorithm>
#include <array>
#include <limits>
#include <map>
#include <vector>

namespace gbplanner_smb::detail {

// Keep exact adjacency separately from Bullet's angular smoothing tolerance:
// a shallow but real fold must not become a two-sided planar contact.
struct MeshEdgeInfoMap final : btTriangleInfoMap {
  std::vector<std::array<btVector3, 3>> faces;
  std::vector<std::array<int, 3>> planar_neighbors;

  void setGeometry(const std::vector<btVector3>& vertices,
                   const std::vector<std::array<int, 3>>& triangles) {
    std::map<std::array<btScalar, 3>, int> unique_vertices;
    std::vector<int> ids;
    for (const auto& v : vertices) {
      const std::array<btScalar, 3> key{v.x(), v.y(), v.z()};
      const auto entry = unique_vertices.emplace(key, static_cast<int>(unique_vertices.size()));
      ids.push_back(entry.first->second);
    }
    using FaceEdge = std::pair<int, int>;
    std::map<std::pair<int, int>, std::vector<FaceEdge>> adjacency;
    for (std::size_t i = 0; i < triangles.size(); ++i) {
      const auto& t = triangles[i];
      faces.push_back({vertices[t[0]], vertices[t[1]], vertices[t[2]]});
      planar_neighbors.push_back({-1, -1, -1});
      for (int edge = 0; edge < 3; ++edge) {
        const int a = ids[t[edge]], b = ids[t[(edge + 1) % 3]];
        adjacency[std::minmax(a, b)].push_back({static_cast<int>(i), edge});
      }
    }
    const btScalar epsilon = 8 * std::numeric_limits<btScalar>::epsilon();
    for (const auto& entry : adjacency) {
      if (entry.second.size() != 2) continue;  // Boundary or non-manifold.
      const auto [a, ae] = entry.second[0];
      const auto [b, be] = entry.second[1];
      const auto& fa = faces[a];
      const auto& fb = faces[b];
      const btVector3 na = (fa[1] - fa[0]).cross(fa[2] - fa[0]).normalized();
      const btVector3 nb = (fb[1] - fb[0]).cross(fb[2] - fb[0]).normalized();
      const btScalar scale = btMax(btScalar(1), (fa[1] - fa[0]).length() + (fb[2] - fb[0]).length());
      if (na.cross(nb).length() > epsilon) continue;
      bool coplanar = true;
      for (const auto& v : fb)
        if (btFabs(na.dot(v - fa[0])) > epsilon * scale) coplanar = false;
      if (coplanar) {
        planar_neighbors[a][ae] = b;
        planar_neighbors[b][be] = a;
      }
    }
  }
};

inline bool insideTriangle(const btVector3& point, const std::array<btVector3, 3>& face) {
  const btVector3 normal = (face[1] - face[0]).cross(face[2] - face[0]).normalized();
  const btScalar epsilon = 8 * std::numeric_limits<btScalar>::epsilon();
  for (int i = 0; i < 3; ++i) {
    const btVector3 edge = face[(i + 1) % 3] - face[i];
    if (edge.cross(point - face[i]).dot(normal) < -epsilon * btMax(btScalar(1), edge.length2()))
      return false;
  }
  return true;
}

inline bool orientPlanarContact(btManifoldPoint& contact,
                                const btCollisionObjectWrapper* mesh,
                                const btCollisionObjectWrapper* other,
                                int part, int triangle) {
  const auto* shape = static_cast<const btBvhTriangleMeshShape*>(mesh->getCollisionObject()->getCollisionShape());
  const auto* geometry = dynamic_cast<const MeshEdgeInfoMap*>(shape->getTriangleInfoMap());
  if (!geometry || part != 0 || triangle < 0 ||
      static_cast<std::size_t>(triangle) >= geometry->faces.size() ||
      !other->getCollisionShape()->isConvex()) return false;
  const auto& face = geometry->faces[triangle];
  int closest = -1;
  btScalar distance_squared = BT_LARGE_FLOAT;
  for (int i = 0; i < 3; ++i) {
    const btVector3 edge = face[(i + 1) % 3] - face[i];
    const btScalar u = btClamped((contact.m_localPointB - face[i]).dot(edge) / edge.length2(), btScalar(0), btScalar(1));
    const btScalar d = (contact.m_localPointB - face[i] - edge * u).length2();
    if (d < distance_squared) { distance_squared = d; closest = i; }
  }
  if (closest < 0 || geometry->planar_neighbors[triangle][closest] < 0 ||
      distance_squared >= geometry->m_edgeDistanceThreshold * geometry->m_edgeDistanceThreshold) return false;
  const btVector3 local_normal = (face[1] - face[0]).cross(face[2] - face[0]).normalized();
  btVector3 normal = mesh->getWorldTransform().getBasis() * local_normal;
  const btVector3 plane_point = mesh->getWorldTransform() * face[0];
  const auto* convex = static_cast<const btConvexShape*>(other->getCollisionShape());
  const btVector3 direction = other->getWorldTransform().getBasis().transpose() * normal;
  const btVector3 support_lo = other->getWorldTransform() * convex->localGetSupportingVertex(-direction);
  const btVector3 support_hi = other->getWorldTransform() * convex->localGetSupportingVertex(direction);
  const btScalar lo = normal.dot(support_lo - plane_point);
  const btScalar hi = normal.dot(support_hi - plane_point);
  const btScalar epsilon = 8 * std::numeric_limits<btScalar>::epsilon();
  // Use the complete convex support interval, including its existing margin.
  // An actually straddling body has no unambiguous surface side; retain
  // Bullet's normal in that case rather than guessing from its origin.
  bool upper_side = true;
  if (lo >= -epsilon && hi > epsilon) {}
  else if (hi <= epsilon && lo < -epsilon) { normal = -normal; upper_side = false; }
  else return false;
  // A false opposite-side triangle result can place A outside the entire
  // convex support interval. Retaining that impossible anchor while changing
  // N would turn positive separation into a large fictitious penetration.
  // Replace only that proven-invalid anchor with the actual convex support;
  // keep an admissible original anchor and the configured mesh skin.
  const btScalar anchor = local_normal.dot(mesh->getWorldTransform().invXform(contact.m_positionWorldOnA) - face[0]);
  const btVector3 point_a = anchor < lo - epsilon || anchor > hi + epsilon
      ? (upper_side ? support_lo : support_hi) : contact.m_positionWorldOnA;
  const btScalar depth = normal.dot(point_a - plane_point) - shape->getMargin();
  const btVector3 point_b = point_a - normal * depth;
  const btVector3 point_on_surface = mesh->getWorldTransform().invXform(point_b - normal * shape->getMargin());
  const int neighbor = geometry->planar_neighbors[triangle][closest];
  if (!insideTriangle(point_on_surface, face) &&
      !insideTriangle(point_on_surface, geometry->faces[neighbor])) return false;
  contact.m_positionWorldOnA = point_a;
  contact.m_localPointA = other->getWorldTransform().invXform(point_a);
  contact.m_normalWorldOnB = normal;
  contact.m_distance1 = depth;
  contact.m_positionWorldOnB = point_b;
  contact.m_localPointB = mesh->getWorldTransform().invXform(point_b);
  return true;
}

inline void adjustMeshContact(btManifoldPoint& contact,
                              const btCollisionObjectWrapper* mesh,
                              const btCollisionObjectWrapper* other,
                              int part, int triangle) {
  if (orientPlanarContact(contact, mesh, other, part, triangle)) return;
  // Bvh meshes already collide from either side. Retain that behaviour while
  // using Bullet's adjacent-face angles to clamp internal-edge normals. These
  // flags do not replace a genuine convex/concave fold with a plane.
  btAdjustInternalEdgeContacts(contact, mesh, other, part, triangle,
      BT_TRIANGLE_CONCAVE_DOUBLE_SIDED | BT_TRIANGLE_CONVEX_DOUBLE_SIDED);
}

class MeshContactResult final : public btManifoldResult {
 public:
  MeshContactResult(const btCollisionObjectWrapper* a, const btCollisionObjectWrapper* b)
      : btManifoldResult(a, b) {
    setShapeIdentifiersA(-1, -1);
    setShapeIdentifiersB(-1, -1);
  }

  void addContactPoint(const btVector3& normal, const btVector3& point, btScalar depth) override {
    const bool triangle_a = m_body0Wrap->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE;
    const bool triangle_b = m_body1Wrap->getCollisionShape()->getShapeType() == TRIANGLE_SHAPE_PROXYTYPE;
    const auto* mesh = triangle_a ? m_body0Wrap : m_body1Wrap;
    const auto* other = triangle_a ? m_body1Wrap : m_body0Wrap;
    // Bullet's convex/concave algorithm fixes the manifold order to
    // (convex, mesh), even when the broadphase wrappers have the reverse order.
    // Its temporary triangle wrapper supplies the mesh part/face identifiers.
    if ((triangle_a || triangle_b) && getPersistentManifold() &&
        getPersistentManifold()->getBody1() == mesh->getCollisionObject() &&
        mesh->getCollisionObject()->getCollisionShape()->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE) {
      const btVector3 point_a = point + normal * depth;
      btManifoldPoint contact(other->getWorldTransform().invXform(point_a),
                              mesh->getWorldTransform().invXform(point), normal, depth);
      contact.m_positionWorldOnA = point_a;
      contact.m_positionWorldOnB = point;
      adjustMeshContact(contact, mesh, other,
                        triangle_a ? m_partId0 : m_partId1,
                        triangle_a ? m_index0 : m_index1);
      // Correct the normal and both anchors/depth before manifold insertion.
      // Insert the corrected anchors before Bullet chooses its cache entry
      // and friction basis, retaining the usual material/solver processing.
      btManifoldResult::addContactPoint(contact.m_normalWorldOnB,
                                        contact.m_positionWorldOnB, contact.getDistance());
      return;
    }
    btManifoldResult::addContactPoint(normal, point, depth);
  }
};

inline void meshNearCallback(btBroadphasePair& pair, btCollisionDispatcher& dispatcher,
                             const btDispatcherInfo& info) {
  // Keep Bullet's continuous/TOI path unchanged. The vehicle uses discrete
  // fixed steps; only that path substitutes a result local to this dispatcher.
  if (info.m_dispatchFunc != btDispatcherInfo::DISPATCH_DISCRETE) {
    btCollisionDispatcher::defaultNearCallback(pair, dispatcher, info);
    return;
  }
  auto* a = static_cast<btCollisionObject*>(pair.m_pProxy0->m_clientObject);
  auto* b = static_cast<btCollisionObject*>(pair.m_pProxy1->m_clientObject);
  if (!dispatcher.needsCollision(a, b)) return;
  btCollisionObjectWrapper aw(nullptr, a->getCollisionShape(), a, a->getWorldTransform(), -1, -1);
  btCollisionObjectWrapper bw(nullptr, b->getCollisionShape(), b, b->getWorldTransform(), -1, -1);
  if (!pair.m_algorithm)
    pair.m_algorithm = dispatcher.findAlgorithm(&aw, &bw, nullptr, BT_CONTACT_POINT_ALGORITHMS);
  if (pair.m_algorithm) {
    MeshContactResult result(&aw, &bw);
    pair.m_algorithm->processCollision(&aw, &bw, info, &result);
  }
}

}  // namespace gbplanner_smb::detail
