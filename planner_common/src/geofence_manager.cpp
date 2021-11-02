#include "planner_common/geofence_manager.h"

#include <time.h>

#include <iostream>


Polygon2d::Polygon2d(std::vector<Eigen::Vector2d> points) {
  for (auto p = points.begin(); p != points.end(); ++p) {
    bg::append(polygon_.outer(), Point2dType(p->x(), p->y()));
  }
  // Close the polygon
  bg::append(polygon_.outer(), Point2dType(points[0].x(), points[0].y()));
  // To enforce the order into clock-wise.
  bg::correct(polygon_);
}

Polygon2d::Polygon2d(const geometry_msgs::Polygon& poly_msg) {
  for (auto p = poly_msg.points.begin(); p != poly_msg.points.end(); ++p) {
    bg::append(polygon_.outer(), Point2dType(p->x, p->y));
  }
  // Close the polygon
  bg::append(polygon_.outer(),
             Point2dType(poly_msg.points[0].x, poly_msg.points[0].y));
  // To enforce the order into clock-wise.
  bg::correct(polygon_);
}

Polygon2d::Polygon2d(Polygon2dType& poly2d) { bg::assign(polygon_, poly2d); }

void Polygon2d::getCenter(Eigen::Vector2d& center) {
  Point2dType p2d;
  bg::centroid(polygon_, p2d);
  convertPointToEigen(p2d, center);
}

void Polygon2d::getVertices(std::vector<Eigen::Vector2d>& vertices) {
  std::vector<Point2dType> const& points = polygon_.outer();
  for (auto it = points.begin(); it != points.end(); ++it) {
    double x = bg::get<0>(*it);
    double y = bg::get<1>(*it);
    vertices.push_back(Eigen::Vector2d(x, y));
  }
}

bool Polygon2d::isInside(const Eigen::Vector2d& point) const {
  return bg::within(Point2dType(point.x(), point.y()), polygon_);
}

bool Polygon2d::doIntersectWithRectangle(const Eigen::Vector2d& center,
                                         const Eigen::Vector2d& size) const {
  Polygon2dType rect;
  bg::append(rect.outer(),
             Point2dType(center.x() + size.x() / 2, center.y() + size.y() / 2));
  bg::append(rect.outer(),
             Point2dType(center.x() + size.x() / 2, center.y() - size.y() / 2));
  bg::append(rect.outer(),
             Point2dType(center.x() - size.x() / 2, center.y() - size.y() / 2));
  bg::append(rect.outer(),
             Point2dType(center.x() - size.x() / 2, center.y() + size.y() / 2));
  bg::append(rect.outer(),
             Point2dType(center.x() + size.x() / 2, center.y() + size.y() / 2));

  return bg::intersects(polygon_, rect);
}

bool Polygon2d::doIntersect(Polygon2dType poly) const {
  return bg::intersects(polygon_, poly);
}

void Polygon2d::convertPointToEigen(Point2dType& p, Eigen::Vector2d& e) {
  e << p.x(), p.y();
}
void Polygon2d::convertEigenToPoint(Eigen::Vector2d& e, Point2dType& p) {
  bg::assign_values(p, e.x(), e.y());
}

void GeofenceManager::addGeofenceArea(const geometry_msgs::Polygon poly_msg,
                                      bool merge) {
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Added new geofence area.");
  Polygon2d poly2d(poly_msg);
  addGeofenceArea(poly2d, merge);
}

void GeofenceManager::addGeofenceArea(Polygon2d& poly, bool merge) {
  geofence_list_.push_back(GeofenceArea(generateNewID(), poly));
  if (merge) {
    mergeGeofenceAreas();
  }
  ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Number of geofence areas: %d", (int)geofence_list_.size());
}

void GeofenceManager::removeGeofenceAreaWithID(int id) {
  for (auto it = geofence_list_.begin(); it != geofence_list_.end(); ++it) {
    geofence_list_.erase(it);
    break;
  }
}

void GeofenceManager::clear() {
  geofence_list_.clear();
  geofence_id_num_ = 0;
}

void GeofenceManager::mergeGeofenceAreas() {
  // Union if intersect
  bool cont = true;
  while (cont) {
    // brute-force
    cont = false;
    for (int i = 0; i < geofence_list_.size(); ++i) {
      bool redo = false;
      for (int j = i + 1; j < geofence_list_.size(); ++j) {
        if ((geofence_list_[i].polygon.doIntersect(
                geofence_list_[j].polygon.polygon_))) {
          std::vector<Polygon2d::Polygon2dType> union_poly;
          bg::union_(geofence_list_[i].polygon.polygon_,
                     geofence_list_[j].polygon.polygon_, union_poly);
          if (union_poly.size() == 1) {
            Polygon2d poly2d(union_poly[0]);
            geofence_list_.push_back(GeofenceArea(generateNewID(), poly2d));
            // remove the old two
            removeGeofenceAreaWithID(geofence_list_[i].id);
            removeGeofenceAreaWithID(geofence_list_[j].id);
            // Restart the merging process again.
            cont = true;
            redo = true;
            break;
          } else {
            // Create more than one polygon.
            ROS_WARN_COND(global_verbosity >= Verbosity::WARN, "Polygon with inner ring or they don't intersect.");
          }
        }
      }
      if (redo) break;
    }
  }
}

void GeofenceManager::addGeofenceAreas(std::vector<Polygon2d>& poly_list,
                                       bool merge) {
  for (auto& p : poly_list) {
    addGeofenceArea(p, merge);
  }
}

void GeofenceManager::getLocalGeofenceAreas(
    Eigen::Vector2d& current_position, double radius,
    std::vector<GeofenceArea>& geo_list) {}

void GeofenceManager::getAllGeofenceAreas(std::vector<GeofenceArea>& geo_list) {
  for (auto& g : geofence_list_) {
    GeofenceArea ga = g;  // make a copy
    geo_list.push_back(g);
  }
}

GeofenceManager::CoordinateStatus GeofenceManager::getCoordinateStatus(
    const Eigen::Vector2d& position) const {
  if (geofence_list_.empty()) return CoordinateStatus::kOK;

  CoordinateStatus status = CoordinateStatus::kOK;
  for (auto& p : geofence_list_) {
    if (p.polygon.isInside(position)) {
      status = CoordinateStatus::kViolated;
      break;
    }
  }
  return status;
}

GeofenceManager::CoordinateStatus GeofenceManager::getBoxStatus(
    const Eigen::Vector2d& center, const Eigen::Vector2d& size) const {
  if (geofence_list_.empty()) return CoordinateStatus::kOK;

  // Compare a rectange and a convex shape.
  CoordinateStatus status = CoordinateStatus::kOK;
  for (auto& p : geofence_list_) {
    if (p.polygon.doIntersectWithRectangle(center, size)) {
      status = CoordinateStatus::kViolated;
      break;
    }
  }
  return status;
}

GeofenceManager::CoordinateStatus GeofenceManager::getPathStatus(
    const Eigen::Vector2d& start, const Eigen::Vector2d& end,
    const Eigen::Vector2d& box_size) const {
  if (geofence_list_.empty()) return CoordinateStatus::kOK;

  // Form a polygon along the path.
  Polygon2d::Polygon2dType path_polygon;

  Eigen::Vector2d p1(start.x() + box_size.x() / 2.0,
                     start.y() + box_size.y() / 2.0);
  Eigen::Vector2d p2(start.x() + box_size.x() / 2.0,
                     start.y() - box_size.y() / 2.0);
  Eigen::Vector2d p3(start.x() - box_size.x() / 2.0,
                     start.y() - box_size.y() / 2.0);
  Eigen::Vector2d p4(start.x() - box_size.x() / 2.0,
                     start.y() + box_size.y() / 2.0);

  Eigen::Vector2d p_dir = end - start;
  Eigen::Vector2d p1_end;
  p1_end = p1 + p_dir;
  Eigen::Vector2d p2_end;
  p2_end = p2 + p_dir;
  Eigen::Vector2d p3_end;
  p3_end = p3 + p_dir;
  Eigen::Vector2d p4_end;
  p4_end = p4 + p_dir;

  // Check the orientation of the line to form the convex hull.
  p_dir.normalize();
  double orientation = std::atan2(p_dir.y(), p_dir.x());
  if ((orientation >= M_PI_2) && (orientation <= M_PI)) {
    // Case 1
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p3.x(), p3.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p3_end.x(), p3_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p4_end.x(), p4_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p1_end.x(), p1_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
  } else if ((orientation >= 0) && (orientation < M_PI_2)) {
    // Case 2
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p3.x(), p3.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p4.x(), p4.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p4_end.x(), p4_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p1_end.x(), p1_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p2_end.x(), p2_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
  } else if ((orientation >= -M_PI_2) && (orientation < 0)) {
    // Case 3
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p1_end.x(), p1_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p2_end.x(), p2_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p3_end.x(), p3_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p3.x(), p3.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p4.x(), p4.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
  } else if ((orientation >= -M_PI) && (orientation < -M_PI_2)) {
    // Case 4
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p2.x(), p2.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p2_end.x(), p2_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p3_end.x(), p3_end.y()));
    bg::append(path_polygon.outer(),
               Polygon2d::Point2dType(p4_end.x(), p4_end.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p4.x(), p4.y()));
    bg::append(path_polygon.outer(), Polygon2d::Point2dType(p1.x(), p1.y()));
  }

  // Compare a rectange and a convex shape.
  CoordinateStatus status = CoordinateStatus::kOK;
  for (auto& p : geofence_list_) {
    if (p.polygon.doIntersect(path_polygon)) {
      status = CoordinateStatus::kViolated;
      break;
    }
  }
  return status;
}

bool GeofenceManager::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;
  std::string parse_str;

  std::vector<std::string> geofence_init_list;
  param_name = ns + "/AreaList";
  if (!ros::param::get(param_name, geofence_init_list)) {
    return true;
  } else {
    for (auto& s : geofence_init_list) {
      Eigen::Vector3d g_center;
      Eigen::Vector3d g_size;

      param_val.clear();
      param_name = ns + "/" + s + "/center";
      if ((!ros::param::get(param_name, param_val)) ||
          (param_val.size() != 3)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
      g_center << param_val[0], param_val[1], param_val[2];

      param_val.clear();
      param_name = ns + "/" + s + "/size";
      if ((!ros::param::get(param_name, param_val)) ||
          (param_val.size() != 3)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
      g_size << param_val[0], param_val[1], param_val[2];

      std::vector<Eigen::Vector2d> points;
      points.push_back(Eigen::Vector2d(g_center.x() + g_size.x() / 2,
                                       g_center.y() + g_size.y() / 2));
      points.push_back(Eigen::Vector2d(g_center.x() + g_size.x() / 2,
                                       g_center.y() - g_size.y() / 2));
      points.push_back(Eigen::Vector2d(g_center.x() - g_size.x() / 2,
                                       g_center.y() - g_size.y() / 2));
      points.push_back(Eigen::Vector2d(g_center.x() - g_size.x() / 2,
                                       g_center.y() + g_size.y() / 2));
      Polygon2d poly(points);
      addGeofenceArea(poly, true);
    }
  }

  ROSPARAM_INFO("Done.");
  return true;
}
