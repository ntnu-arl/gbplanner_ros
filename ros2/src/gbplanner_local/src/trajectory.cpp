// Copyright (c) 2020 UNR Autonomous Robots Lab. BSD-3-Clause.
// Extracted/adapted by tools/port_global_helpers.py; see global_helpers_provenance.json.
#include "gbplanner_local/trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace gbplanner2 {
bool Trajectory::compareTwoTrajectories(const PathType& path_1,
                                        const PathType& path_2,
                                        double dist_threshold,
                                        double discrete_length,
                                        bool shorten_to_same_length,
                                        bool scale_with_length) {
  double dist = computeDistanceBetweenTwoTrajectories(
      path_1, path_2, discrete_length, shorten_to_same_length,
      scale_with_length);
  if (dist > dist_threshold)
    return false;
  else
    return true;
}

double Trajectory::computeDistanceBetweenTwoTrajectories(
    const PathType& path_1, const PathType& path_2, double discrete_length,
    bool shorten_to_same_length, bool scale_with_length) {
  // Currently, only scale over length --> spatial info only.
  // @TODO: Should scale over time as well to amplify temporal difference.
  double dist_ret = std::numeric_limits<double>::infinity();

  if ((path_1.size() <= 0) || (path_2.size() <= 0) || (discrete_length <= 0))
    return dist_ret;

  PathType path_1_intp;
  if (!interpolatePath(path_1, discrete_length, path_1_intp)) return dist_ret;

  PathType path_2_intp;
  if (!interpolatePath(path_2, discrete_length, path_2_intp)) return dist_ret;

  // Cut trajectories to the same length.
  double path_1_len = getPathLength(path_1_intp);
  double path_2_len = getPathLength(path_2_intp);
  double common_length = std::min(path_1_len, path_2_len);
  if (shorten_to_same_length) {
    if (path_1_len > path_2_len) {
      // Cut path_1.
      common_length = path_2_len;
      shortenPath(path_1_intp, common_length);
    } else if (path_1_len < path_2_len) {
      // Cut path_2.
      common_length = path_1_len;
      shortenPath(path_2_intp, common_length);
    }
  }

  // DTW distance.
  dist_ret = computeDTWDistance(path_1_intp, path_2_intp);
  if (scale_with_length) {
    // Degenerate constant paths have zero common length; avoid upstream 0/0.
    if (common_length <= 1e-12)
      return dist_ret <= 1e-12 ? 0.0 : std::numeric_limits<double>::infinity();
    dist_ret /= (common_length * common_length);
  }
  return dist_ret;
}

double Trajectory::computeDistanceBetweenTrajectoryAndDirection(
    const PathType& path,double heading,double discrete_length,bool scale_with_length) {
  if(path.empty()||!std::isfinite(heading)||!std::isfinite(discrete_length)||discrete_length<=0)
    return std::numeric_limits<double>::max();
  PathType path_intp;
  if(!interpolatePath(path,discrete_length,path_intp))return std::numeric_limits<double>::max();
  const double path_length=getPathLength(path_intp);
  if(path_length==0)return std::numeric_limits<double>::max();
  const Eigen::Vector3d p0=path.front();
  const Eigen::Vector3d uvector=Eigen::AngleAxisd(heading,Eigen::Vector3d::UnitZ())*
      Eigen::Vector3d(1,0,0)*discrete_length;
  PathType path_ref;
  const double steps=path_length/discrete_length;
  const double nearest=std::round(steps);
  // Translating a path can make a sampled 0.2 m segment measure just below
  // 0.2. Snap only roundoff near positive integers before upstream's floor;
  // a genuinely sub-step path must still produce an empty reference.
  const double tolerance=8.*std::numeric_limits<double>::epsilon()*std::max(1.,steps);
  const int n=static_cast<int>(nearest>=1.&&std::abs(steps-nearest)<=tolerance?nearest:steps);
  for(int i=0;i<n;++i){
    // Preserve upstream trajectory.cpp:93 exactly: all n reference points
    // are the same forward endpoint (*n), not a line sampled using *i.
    path_ref.push_back(p0+uvector*n);
  }
  double distance=computeDTWDistance(path_intp,path_ref);
  if(scale_with_length)distance/=(path_length*path_length);
  return distance;
}

void Trajectory::shortenPath(PathType& path, double max_len) {
  double total_len = 0;
  int path_size = path.size();
  for (int i = 0; i < (path_size - 1); ++i) {
    VectorType vec = path[i + 1] - path[i];
    double segment_len = vec.norm();
    total_len += segment_len;
    if (total_len > max_len) {
      // i+1 to make sure the path has at least 1 segment.
      path.erase(path.begin() + i + 1, path.end());
      break;
    }
  }
}

double Trajectory::getPathLength(const PathType& path) {
  double total_len = 0;
  int path_size = path.size();
  for (int i = 0; i < (path_size - 1); ++i) {
    VectorType vec = path[i + 1] - path[i];
    total_len += vec.norm();
  }
  return total_len;
}

bool Trajectory::interpolatePath(const PathType& path, double discrete_length,
                                 PathType& path_intp) {
  path_intp.clear();
  if (discrete_length <= 0) return false;

  int path_size = path.size();
  if (path_size == 0) {
    return false;
  } else if (path_size == 1) {
    path_intp.push_back(path[0]);
    return true;
  }

  for (int i = 0; i < (path_size - 1); ++i) {
    // Interpolate along the segment.
    VectorType vec = path[i + 1] - path[i];
    double segment_len = vec.norm();
    if (std::abs(segment_len) < 0.01) {
      // Duplicated nodes. Add one only.
      path_intp.push_back(path[i]);
    } else {
      int n = (int)(segment_len / discrete_length);
      VectorType uvec = vec / segment_len * discrete_length;
      for (int j = 0; j <= n; ++j) {
        path_intp.push_back(path[i] + j * uvec);
      }
    }
  }
  return true;
}

double Trajectory::computeDTWDistance(const PathType& pa, const PathType& pb) {
  int n = pa.size();
  int m = pb.size();

  std::vector<std::vector<double>> dist;
  dist.resize(n + 1);
  for (auto& v : dist) {
    v.resize(m + 1);
  }

  for (int i = 1; i <= n; ++i) {
    dist[i][0] = std::numeric_limits<double>::infinity();
  }
  for (int j = 1; j <= m; ++j) {
    dist[0][j] = std::numeric_limits<double>::infinity();
  }

  dist[0][0] = 0;
  for (int i = 1; i <= n; ++i) {
    for (int j = 1; j <= m; ++j) {
      double d = (pa[i - 1] - pb[j - 1]).norm();
      dist[i][j] = d + std::min(std::min(dist[i - 1][j - 1], dist[i - 1][j]),
                                dist[i][j - 1]);
    }
  }

  return dist[n][m];
}
std::vector<int> FrontierClusterer::cluster(
    const std::shared_ptr<GraphManager> graph_manager,
    const ShortestPathsReport& graph_rep, std::vector<Vertex*>& vertices,
    double dist_threshold, double principle_path_min_length,
    bool refinement_enable) {
  // Asumme long paths are principle paths.
  // Go over one by one from the longest ones.
  // Group into clusters based on the normalized DTW distance metric.
  // Refine by choosing closest & valid principle path.

  // Sort into descending order.
  std::sort(vertices.begin(), vertices.end(),
            [&graph_manager, &graph_rep](const Vertex* a, const Vertex* b) {
              return graph_manager->getShortestDistance(a->id, graph_rep) >
                     graph_manager->getShortestDistance(b->id, graph_rep);
            });

  std::vector<std::vector<Eigen::Vector3d>> cluster_paths;
  std::vector<int> cluster_ids;
  for (std::size_t i = 0; i < vertices.size(); ++i) {
    std::vector<Eigen::Vector3d> path_cur;
    graph_manager->getShortestPath(vertices[i]->id, graph_rep, true, path_cur);
    bool found_a_neigbor = false;
    for (std::size_t j = 0; j < cluster_paths.size(); ++j) {
      if (Trajectory::compareTwoTrajectories(path_cur, cluster_paths[j],
                                             dist_threshold)) {
        vertices[i]->cluster_id = cluster_ids[j];
        found_a_neigbor = true;
        break;
      }
    }
    if (!found_a_neigbor) {
      // Can not find any neigbor, set this as a new principle path.
      cluster_paths.emplace_back(path_cur);
      cluster_ids.push_back(vertices[i]->id);
      vertices[i]->cluster_id = vertices[i]->id;
    }
  }



  // Refinement step, remove short path and choose closest cluster.
  if (refinement_enable) {
    // Clean out noisy paths by removing short path.
    std::vector<std::vector<Eigen::Vector3d>> cluster_paths_refine;
    std::vector<int> cluster_ids_refine;
    for (std::size_t j = 0; j < cluster_paths.size(); ++j) {
      double path_len = Trajectory::getPathLength(cluster_paths[j]);
      if (path_len >= principle_path_min_length) {
        cluster_paths_refine.push_back(cluster_paths[j]);
        cluster_ids_refine.push_back(cluster_ids[j]);
      }
    }
    // Recheck and choose closest one.
    for (std::size_t i = 0; i < vertices.size(); ++i) {
      std::vector<Eigen::Vector3d> path_cur;
      graph_manager->getShortestPath(vertices[i]->id, graph_rep, true,
                                     path_cur);
      double dist_min = std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < cluster_paths_refine.size(); ++j) {
        double dist_score = Trajectory::computeDistanceBetweenTwoTrajectories(
            path_cur, cluster_paths_refine[j]);
        if (dist_min > dist_score) {
          dist_min = dist_score;
          vertices[i]->cluster_id = cluster_ids_refine[j];
        }
      }
    }
  
    return cluster_ids_refine;
  } else {
    return cluster_ids;
  }
}
}
