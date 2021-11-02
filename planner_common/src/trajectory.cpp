#include "planner_common/trajectory.h"

bool Trajectory::compareTwoTrajectories(const TrajectoryType& traj_1,
                                        const TrajectoryType& traj_2,
                                        double dist_threshold,
                                        double discrete_length,
                                        bool shorten_to_same_length,
                                        bool scale_with_length) {
  if ((traj_1.size() <= 0) || (traj_2.size() <= 0) || (discrete_length <= 0))
    return false;

  PathType path_1;
  extractPathFromTrajectory(traj_1, path_1);
  PathType path_2;
  extractPathFromTrajectory(traj_2, path_2);
  return (compareTwoTrajectories(path_1, path_2, dist_threshold,
                                 discrete_length, shorten_to_same_length,
                                 scale_with_length));
}

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
  if (scale_with_length) dist_ret /= (common_length * common_length);
  return dist_ret;
}

double Trajectory::computeDistanceBetweenTrajectoryAndDirection(
    const PathType& path, double heading, double discrete_length,
    bool scale_with_length) {
  if ((path.size() <= 0) || (discrete_length <= 0))
    return std::numeric_limits<double>::max();

  PathType path_intp;
  if (!interpolatePath(path, discrete_length, path_intp))
    return std::numeric_limits<double>::max();
  double path_length = getPathLength(path_intp);
  if (path_length == 0) return std::numeric_limits<double>::max();

  Eigen::Vector3d p0(path[0][0], path[0][1], path[0][2]);
  Eigen::Vector3d uvector =
      Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()) *
      Eigen::Vector3d(1, 0, 0) * discrete_length;
  std::vector<Eigen::Vector3d> path_ref;
  int n = (int)(path_length / discrete_length);
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d ei = p0 + uvector * n;
    path_ref.push_back(ei);
  }

  // DTW distance.
  double dtw_dist = computeDTWDistance(path_intp, path_ref);
  // Scale with length or length square
  if (scale_with_length) dtw_dist /= (path_length * path_length);

  return dtw_dist;
}

void Trajectory::extractPathFromTrajectory(const TrajectoryType& traj,
                                           PathType& path) {
  path.clear();
  for (auto t = traj.begin(); t != traj.end(); ++t) {
    path.push_back(
        VectorType((*t).position.x, (*t).position.y, (*t).position.z));
  }
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

double Trajectory::getPathLength(const TrajectoryType& traj) {
  double total_len = 0;
  int traj_size = traj.size();
  for (int i = 0; i < (traj_size - 1); ++i) {
    VectorType vec(traj[i + 1].position.x - traj[i].position.x,
                   traj[i + 1].position.y - traj[i].position.y,
                   traj[i + 1].position.z - traj[i].position.z);
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

bool Trajectory::interpolatePath(const TrajectoryType& traj,
                                 double discrete_length,
                                 TrajectoryType& traj_intp) {
  traj_intp.clear();
  // Invalid discretization distance
  if (discrete_length <= 0) {
    return false;
  }

  int traj_size = traj.size();
  if (traj_size == 0) {
    return false;
  } else if (traj_size == 1) {
    traj_intp.push_back(traj[0]);
    return true;
  }

  for (int i = 0; i < (traj_size - 1); ++i) {
    VectorType v1(traj[i].position.x, traj[i].position.y, traj[i].position.z);
    VectorType v2(traj[i + 1].position.x, traj[i + 1].position.y,
                  traj[i + 1].position.z);
    VectorType vec = v2 - v1;
    double segment_len = vec.norm();
    if (segment_len < 0.01) {
      // Too close, only add one
      traj_intp.push_back(traj[i]);
    } else {
      int n = (int)(segment_len / discrete_length);
      if (std::abs((double)n - (segment_len / discrete_length)) < 0.01) {
        // Avoid adding traj[i+1] twice (once in this and once in next
        // iteration) if segment length is integral multiple of discrete length
        --n;
      }
      VectorType uvec = vec / segment_len * discrete_length;
      for (int j = 0; j <= n; ++j) {
        VectorType int_vec = v1 + j * uvec;
        WayPointType wp;
        wp.position.x = int_vec(0);
        wp.position.y = int_vec(1);
        wp.position.z = int_vec(2);
        wp.orientation.x = traj[i + 1].orientation.x;
        wp.orientation.y = traj[i + 1].orientation.y;
        wp.orientation.z = traj[i + 1].orientation.z;
        wp.orientation.w = traj[i + 1].orientation.w;
        traj_intp.push_back(wp);
      }
    }
  }
  return true;
}

double Trajectory::estimateDirectionFromPath(const PathType& path) {
  // First approach: sum up from each edge
  // Modified approach: sum up from first vertex towards each vertex
  const double kAlpha = 0.9;  // depends more on close vertices.
  if (path.size() <= 1) return 0.0;
  double yaw = 0;
  if (path.size() >= 2) {
    Eigen::Vector3d cur_dir(path[1][0] - path[0][0], path[1][1] - path[0][1],
                            path[1][2] - path[0][2]);
    yaw = atan2(cur_dir[1], cur_dir[0]);
  }

  for (int i = 2; i < path.size(); ++i) {
    // 1
    // Eigen::Vector3d cur_dir(path[i][0] - path[i - 1][0],
    //                         path[i][1] - path[i - 1][1],
    //                         path[i][2] - path[i - 1][2]);
    // 2
    Eigen::Vector3d cur_dir(path[i][0] - path[0][0], path[i][1] - path[0][1],
                            path[i][2] - path[0][2]);
    double yaw_tmp = atan2(cur_dir[1], cur_dir[0]);
    double dyaw = yaw_tmp - yaw;
    truncateYaw(dyaw);
    yaw = yaw + (1 - kAlpha) * dyaw;
    truncateYaw(yaw);
  }
  return yaw;
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

void Trajectory::TestDTW() {
  {
    std::vector<Eigen::Vector3d> pa;
    pa.resize(20);
    for (int i = 0; i < 20; ++i) {
      pa[i] << 0, 0, 0;
    }
    std::vector<Eigen::Vector3d> pb;
    pb.resize(20);
    for (int i = 0; i < 20; ++i) {
      pb[i] << 0, 0, 0;
    }
    std::cout << "T1: " << compareTwoTrajectories(pa, pb) << std::endl;
  }
  {
    std::vector<Eigen::Vector3d> pa;
    pa.resize(20);
    for (int i = 0; i < 20; ++i) {
      pa[i] << 0, 0, 0;
    }
    std::vector<Eigen::Vector3d> pb;
    pb.resize(20);
    for (int i = 0; i < 20; ++i) {
      pb[i] << 10, 10, 10;
    }
    std::cout << "T2: " << compareTwoTrajectories(pa, pb) << std::endl;
  }
  {
    std::vector<Eigen::Vector3d> pa;
    pa.resize(20);
    for (int i = 0; i < 20; ++i) {
      pa[i] << i, 0, 0;
    }
    std::vector<Eigen::Vector3d> pb;
    pb.resize(20);
    for (int i = 0; i < 20; ++i) {
      pb[i] << 1 + i, 0, 0;
    }
    std::cout << "T3: " << compareTwoTrajectories(pa, pb) << std::endl;
  }
  {
    std::vector<Eigen::Vector3d> pa;
    pa.resize(10);
    for (int i = 0; i < 10; ++i) {
      pa[i] << i, 0, 0;
    }
    std::vector<Eigen::Vector3d> pb;
    pb.resize(20);
    for (int i = 0; i < 20; ++i) {
      pb[i] << 1 + i, 0, 0;
    }
    std::cout << "T4: " << compareTwoTrajectories(pa, pb) << std::endl;
  }
  {
    std::vector<Eigen::Vector3d> pa;
    pa.resize(4);
    pa.push_back(Eigen::Vector3d(0, 0, 0));
    pa.push_back(Eigen::Vector3d(1, 0, 0));
    pa.push_back(Eigen::Vector3d(2, 0, 0));
    pa.push_back(Eigen::Vector3d(3, 0, 0));

    std::vector<Eigen::Vector3d> pb;
    pb.resize(4);
    pb.push_back(Eigen::Vector3d(0, 0, 0));
    pb.push_back(Eigen::Vector3d(0.5, 0, 0));
    pb.push_back(Eigen::Vector3d(1.5, 0, 0));
    pb.push_back(Eigen::Vector3d(3.0, 0, 0));

    std::cout << "T4: " << compareTwoTrajectories(pa, pb) << std::endl;
  }
}