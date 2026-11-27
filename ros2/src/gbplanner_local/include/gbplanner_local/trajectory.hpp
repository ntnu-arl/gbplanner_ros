#pragma once
#include <planner_common/graph_manager.h>

namespace gbplanner2 {
// Pure path helpers from planner_common/trajectory.cpp; see extraction manifest.
class Trajectory {
 public:
  using VectorType=Eigen::Vector3d;
  using PathType=std::vector<VectorType>;
  static bool compareTwoTrajectories(const PathType&,const PathType&,double threshold=1.0,
                                    double step=0.2,bool shorten=true,bool scale=true);
  static double computeDistanceBetweenTwoTrajectories(const PathType&,const PathType&,
                                    double step=0.2,bool shorten=true,bool scale=true);
  static double computeDistanceBetweenTrajectoryAndDirection(const PathType&,double heading,
                                    double step=0.2,bool scale=false);
  static double computeDTWDistance(const PathType&,const PathType&);
  static bool interpolatePath(const PathType&,double,PathType&);
  static void shortenPath(PathType&,double);
  static double getPathLength(const PathType&);
};
class FrontierClusterer {
 public:
  static std::vector<int> cluster(const std::shared_ptr<GraphManager> graph,
      const ShortestPathsReport& report,std::vector<Vertex*>& vertices,
      double distance_threshold=1.0,double principal_min_length=1.0,bool refinement=true);
};
}
