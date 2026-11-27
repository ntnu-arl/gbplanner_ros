// Exercise decision snapshots on actual plans, including discarded candidates.
#include "gbplanner_local/local_planner.hpp"
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool close(double a,double b){return std::abs(a-b)<1e-9;}
bool samePath(const std::vector<StateVec>& a,const std::vector<StateVec>& b){
  if(a.size()!=b.size())return false;
  for(std::size_t i=0;i<a.size();++i)if((a[i]-b[i]).norm()!=0.)return false;
  return true;
}
double pathLength(const std::vector<StateVec>& path){
  double total=0.;
  for(std::size_t i=1;i<path.size();++i)total+=(path[i]-path[i-1]).head<3>().norm();
  return total;
}
// Count every backend entry point, not just raycasts: diagnostics must not
// reproject a ground path or perform new collision queries to draw it.
class CountingMap final:public MapQuery {
 public:
  explicit CountingMap(std::shared_ptr<const MapQuery> map):map_(std::move(map)){}
  double resolution() const override {++queries[0];return map_->resolution();}
  Eigen::Vector3d minimum() const override {++queries[1];return map_->minimum();}
  Eigen::Vector3d maximum() const override {++queries[2];return map_->maximum();}
  VoxelStatus voxel(const Eigen::Vector3d& point) const override {++queries[3];return map_->voxel(point);}
  std::optional<TsdfDistance> tsdfDistance(const Eigen::Vector3d& point) const override {
    ++queries[4];return map_->tsdfDistance(point);
  }
  std::optional<TsdfDistance> tsdfDistanceSimplex(const Eigen::Vector3d& point) const override {
    ++queries[5];return map_->tsdfDistanceSimplex(point);
  }
  VoxelStatus box(const Eigen::Vector3d& point,const Eigen::Vector3d& size) const override {
    ++queries[6];return map_->box(point,size);
  }
  VoxelStatus path(const Eigen::Vector3d& from,const Eigen::Vector3d& to,
                   const Eigen::Vector3d& size) const override {
    ++queries[7];return map_->path(from,to,size);
  }
  bool gainAllowed(const Eigen::Vector3d& point) const override {++queries[8];return map_->gainAllowed(point);}
  void ray(const Eigen::Vector3d& from,const Eigen::Vector3d& to,
           const std::function<bool(const Eigen::Vector3d&,VoxelStatus)>& visit) const override {
    ++queries[9];map_->ray(from,to,visit);
  }
  mutable std::array<std::size_t,10> queries{};
 private:
  std::shared_ptr<const MapQuery> map_;
};
std::shared_ptr<VoxelMap> scene(bool ground=false){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-1.,-1.,-.5),Eigen::Vector3i(70,20,25));
  map->fill(VoxelStatus::Free);
  if(ground)for(int z=0;z<5;++z)for(int y=0;y<20;++y)for(int x=0;x<70;++x)
    map->set(Eigen::Vector3i(x,y,z),VoxelStatus::Occupied);
  return map;
}
Parameters lineParameters(){
  Parameters p;p.local_min={-.5,-.5,-.1};p.local_max={4.5,.5,.1};
  p.robot_size={.1,.1,.1};p.edge_min=.05;p.edge_max=1.;p.overshoot=0.;
  p.max_vertices=5;p.max_attempts=10;p.graph_mode=false;
  p.free_gain=1.;p.path_length_penalty=0.;p.sensor_range=1.;
  p.horizontal_fov=1.;p.horizontal_resolution=1.;p.vertical_fov=.2;p.vertical_resolution=.2;
  p.max_velocity=.2;p.traverse_length_max=1.2;p.traverse_time_max=1000.;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=4.;return p;
}
void samePlan(const LocalPlanner& off,const PlanResult& a,const LocalPlanner& on,const PlanResult& b){
  require(a.success==b.success&&a.reason==b.reason&&a.vertices==b.vertices&&a.edges==b.edges&&
          a.attempts==b.attempts&&a.best_id==b.best_id&&a.gain==b.gain&&
          a.candidate_path_length==b.candidate_path_length&&a.path_length==b.path_length&&
          a.truncated_for_budget==b.truncated_for_budget&&a.truncated_for_hanging==b.truncated_for_hanging&&
          samePath(a.path,b.path),"Path diagnostics changed the selected plan");
  require(off.graph().edge_map_==on.graph().edge_map_,"Path diagnostics changed graph topology or weights");
  for(const auto& item:off.graph().vertices_map_){
    const auto* v=item.second;const auto* other=on.graph().vertices_map_.at(item.first);
    require((v->state-other->state).norm()==0.&&v->is_leaf_vertex==other->is_leaf_vertex&&
            v->vol_gain.gain==other->vol_gain.gain&&
            v->vol_gain.accumulative_gain==other->vol_gain.accumulative_gain,
            "Path diagnostics changed a graph vertex or its gain");
  }
  require(off.diagnostics().shortest_vertices.empty()&&off.diagnostics().best_path.empty()&&
          off.diagnostics().best_id==-1&&off.diagnostics().best_gain==0.,
          "Disabled path diagnostics retained a snapshot");
}
void checkTree(const LocalPlanner& planner){
  const auto& tree=planner.diagnostics().shortest_vertices;
  const auto& graph=planner.graph();
  require(tree.size()==graph.vertices_map_.size(),"Reachable vertices are missing from the tree snapshot");
  std::set<int> ids;
  for(const auto& point:tree){
    require(ids.insert(point.id).second,"The shortest-path tree contains duplicate vertices");
    const auto* vertex=graph.vertices_map_.at(point.id);
    require((point.state-vertex->state).norm()==0.&&point.leaf==vertex->is_leaf_vertex,
            "Tree snapshot differs from the scored vertex");
    // This deterministic graph is a four-edge chain. Every predecessor and
    // leaf is known independently of the diagnostic implementation.
    require(point.parent==(point.id?point.id-1:0)&&point.leaf==(point.id==4),
            "The recorded predecessor or leaf is not the actual shortest-path tree");
  }
}
}

int main(){
  try{
    PlanningDiagnosticsOptions options;options.paths=true;
    const StateVec root(0.,0.,.6,0.);auto p=lineParameters();
    auto aerial_scene=scene();
    auto aerial_off_map=std::make_shared<CountingMap>(aerial_scene);
    auto aerial_on_map=std::make_shared<CountingMap>(aerial_scene);
    LocalPlanner aerial_off(aerial_off_map,p),aerial_on(aerial_on_map,p,options);
    const auto aerial_plain=aerial_off.plan(root),aerial=aerial_on.plan(root);
    samePlan(aerial_off,aerial_plain,aerial_on,aerial);checkTree(aerial_on);
    require(aerial_off_map->queries==aerial_on_map->queries,"Aerial diagnostics issued additional map queries");
    const auto& aerial_diagnostics=aerial_on.diagnostics();
    require(aerial.success&&aerial.truncated_for_budget&&aerial.path.size()==3&&
            aerial_diagnostics.best_path.size()==5&&aerial_diagnostics.best_id==aerial.best_id&&
            aerial_diagnostics.best_gain==aerial.gain&&close(pathLength(aerial_diagnostics.best_path),4.)&&
            close(aerial.path_length,2.),"Best candidate was replaced by its budget-truncated execution path");
    for(const auto& state:aerial_diagnostics.best_path)
      require((state-aerial_on.graph().vertices_map_.at(static_cast<int>(state.x()))->state).norm()==0.,
              "Aerial candidate contains a fabricated point");

    auto ground_parameters=p;ground_parameters.ground_robot=true;
    ground_parameters.ground.max_ground_height=.6;ground_parameters.ground.robot_height=.4;
    ground_parameters.traverse_length_max=.25;
    auto ground_scene=scene(true);
    auto ground_off_map=std::make_shared<CountingMap>(ground_scene);
    auto ground_on_map=std::make_shared<CountingMap>(ground_scene);
    LocalPlanner ground_off(ground_off_map,ground_parameters),ground_on(ground_on_map,ground_parameters,options);
    const auto ground_plain=ground_off.plan(root),ground=ground_on.plan(root);
    samePlan(ground_off,ground_plain,ground_on,ground);checkTree(ground_on);
    require(ground_off_map->queries==ground_on_map->queries,"Ground diagnostics re-queried terrain or collision geometry");
    auto unlimited_parameters=ground_parameters;unlimited_parameters.traverse_length_max=100.;
    const auto unlimited=LocalPlanner(ground_scene,unlimited_parameters).plan(root);
    const auto& ground_diagnostics=ground_on.diagnostics();
    require(ground.success&&ground.truncated_for_budget&&unlimited.success&&
            ground_diagnostics.best_path.size()>ground_diagnostics.shortest_vertices.size()&&
            ground_diagnostics.best_path.size()>ground.path.size()&&
            samePath(ground_diagnostics.best_path,unlimited.path)&&
            close(pathLength(ground_diagnostics.best_path),ground.candidate_path_length),
            "Ground candidate lost its complete cached projection polyline");

    auto short_parameters=p;short_parameters.max_vertices=2;short_parameters.axis_sampling[0].constant=.3;
    auto short_off_map=std::make_shared<CountingMap>(aerial_scene);
    auto short_on_map=std::make_shared<CountingMap>(aerial_scene);
    LocalPlanner short_off(short_off_map,short_parameters),short_on(short_on_map,short_parameters,options);
    const auto short_plain=short_off.plan(root),short_result=short_on.plan(root);
    samePlan(short_off,short_plain,short_on,short_result);
    require(short_off_map->queries==short_on_map->queries,"Capturing a short candidate issued map queries");
    require(short_result.reason=="best_path_too_short"&&!short_result.success&&short_result.path.empty()&&
            short_on.diagnostics().best_path.size()==2&&short_on.diagnostics().best_id==short_result.best_id&&
            short_on.diagnostics().best_gain==short_result.gain,
            "The scored short candidate was discarded before its diagnostic snapshot");

    auto no_gain_parameters=p;no_gain_parameters.free_gain=0.;no_gain_parameters.unknown_gain=0.;
    LocalPlanner no_gain(aerial_scene,no_gain_parameters,options);const auto empty=no_gain.plan(root);
    require(empty.reason=="no_gain"&&!no_gain.diagnostics().shortest_vertices.empty()&&
            no_gain.diagnostics().best_path.empty()&&no_gain.diagnostics().best_id==-1&&
            no_gain.diagnostics().best_gain==0.,"No-gain graph fabricated a best candidate");
    StateVec invalid=root;invalid.x()=std::numeric_limits<double>::quiet_NaN();
    const auto failed=short_on.plan(invalid);
    require(failed.reason=="invalid_root"&&short_on.diagnostics().shortest_vertices.empty()&&
            short_on.diagnostics().best_path.empty()&&short_on.diagnostics().best_id==-1&&
            short_on.diagnostics().best_gain==0.,"A new planning call retained stale path diagnostics");

    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"tree_vertices\":"
      <<aerial_diagnostics.shortest_vertices.size()<<",\"aerial_candidate_points\":"
      <<aerial_diagnostics.best_path.size()<<",\"aerial_execution_points\":"<<aerial.path.size()
      <<",\"ground_candidate_points\":"<<ground_diagnostics.best_path.size()
      <<",\"ground_execution_points\":"<<ground.path.size()
      <<",\"short_candidate_retained\":true,\"no_gain_has_no_best\":true"
      <<",\"disabled_is_empty\":true,\"plan_and_graph_unchanged\":true,\"additional_map_queries\":0}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
