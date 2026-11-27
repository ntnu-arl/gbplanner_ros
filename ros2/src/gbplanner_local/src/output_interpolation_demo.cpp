// Final planner output spacing, independent of geometric path enhancement.
#include "gbplanner_local/local_planner.hpp"
#include "gbplanner_local/path_refinement.hpp"
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
bool exact(const StateVec& a,const StateVec& b){return (a.array()==b.array()).all();}
double length(const std::vector<StateVec>& path){
  double total=0.;
  for(std::size_t i=1;i<path.size();++i)total+=(path[i]-path[i-1]).head<3>().norm();
  return total;
}

void preservesPolyline(const std::vector<StateVec>& input,
                       const std::vector<StateVec>& output,double spacing){
  require(!input.empty()&&!output.empty()&&exact(input.front(),output.front())&&
          exact(input.back(),output.back()),"Interpolation changed an exact endpoint state");
  std::size_t segment=0;
  for(std::size_t i=1;i<output.size();++i){
    require(segment+1<input.size(),"Interpolation produced points after the endpoint");
    const Eigen::Vector3d step=(output[i]-output[i-1]).head<3>();
    require(step.norm()<=spacing+1e-12,"An output segment exceeds the requested spacing");
    if(exact(output[i],input[segment+1])){++segment;continue;}
    const Eigen::Vector3d original=(input[segment+1]-input[segment]).head<3>();
    const Eigen::Vector3d relative=(output[i]-input[segment]).head<3>();
    require(original.squaredNorm()>0.,"Inserted a spatial point in a pure-yaw segment");
    const double along=relative.dot(original)/original.squaredNorm();
    require(along>0.&&along<1.&&
            (relative-along*original).norm()<1e-12,
            "Interpolation skipped a terrain knot or left its original segment");
    require(output[i][3]==input[segment+1][3],"Inserted point does not use successor yaw");
  }
  require(segment+1==input.size(),"An existing waypoint was omitted or changed");
  require(std::abs(length(input)-length(output))<1e-10,
          "Interpolation changed the checked polyline length");
}

std::shared_ptr<VoxelMap> scene(){
  auto map=std::make_shared<VoxelMap>(.1,Eigen::Vector3d(-1.,-1.,-.5),Eigen::Vector3i(70,20,25));
  map->fill(VoxelStatus::Free);return map;
}
Parameters lineParameters(){
  Parameters p;p.local_min={-.5,-.5,-.1};p.local_max={4.5,.5,.1};
  p.robot_size={.1,.1,.1};p.edge_min=.05;p.edge_max=1.2;p.overshoot=0.;
  p.max_vertices=3;p.max_attempts=10;p.graph_mode=false;
  p.free_gain=1.;p.path_length_penalty=0.;p.sensor_range=1.;
  p.horizontal_fov=1.;p.horizontal_resolution=1.;p.vertical_fov=.2;p.vertical_resolution=.2;
  p.max_velocity=.3;p.traverse_length_max=1.;p.traverse_time_max=100.;
  p.refinement.enabled=false;p.refinement.interpolation_distance=.5;
  for(auto& axis:p.axis_sampling){axis.distribution="constant";axis.constant=0.;}
  p.axis_sampling[0].constant=4.;return p;
}
}  // namespace

int main(){
  try{
    const std::vector<StateVec> straight{{0.,0.,.6,.7},{1.2,0.,.6,-.4}};
    const auto straight_output=interpolateOutputPath(straight,.5);
    preservesPolyline(straight,straight_output,.5);
    require(straight_output.size()==4&&straight_output[1].x()==.5&&
            straight_output[2].x()==1.&&straight_output.back().x()==1.2,
            "1.2 m segment did not retain 0.5 m spacing and the exact endpoint");

    // Every slope and horizontal turn is an existing checked-segment boundary.
    // Pure subdivision must retain all of them; a start-to-end chord would
    // flatten this rise/fall and leave the L-shaped surface route.
    const std::vector<StateVec> terrain{{0.,0.,.6,.3},{.8,0.,.8,.8},
        {.8,.6,1.,-.4},{1.3,.6,.9,1.2}};
    const auto terrain_output=interpolateOutputPath(terrain,.25);
    preservesPolyline(terrain,terrain_output,.25);
    const auto peak=std::max_element(terrain_output.begin(),terrain_output.end(),
        [](const auto& a,const auto& b){return a.z()<b.z();});
    require(peak!=terrain_output.end()&&exact(*peak,terrain[2]),
            "Terrain peak disappeared during output interpolation");

    const std::vector<StateVec> short_and_yaw{{0.,0.,.6,.7},{.2,0.,.6,-.5},
        {.2,0.,.6,1.3},{.2,0.,.6,1.3},{.4,.1,.6,-.8}};
    const auto short_output=interpolateOutputPath(short_and_yaw,.5);
    preservesPolyline(short_and_yaw,short_output,.5);
    require(short_output.size()==short_and_yaw.size(),"Short/pure-yaw path changed its waypoint count");
    for(std::size_t i=0;i<short_output.size();++i)
      require(exact(short_output[i],short_and_yaw[i]),"An existing short or pure-yaw waypoint changed");
    const auto single=interpolateOutputPath({straight.front()},.5);
    require(single.size()==1&&exact(single.front(),straight.front()),"Single-pose response changed");
    require(interpolateOutputPath({},.5).empty(),"Empty response gained a fabricated pose");

    // Exercise the real planner's complete-edge traversal gate with geometric
    // enhancement disabled. Output interpolation happens on a copy after the
    // 1 m budget has retained its crossing 1.2 m edge; graph IDs still refer to
    // that original endpoint, while stop-and-observe executes the first 0.5 m.
    const auto p=lineParameters();LocalPlanner planner(scene(),p);
    const auto plan=planner.plan(StateVec(0.,0.,.6,.7));
    require(plan.success&&plan.truncated_for_budget&&!plan.refinement.attempted&&
            plan.path.size()==2&&std::abs(plan.traverse_length-1.2)<1e-10&&
            plan.executed_target_id==1,"Unrefined complete-edge planner fixture failed");
    const auto output=interpolateOutputPath(plan.path,p.refinement.interpolation_distance);
    preservesPolyline(plan.path,output,.5);
    require(output.size()==4&&std::abs((output[1]-output.front()).head<3>().norm()-.5)<1e-12,
            "Unrefined planner output has the wrong first execution endpoint");
    require(exact(plan.path.back(),planner.graph().vertices_map_.at(plan.executed_target_id)->state)&&
            exact(output.back(),plan.path.back())&&plan.path.size()==2&&
            std::abs(plan.traverse_time-4.)<1e-10,
            "Output insertion changed graph endpoint association or traversal accounting");

    // The budget is aggregate: original knots count too. It must reject a
    // request before returning a partial route or silently omitting its goal.
    bool bounded=false;
    try{(void)interpolateOutputPath(straight,.5,3);}
    catch(const std::length_error&){bounded=true;}
    require(bounded&&straight.size()==2&&straight.back().x()==1.2,
            "Point-budget failure returned a partial or altered input route");

    std::cout<<std::setprecision(15)
      <<"{\"status\":\"passed\",\"spacing_m\":0.5,\"straight_points\":"<<straight_output.size()
      <<",\"first_execution_distance_m\":"<<(output[1]-output.front()).head<3>().norm()
      <<",\"complete_traversal_edge_m\":"<<plan.traverse_length
      <<",\"traversal_time_s\":"<<plan.traverse_time
      <<",\"graph_target_id\":"<<plan.executed_target_id
      <<",\"terrain_input_knots\":"<<terrain.size()<<",\"terrain_output_points\":"<<terrain_output.size()
      <<",\"terrain_length_m\":"<<length(terrain_output)
      <<",\"terrain_peak_z_m\":"<<peak->z()
      <<",\"short_and_yaw_knots\":"<<short_output.size()
      <<",\"refinement_disabled\":true,\"original_states_exact\":true,"
        "\"candidate_and_graph_untouched\":true,\"point_budget_enforced\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
