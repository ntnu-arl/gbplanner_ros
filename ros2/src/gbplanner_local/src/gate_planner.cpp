// Adapted from GBPlanner 2 gate search, BSD-3-Clause.
// Copyright (c) 2020 UNR Autonomous Robots Lab.
#include "gbplanner_local/gate_planner.hpp"
#include "gbplanner_local/parameters.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace gbplanner2 {
void GateParameters::validate()const{
  const auto positive=[](double value){return std::isfinite(value)&&value>0.;};
  if(!center.allFinite()||!std::isfinite(heading)||!positive(center_search_radius)||
     !positive(center_search_step)||!positive(line_search_length)||
     !positive(line_search_range)||!positive(line_search_step)||robot_sizes.size()>3)
    throw std::invalid_argument("Invalid gate parameters");
  const double rings=std::floor(center_search_radius/center_search_step);
  const double headings=std::floor(line_search_range/line_search_step);
  // Original loops are exclusive at the configured radius/range. Preserve
  // that ordering and extent while refusing a configuration with no samples.
  if(rings<1.||headings<1.||rings>1000.||headings>1000.||
     (1.+4.*rings*(rings-1.))*(2.*headings-1.)*std::max<std::size_t>(1,robot_sizes.size())>2000000.)
    throw std::invalid_argument("Invalid or excessive gate search size");
  for(std::size_t i=0;i<robot_sizes.size();++i){
    if(!robot_sizes[i].allFinite()||(robot_sizes[i].array()<=0.).any()||
       (i&&(robot_sizes[i].array()>robot_sizes[i-1].array()).any()))
      throw std::invalid_argument("Gate robot sizes must be positive and largest first");
  }
}
GatePlanner::GatePlanner(std::shared_ptr<const MapQuery> map,const Parameters& robot,GateParameters parameters)
    :map_(std::move(map)),p_(std::move(parameters)),robot_size_(robot.robot_size),
     center_offset_(robot.center_offset),ground_robot_(robot.ground_robot){
  p_.validate();robot.validate();if(!map_)throw std::invalid_argument("Gate search needs a map");
  for(const auto& size:p_.robot_sizes)if((size.array()<robot_size_.array()).any())
    throw std::invalid_argument("Gate bound cannot shrink the configured robot body");
}
bool GatePlanner::throughCenter(const StateVec& current,const Eigen::Vector3d& center,
                                 double heading,const Eigen::Vector3d& size,GateResult& result)const{
  const int count=static_cast<int>(p_.line_search_range/p_.line_search_step);
  for(int index=0;index<count;++index){
    // Positive then negative offset, omitting the duplicate zero heading.
    for(int side=0;side<(index?2:1);++side){
      ++result.lines_tested;
      const double angle=heading+(side?-1.:1.)*index*p_.line_search_step;
      const Eigen::Vector3d direction(std::cos(angle),std::sin(angle),0.);
      const Eigen::Vector3d entry=center-.5*p_.line_search_length*direction;
      const Eigen::Vector3d exit=center+.5*p_.line_search_length*direction;
      if(map_->path(current.head<3>()+center_offset_,entry+center_offset_,size)!=VoxelStatus::Free||
         map_->path(entry+center_offset_,exit+center_offset_,size)!=VoxelStatus::Free)continue;
      StateVec start_state,end_state;start_state<<entry,0.;end_state<<exit,0.;
      // The original gate branch uses yaw zero at both generated poses. The
      // separate PCI trajectory layer assigns/interpolates executable yaw.
      result.path={current,start_state,end_state};
      result.path_length=(entry-current.head<3>()).norm()+(exit-entry).norm();
      result.heading=angle;result.center=center;result.robot_size=size;return true;
    }
  }
  return false;
}
GateResult GatePlanner::plan(const StateVec& current)const{return plan(current,p_.center,p_.heading);}
GateResult GatePlanner::plan(const StateVec& current,const Eigen::Vector3d& center,double heading)const{
  GateResult result;
  if(!p_.enabled){result.reason="gate_disabled";return result;}
  if(!current.allFinite()||!center.allFinite()||!std::isfinite(heading)){
    result.reason="invalid_gate_pose";return result;
  }
  // Upstream gate paths have a fixed-height line and no terrain projection.
  // Returning it for a ground robot would bypass the implemented support and
  // directed-slope checks; ground gate planning is not advertised here.
  if(ground_robot_){result.reason="ground_gate_not_supported";return result;}
  const auto sizes=p_.robot_sizes.empty()?std::vector<Eigen::Vector3d>{robot_size_}:p_.robot_sizes;
  const int rings=static_cast<int>(p_.center_search_radius/p_.center_search_step);
  for(std::size_t level=0;level<sizes.size();++level){
    const auto& size=sizes[level];
    const auto tryCenter=[&](const Eigen::Vector3d& candidate){
      ++result.centers_tested;
      if(map_->box(candidate+center_offset_,size)!=VoxelStatus::Free)return false;
      if(!throughCenter(current,candidate,heading,size,result))return false;
      result.success=true;result.reason="ok";result.bound_level=static_cast<int>(level);
      result.bound_mode=p_.robot_sizes.empty()?3:static_cast<int>(level);return true;
    };
    for(int ring=0;ring<rings;++ring){
      if(!ring){if(tryCenter(center))return result;continue;}
      const double offset=ring*p_.center_search_step;
      // Preserve the upstream world-YZ square-ring scan. Heading rotates only
      // the pass-through line, not the center search plane or its order.
      for(int index=-ring+1;index<ring;++index){
        if(tryCenter(center+Eigen::Vector3d(0.,offset,index*p_.center_search_step)))return result;
        if(tryCenter(center+Eigen::Vector3d(0.,-offset,index*p_.center_search_step)))return result;
      }
      for(int index=-ring;index<=ring;++index){
        if(tryCenter(center+Eigen::Vector3d(0.,index*p_.center_search_step,offset)))return result;
        if(tryCenter(center+Eigen::Vector3d(0.,index*p_.center_search_step,-offset)))return result;
      }
    }
  }
  result.reason="no_gate_path";return result;
}
}
