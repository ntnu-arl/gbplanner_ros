// Explicit offline conversion of upstream ROS 1 saved topology to GBP2MEM1.
#include "gbplanner_local/global_memory.hpp"
#include "gbplanner_local/ros1_graph_reader.hpp"
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

namespace {
std::string quote(const std::string& text){
  std::ostringstream out;out<<'"';
  for(unsigned char c:text){
    if(c=='"'||c=='\\')out<<'\\'<<c;
    else if(c<32)out<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
    else out<<c;
  }
  out<<'"';return out.str();
}
void usage(){
  std::cout<<"Convert a fixed GBPlanner 2 ROS 1 saved Graph to a ROS 2 exploration snapshot.\n"
    "Usage: import_ros1_graph --input OLD --output NEW --frame FRAME --robot aerial|ground\n"
    "                         --coordinates planning [--planning-height METRES]\n"
    "Ground mode requires the original max_ground_height, also used by the destination planner.\n"
    "Coordinates remain unchanged. No TF transform, base-height conversion, map or visit history is inferred.\n"
    "The output must not already exist. Load it with the existing gbplanner/load_graph service.\n";
}
}

int main(int argc,char** argv){
  try{
    std::map<std::string,std::string> options;
    for(int i=1;i<argc;++i){
      const std::string key=argv[i];
      if(key=="--help"){usage();return 0;}
      if(key!="--input"&&key!="--output"&&key!="--frame"&&key!="--robot"&&
         key!="--coordinates"&&key!="--planning-height")throw std::invalid_argument("Unknown option: "+key);
      if(i+1>=argc||!options.emplace(key,argv[++i]).second)throw std::invalid_argument("Missing value or duplicate option: "+key);
    }
    for(const auto* key:{"--input","--output","--frame","--robot","--coordinates"})
      if(!options.count(key)||options.at(key).empty())throw std::invalid_argument(std::string("Required option: ")+key);
    if(options.at("--coordinates")!="planning")throw std::invalid_argument("Only explicitly declared planning coordinates are supported");
    const auto& robot=options.at("--robot");
    if(robot!="ground"&&robot!="aerial")throw std::invalid_argument("Robot must be aerial or ground");
    if((robot=="ground")!=bool(options.count("--planning-height")))
      throw std::invalid_argument("--planning-height is required only for ground mode");
    gbplanner2::Parameters parameters;parameters.ground_robot=robot=="ground";
    if(parameters.ground_robot){
      std::size_t used=0;const auto& value=options.at("--planning-height");
      const double height=std::stod(value,&used);
      if(used!=value.size()||!std::isfinite(height)||height<=0.)throw std::invalid_argument("Invalid ground planning height");
      parameters.ground.max_ground_height=height;
    }
    parameters.validate();
    const auto& input=options.at("--input");const auto& output=options.at("--output");
    if(std::filesystem::exists(output))throw std::invalid_argument("Output already exists; choose a new path");
    // Retain the original Header details in the conversion record. The fixed
    // upstream writer normally left them empty/zero; they do not identify a map.
    const auto original=gbplanner2::readRos1Graph(input,options.at("--frame"));
    gbplanner2::GlobalMemory memory(parameters);
    const auto imported=memory.importRos1Graph(input,options.at("--frame"));
    if(!imported.success)throw std::runtime_error(imported.reason);
    const auto saved=memory.saveGraph(output,options.at("--frame"));
    if(!saved.success)throw std::runtime_error(saved.reason);
    std::size_t source_frontiers=0;for(const auto& vertex:original.graph.vertices)source_frontiers+=vertex.is_frontier;
    std::cout<<std::setprecision(17)<<"{\"status\":\"converted\",\"input\":"<<quote(input)
      <<",\"output\":"<<quote(output)<<",\"format\":\"GBP2MEM1\",\"frame\":"<<quote(options.at("--frame"))
      <<",\"robot\":"<<quote(robot)<<",\"coordinates\":\"planning\",\"planning_height\":";
    if(parameters.ground_robot)std::cout<<parameters.ground.max_ground_height;else std::cout<<"null";
    std::cout<<",\"vertices\":"<<imported.vertices<<",\"active_edges\":"<<imported.active_edges
      <<",\"suspended_edges\":"<<imported.suspended_edges<<",\"pending_frontier_identities\":"<<source_frontiers
      <<",\"original_header\":{\"sequence\":"<<original.sequence<<",\"sec\":"<<original.stamp_sec
      <<",\"nanosec\":"<<original.stamp_nanosec<<",\"frame\":"<<quote(original.source_frame)<<"}"
      <<",\"map_revalidated\":false,\"visited_history_restored\":false,\"semantic_state_restored\":false}\n";
    return 0;
  }catch(const std::exception& error){
    std::cout<<"{\"status\":\"failed\",\"error\":"<<quote(error.what())<<"}\n";return 1;
  }
}
