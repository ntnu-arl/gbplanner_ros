// Focused numerical checks for four-axis sampling and legacy compatibility.
#include "gbplanner_local/local_planner.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace gbplanner2;
namespace {
void require(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
// The previous ROS 2 LocalSampler, independently retained here to check exact
// seeded compatibility for all three legacy distributions, not just moments.
StateVec previousSample(const Parameters& p,const Eigen::Matrix3d& sampling_to_world,
                        const StateVec& root,std::mt19937& engine){
  StateVec result=root;Eigen::Vector3d offset;double chi=1.;
  if(p.distribution=="cauchy"){
    std::chi_squared_distribution<double> distribution(1.);
    do{chi=distribution(engine);}while(chi<=0.);
  }
  for(int i=0;i<3;++i){
    if(p.distribution=="uniform")
      offset[i]=std::uniform_real_distribution<double>(p.local_min[i],p.local_max[i])(engine);
    else if(p.distribution=="normal")
      offset[i]=std::normal_distribution<double>(p.sample_mean[i],p.sample_scale[i])(engine);
    else offset[i]=std::normal_distribution<double>(0.,p.sample_scale[i])(engine)/std::sqrt(chi)+p.sample_mean[i];
  }
  // The previous constructor cached this evaluated Matrix3d. Leaving a
  // Transpose expression here changes Eigen's accumulation order by a ULP.
  result.head<3>()+=sampling_to_world*offset;
  result[3]+=std::uniform_real_distribution<double>(-M_PI,M_PI)(engine);return result;
}
bool exact(const StateVec& a,const StateVec& b){return (a.array()==b.array()).all();}
}

int main(){
  try{
    int legacy_comparisons=0;
    const StateVec root(.2,-.1,.3,.4);
    for(const std::string distribution:{"uniform","normal","cauchy"}){
      Parameters parameters;parameters.distribution=distribution;
      parameters.local_rotations={.25,-.17,.08};
      parameters.sample_mean={.1,-.2,.3};parameters.sample_scale={.7,.8,.9};
      LocalSampler sampler(parameters);std::mt19937 reference(parameters.seed);
      const Eigen::Matrix3d previous_rotation=parameters.worldToSampling().transpose();
      for(int i=0;i<64;++i){
        require(exact(sampler.generate(root),previousSample(parameters,previous_rotation,root,reference)),
                "A legacy seeded sample changed");++legacy_comparisons;
      }
    }

    Parameters fixed;fixed.local_min={-1.,-1.,-.5};fixed.local_max={1.,1.,.5};
    fixed.local_rotations={.6,0.,0.};fixed.edge_max=3.;fixed.tangent_yaw=false;
    fixed.max_vertices=24;fixed.max_attempts=1000;fixed.free_gain=1.;
    fixed.axis_sampling[0].minimum=-.4;fixed.axis_sampling[0].maximum=.4;
    fixed.axis_sampling[2].distribution="constant";fixed.axis_sampling[2].constant=.15;
    fixed.axis_sampling[3].distribution="constant";fixed.axis_sampling[3].constant=.25;
    LocalSampler fixed_sampler(fixed);
    for(int i=0;i<32;++i){
      const auto point=fixed_sampler.generate(root);
      const Eigen::Vector3d local=fixed.worldToSampling()*(point-root).head<3>();
      require(std::abs(point.z()-(root.z()+.15))<1e-12&&point[3]==root[3]+.25,
              "Constant height/heading did not remain relative to the root");
      require(local.x()>=-.4-1e-12&&local.x()<=.4+1e-12,"Per-axis uniform range was ignored");
    }
    auto map=std::make_shared<VoxelMap>(.25,Eigen::Vector3d(-5.,-5.,-3.),Eigen::Vector3i(40,40,24));
    map->fill(VoxelStatus::Free);
    LocalPlanner fixed_planner(map,fixed);const auto fixed_result=fixed_planner.plan(root);
    require(fixed_result.success&&fixed_result.vertices>2,"Fixed-axis local planning failed");
    for(const auto& item:fixed_planner.graph().vertices_map_){
      if(item.first==0)continue;
      require(std::abs(item.second->state.z()-(root.z()+.15))<1e-12&&
              item.second->state[3]==root[3]+.25,"Graph insertion lost a constant sampled axis");
    }
    auto yaw_range=fixed;yaw_range.axis_sampling[3].distribution="uniform";
    yaw_range.axis_sampling[3].minimum=-.1;yaw_range.axis_sampling[3].maximum=.2;
    LocalSampler yaw_sampler(yaw_range);
    for(int i=0;i<16;++i){
      const double offset=yaw_sampler.generate(root)[3]-root[3];
      require(offset>=-.1-1e-12&&offset<=.2+1e-12,"Heading range was ignored");
    }

    Parameters mixed;mixed.adaptive.enabled=true;mixed.adaptive.leaf=.15;
    mixed.adaptive.size_max=6.;mixed.adaptive.scale_max=2.;
    mixed.axis_sampling[0].distribution="normal";
    mixed.axis_sampling[0].mean=8.;mixed.axis_sampling[0].scale=4.;
    mixed.adaptive_axis_sampling[0].distribution="cauchy";
    mixed.axis_sampling[1].distribution="normal";
    mixed.axis_sampling[1].mean=-7.;mixed.axis_sampling[1].scale=3.;
    mixed.axis_sampling[2].distribution="constant";mixed.axis_sampling[2].constant=.12;
    mixed.axis_sampling[3].distribution="cauchy";
    mixed.axis_sampling[3].mean=.17;mixed.axis_sampling[3].scale=.08;
    const Eigen::Matrix3d rotation=(Eigen::AngleAxisd(.4,Eigen::Vector3d::UnitZ())*
        Eigen::AngleAxisd(.2,Eigen::Vector3d::UnitY())*
        Eigen::AngleAxisd(-.1,Eigen::Vector3d::UnitX())).toRotationMatrix();
    std::vector<Eigen::Vector3d> cloud;
    for(int x=-8;x<=8;++x)for(int y=-3;y<=3;++y)for(double z:{-.4,.4})
      cloud.push_back(root.head<3>()+rotation*Eigen::Vector3d(.3*x,.2*y,z));
    const auto adaptive=AdaptiveBound(mixed.adaptive).fromCloud(cloud,root.head<3>());
    require(adaptive.applied,"PCA geometry was not available");
    mixed.local_min=adaptive.min;mixed.local_max=adaptive.max;mixed.local_rotations=adaptive.rotations;
    mixed.sample_mean=adaptive.mean;mixed.sample_scale=adaptive.scale;
    require(mixed.samplingAxes(true)[0].distribution=="cauchy"&&mixed.samplingAxes()[0].distribution=="normal",
            "Adaptive profile changed the basic exploration profile");
    applyAdaptiveSamplingGeometry(mixed.axis_sampling,adaptive.min,adaptive.max,adaptive.mean,adaptive.scale);
    applyAdaptiveSamplingGeometry(mixed.adaptive_axis_sampling,adaptive.min,adaptive.max,adaptive.mean,adaptive.scale);
    const auto axes=mixed.samplingAxes(true);
    for(int i=0;i<3;++i){
      require(axes[i].minimum==adaptive.min[i]&&axes[i].maximum==adaptive.max[i]&&
              axes[i].mean==adaptive.mean[i]&&axes[i].scale==adaptive.scale[i],
              "PCA did not replace an explicit XYZ bound or moment");
    }
    require(axes[2].constant==.12&&axes[3].mean==.17&&axes[3].scale==.08,
            "PCA changed the constant or Heading parameters");
    LocalSampler mixed_sampler(mixed,axes);std::mt19937 reference(mixed.seed);
    const Eigen::Matrix3d mixed_rotation=mixed.worldToSampling().transpose();
    for(int i=0;i<64;++i){
      double chi;std::chi_squared_distribution<double> chi_distribution(1.);
      do{chi=chi_distribution(reference);}while(chi<=0.);
      Eigen::Vector3d offset;
      offset.x()=std::normal_distribution<double>(0.,adaptive.scale.x())(reference)/std::sqrt(chi)+adaptive.mean.x();
      offset.y()=std::normal_distribution<double>(adaptive.mean.y(),adaptive.scale.y())(reference);
      offset.z()=.12;
      const double heading=std::normal_distribution<double>(0.,.08)(reference)/std::sqrt(chi)+.17;
      StateVec expected=root;expected.head<3>()+=mixed_rotation*offset;
      expected[3]+=heading;
      require(exact(mixed_sampler.generate(root),expected),
              "Mixed axes lost the common Cauchy divisor or PCA parameters");
    }
    std::cout<<std::setprecision(15)<<"{\"status\":\"passed\",\"legacy_exact_samples\":"<<legacy_comparisons
      <<",\"fixed_axis_samples\":32,\"fixed_axis_graph_vertices\":"<<fixed_result.vertices
      <<",\"heading_range_samples\":16,\"mixed_pca_samples\":64,\"pca_points\":"<<adaptive.filtered_points
      <<",\"shared_cauchy_divisor\":true,\"pca_preserves_constant_and_heading\":true}\n";
    return 0;
  }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
