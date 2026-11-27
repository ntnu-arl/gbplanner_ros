#include "gbplanner_voxblox/tsdf_backend.hpp"
#include "gbplanner_voxblox/layer_conversions.hpp"
#include "gbplanner_voxblox/distance_products.hpp"
#include "gbplanner_voxblox/map_visualization.hpp"
#include "gbplanner_voxblox/mesh_messages.hpp"
#include "gbplanner_voxblox/transform_queue.hpp"
#include "gbplanner_voxblox/icp_refiner.hpp"
#include "gbplanner_voxblox/pointcloud_buffer.hpp"
#include "gbplanner_voxblox/voxel_scan_capture.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <voxblox/io/mesh_ply.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <gbplanner_voxblox/srv/file_path.hpp>
#include <planner_msgs/msg/voxel_map_update.hpp>
#include <planner_msgs/srv/augment_free_box.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <chrono>
#include <csignal>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>
#include <set>
#include <array>
#include <deque>
#include <sstream>
#include <random>
#include <optional>
#include <unistd.h>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace {
volatile std::sig_atomic_t stop_requested=0;
void requestStop(int){stop_requested=1;}
std::string newMapSourceId(){
  std::random_device random;
  std::ostringstream id;
  id<<"voxblox-"<<getpid()<<'-'<<std::hex
    <<std::chrono::system_clock::now().time_since_epoch().count()<<'-'
    <<random()<<'-'<<random();
  return id.str();
}
}

namespace gbplanner2 {
class VoxbloxMapperNode final:public rclcpp::Node {
 public:
  VoxbloxMapperNode():Node("voxblox_mapper") {
    const auto snapshot_trace=parameter<std::string>("diagnostics.snapshot_trace_file","");
    if(!snapshot_trace.empty()){
      snapshot_trace_.open(snapshot_trace,std::ios::out|std::ios::app);
      if(!snapshot_trace_)throw std::invalid_argument("Cannot open diagnostics.snapshot_trace_file");
    }
    frame_=parameter<std::string>("frame_id","map");
    sensor_frame_=parameter<std::string>("sensor_frame","lidar");
    if(frame_.empty())throw std::invalid_argument("Empty frame name");
    storage_=parameter<std::string>("map.storage","dense");
    if(storage_!="dense"&&storage_!="sparse")throw std::invalid_argument("map.storage must be dense or sparse");
    snapshot_publish_period_=parameter<double>("map.snapshot_publish_period",0.0);
    if(!std::isfinite(snapshot_publish_period_)||snapshot_publish_period_<0.||snapshot_publish_period_>3600.)
      throw std::invalid_argument("map.snapshot_publish_period must be finite and in [0, 3600]");
    double resolution=parameterWithLegacyAlias<double>("map.resolution","tsdf_voxel_size",0.25);
    auto origin=parameter<std::vector<double>>("map.origin",{-5,-5,-1.5});
    auto dimensions=parameter<std::vector<int64_t>>("map.dimensions",{40,40,12});
    if(storage_=="dense"&&(origin.size()!=3||dimensions.size()!=3))
      throw std::invalid_argument("Map shape requires three components in dense mode");
    if(storage_=="dense")for(auto d:dimensions)if(d<1||d>10000)throw std::invalid_argument("Invalid map dimension");
    TsdfConfig config;config.resolution=resolution;
    config.storage=storage_;
    if(storage_=="dense"){
      config.origin={origin[0],origin[1],origin[2]};
      config.dimensions={int(dimensions[0]),int(dimensions[1]),int(dimensions[2])};
    }
    config.voxels_per_side=parameterWithLegacyAlias<int>("tsdf.voxels_per_side","tsdf_voxels_per_side",16);
    config.truncation_distance=parameterWithLegacyAlias<double>("tsdf.truncation_distance","truncation_distance",3*resolution);
    config.method=parameterWithLegacyAlias<std::string>("tsdf.method","method","fast");
    config.threads=parameter<int>("tsdf.threads",1);
    config.max_weight=parameterWithLegacyAlias<double>("tsdf.max_weight","max_weight",200.);
    config.clearing_weight=parameterWithLegacyAlias<double>("tsdf.clearing_ray_weight_factor","clearing_ray_weight_factor",0.01);
    config.weight_dropoff=parameterWithLegacyAlias<bool>("tsdf.use_weight_dropoff","use_weight_dropoff",true);
    config.use_const_weight=parameterWithLegacyAlias<bool>("tsdf.use_const_weight","use_const_weight",!config.weight_dropoff);
    config.use_symmetric_weight_dropoff=parameterWithLegacyAlias<bool>(
      "tsdf.use_symmetric_weight_dropoff","use_symmetric_weight_dropoff",false);
    config.weight_ray_by_range=parameterWithLegacyAlias<bool>("tsdf.weight_ray_by_range","weight_ray_by_range",false);
    config.carving=parameterWithLegacyAlias<bool>("tsdf.voxel_carving_enabled","voxel_carving_enabled",true);
    config.allow_clear=parameterWithLegacyAlias<bool>("tsdf.allow_clear","allow_clear",true);
    config.integration_order_mode=parameterWithLegacyAlias<std::string>(
      "tsdf.integration_order_mode","integration_order_mode","mixed");
    config.start_voxel_subsampling_factor=parameterWithLegacyAlias<double>(
      "tsdf.start_voxel_subsampling_factor","start_voxel_subsampling_factor",2.);
    config.max_consecutive_ray_collisions=parameterWithLegacyAlias<int>(
      "tsdf.max_consecutive_ray_collisions","max_consecutive_ray_collisions",0);
    config.clear_checks_every_n_frames=parameterWithLegacyAlias<int>(
      "tsdf.clear_checks_every_n_frames","clear_checks_every_n_frames",1);
    config.max_integration_time_s=parameterWithLegacyAlias<double>(
      "tsdf.max_integration_time_s","max_integration_time_s",std::numeric_limits<float>::max());
    config.anti_grazing=parameterWithLegacyAlias<bool>("tsdf.anti_grazing","anti_grazing",false);
    config.occupancy_factor=parameter<double>("tsdf.occupancy_distance_factor",1.0);
    config.min_range=parameterWithLegacyAlias<double>("sensor.min_range","min_ray_length_m",0.1);
    config.max_range=parameterWithLegacyAlias<double>("sensor.max_range","max_ray_length_m",6.0);
    const double maximum_block_distance=parameter<double>("max_block_distance_from_body",
      static_cast<double>(std::numeric_limits<voxblox::FloatingPoint>::max()));
    if(!std::isfinite(maximum_block_distance)||maximum_block_distance<0.)
      throw std::invalid_argument("max_block_distance_from_body must be finite and nonnegative");
    // The upstream parameter is double even though its default is float::max.
    max_block_distance_=maximum_block_distance;
    config.esdf_enabled=parameter<bool>("esdf.enabled",false);
    config.esdf_max_distance=parameterWithLegacyAlias<double>("esdf.max_distance","esdf_max_distance_m",2.0);
    config.esdf_full_euclidean_distance=parameterWithLegacyAlias<bool>(
      "esdf.full_euclidean_distance","esdf_euclidean_distance",false);
    config.esdf_min_distance=parameterWithLegacyAlias<double>(
      "esdf.min_distance","esdf_min_distance_m",config.truncation_distance/2.);
    config.esdf_default_distance=parameterWithLegacyAlias<double>(
      "esdf.default_distance","esdf_default_distance_m",2.0);
    config.esdf_min_diff=parameterWithLegacyAlias<double>("esdf.min_diff","esdf_min_diff_m",.001);
    config.esdf_add_occupied_crust=parameterWithLegacyAlias<bool>(
      "esdf.add_occupied_crust","esdf_add_occupied_crust",false);
    config.esdf_clear_sphere_for_planning=parameterWithLegacyAlias<bool>(
      "esdf.clear_sphere_for_planning","clear_sphere_for_planning",false);
    config.esdf_clear_sphere_radius=parameterWithLegacyAlias<double>(
      "esdf.clear_sphere_radius","clear_sphere_radius",1.5);
    config.esdf_occupied_sphere_radius=parameterWithLegacyAlias<double>(
      "esdf.occupied_sphere_radius","occupied_sphere_radius",5.0);
    clear_sphere_pose_source_=parameter<std::string>("esdf.clear_sphere_pose_source","sensor");
    if(clear_sphere_pose_source_!="sensor"&&clear_sphere_pose_source_!="odometry")
      throw std::invalid_argument("esdf.clear_sphere_pose_source must be sensor or odometry");
    config.distance_field=parameter<std::string>("map.distance_field","tsdf");
    // Zero retains the ROS2 adapter's synchronous mode. The RMF profile opts
    // into the original EsdfServer's one-second ROS-clock update interval.
    esdf_update_period_=parameter<double>("update_esdf_every_n_sec",0.0);
    if(!std::isfinite(esdf_update_period_)||esdf_update_period_<0.||esdf_update_period_>3600.)
      throw std::invalid_argument("update_esdf_every_n_sec must be in [0, 3600]");
    config.esdf_update_on_scan=esdf_update_period_==0.;
    if(esdf_update_period_>0.&&config.esdf_clear_sphere_for_planning)
      throw std::invalid_argument("Periodic ESDF currently requires clear_sphere_for_planning=false");
    color_mode_=parameter<std::string>("color_mode","colors");
    mesh_color_mode_=meshColorModeFromString(color_mode_);
    // Upstream display modes do not discard the sensor's stored RGB. Keep
    // only the pre-existing ROS2 'none' alias as an explicit color-off mode.
    config.use_color=color_mode_!="none";
    config.mesh_min_weight=parameterWithLegacyAlias<double>("mesh.min_weight","mesh_min_weight",1e-4);
    config.mesh_use_color=parameterWithLegacyAlias<bool>("mesh.use_color","mesh_use_color",true);
    if(!config.mesh_use_color&&(mesh_color_mode_==MeshColorMode::Color||mesh_color_mode_==MeshColorMode::LambertColor))
      throw std::invalid_argument("mesh.use_color=false requires color_mode=gray, height, normals or lambert");
    occupancy_min_distance_factor_=parameter<double>("occupancy_min_distance_voxel_size_factor",1.0);
    if(!std::isfinite(occupancy_min_distance_factor_))
      throw std::invalid_argument("occupancy_min_distance_voxel_size_factor must be finite");
    update_mesh_every_n_sec_=parameter<double>("update_mesh_every_n_sec",0.0);
    if(!std::isfinite(update_mesh_every_n_sec_)||update_mesh_every_n_sec_<0.)
      throw std::invalid_argument("update_mesh_every_n_sec must be finite and non-negative");
    // Use GBPlanner's ROS 1 SMB/RMF profile depth (1000); the standalone
    // Voxblox server defaults to 1. Subscriber depth and the internal
    // TF-waiting queue are separate (the latter is fixed at ten below).
    pointcloud_queue_size_=parameter<int>("pointcloud_queue_size",1000);
    pointcloud_reliability_=parameter<std::string>("pointcloud_reliability","best_effort");
    if(pointcloud_reliability_!="best_effort"&&pointcloud_reliability_!="reliable")
      throw std::invalid_argument("pointcloud_reliability must be best_effort or reliable");
    min_time_between_msgs_sec_=parameter<double>("min_time_between_msgs_sec",0.0);
    timestamp_tolerance_sec_=parameter<double>("timestamp_tolerance_sec",0.001);
    if(pointcloud_queue_size_<1||pointcloud_queue_size_>100000||
       !std::isfinite(min_time_between_msgs_sec_)||min_time_between_msgs_sec_<0.||min_time_between_msgs_sec_>1e6||
       !std::isfinite(timestamp_tolerance_sec_)||timestamp_tolerance_sec_<0.||timestamp_tolerance_sec_>1e6)
      throw std::invalid_argument("Invalid pointcloud queue/timestamp parameters");
    min_time_between_msgs_ns_=std::llround(min_time_between_msgs_sec_*1e9);
    // Tolerance belongs exclusively to the transform-topic queue. TF lookup
    // continues to request the exact acquisition time.
    use_tf_transforms_=parameter<bool>("use_tf_transforms",true);
    const double tf_cache_time_sec=parameter<double>("tf.cache_time_sec",10.0);
    if(!std::isfinite(tf_cache_time_sec)||tf_cache_time_sec<=0.)
      throw std::invalid_argument("tf.cache_time_sec must be finite and positive");
    transform_queue_=std::make_unique<TransformQueue>(static_cast<int64_t>(timestamp_tolerance_sec_*1e9));
    map_file_=parameter<std::string>("map_file","");
    mesh_file_=parameter<std::string>("mesh_file","");
    const auto startup_map=parameter<std::string>("load_map_file","");
    backend_=std::make_unique<TsdfBackend>(config);
    const auto voxel_capture=parameter<std::string>("diagnostics.voxel_capture_directory","");
    const auto voxel_indices=parameter<std::vector<int64_t>>("diagnostics.voxel_capture_indices",{});
    const auto voxel_log_all_changes=parameter<bool>("diagnostics.voxel_capture_log_all_changes",true);
    if(!voxel_capture.empty())voxel_scan_capture_=std::make_unique<VoxelScanCapture>(
      voxel_capture,voxel_indices,backend_->config(),voxel_log_all_changes);
    mesh_layer_=std::make_unique<voxblox::MeshLayer>(backend_->tsdfLayer().block_size());
    if(!startup_map.empty()){backend_->loadMap(startup_map);++loads_;mesh_dirty_=true;}
    tf_buffer_=std::make_unique<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(tf_cache_time_sec));
    tf_listener_=std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    map_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("voxel_map",rclcpp::QoS(1).reliable().transient_local());
    map_update_pub_=create_publisher<planner_msgs::msg::VoxelMapUpdate>(
      "voxel_map_update",rclcpp::QoS(1).reliable().transient_local());
    tsdf_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("tsdf_voxels",rclcpp::QoS(1).reliable().transient_local());
    esdf_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("esdf_voxels",rclcpp::QoS(1).reliable().transient_local());
    mesh_pub_=create_publisher<visualization_msgs::msg::Marker>("surface_mesh",rclcpp::QoS(1).reliable().transient_local());
    mesh_blocks_pub_=create_publisher<voxblox_msgs::msg::Mesh>("mesh",rclcpp::QoS(10).reliable().durability_volatile());
    mesh_readers_timer_=create_wall_timer(std::chrono::milliseconds(100),[this](){
      std::set<std::string> readers;bool new_reader=false;
      for(const auto& endpoint:get_subscriptions_info_by_topic(mesh_blocks_pub_->get_topic_name())){
        const auto& id=endpoint.endpoint_gid();const std::string key(id.begin(),id.end());
        readers.insert(key);if(!mesh_readers_.count(key))new_reader=true;
      }
      mesh_readers_=std::move(readers);
      // A late reader needs all existing blocks, not the last delta. This
      // watcher never runs meshing or overrides the configured mesh interval.
      if(new_reader)try{publishMeshBlocks(false);}
      catch(const std::exception& error){RCLCPP_WARN(get_logger(),"Mesh snapshot failed: %s",error.what());}
    });
    publish_slices_=parameter<bool>("publish_slices",false);
    publish_pointclouds_=parameter<bool>("publish_pointclouds",false);
    publish_pointclouds_on_update_=parameter<bool>("publish_pointclouds_on_update",false);
    publish_traversable_=parameter<bool>("publish_traversable",false);
    slice_level_=parameter<double>("slice_level",.5);
    traversability_radius_=parameter<double>("traversability_radius",1.0);
    if(!std::isfinite(slice_level_)||!std::isfinite(traversability_radius_)||traversability_radius_<0.)
      throw std::invalid_argument("Invalid slice_level or traversability_radius");
    if(publish_traversable_&&!backend_->esdfLayer())
      throw std::invalid_argument("publish_traversable requires an enabled ESDF");
    const auto product_qos=rclcpp::QoS(1).reliable().transient_local();
    surface_cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("surface_pointcloud",product_qos);
    tsdf_cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("tsdf_pointcloud",product_qos);
    esdf_cloud_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("esdf_pointcloud",product_qos);
    occupied_nodes_pub_=create_publisher<visualization_msgs::msg::MarkerArray>("occupied_nodes",product_qos);
    tsdf_slice_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("tsdf_slice",product_qos);
    esdf_slice_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("esdf_slice",product_qos);
    traversable_pub_=create_publisher<sensor_msgs::msg::PointCloud2>("traversable",product_qos);
    publish_clouds_service_=create_service<std_srvs::srv::Trigger>("publish_pointclouds",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        publish(now(),true);response->success=true;
        response->message="Published current surface, distance, occupancy and enabled slice/traversable products";
      });
    configureLayerTransport();
    configureInputs();
    configureFreeSpace();
    configureTransforms();
    configureIcp();
    if(backend_->esdfLayer()&&esdf_update_period_>0.)
      esdf_timer_=rclcpp::create_timer(this,get_clock(),rclcpp::Duration::from_seconds(esdf_update_period_),[this](){
        if(!esdf_pending_)return;
        updatePendingEsdf();publishEsdfVoxels();publishEsdfProducts(publish_pointclouds_,esdf_stamp_);
      });
    pending_timer_=create_wall_timer(std::chrono::milliseconds(20),[this](){
      for(std::size_t i=0;i<inputs_.size();++i)drainInput(i);
      drainFreeSpace();
    });
    if(snapshot_publish_period_>0.)
      snapshot_timer_=create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(.001,std::min(.05,snapshot_publish_period_)))),
        [this](){flushPendingObservation();});
    status_service_=create_service<std_srvs::srv::Trigger>("mapper_status",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        response->success=scans_>0||loads_>0||freespace_scans_>0||box_clears_>0||esdf_clear_sphere_operations_>0||received_layers_>0;
        response->message="{\"integrated_scans\":"+std::to_string(scans_)+",\"rejected_scans\":"+
          std::to_string(rejected_)+",\"known_cells\":"+std::to_string(known_)+
          ",\"accepted_points\":"+std::to_string(last_.accepted_points)+
          ",\"skipped_points\":"+std::to_string(last_.skipped_points)+",\"backend\":\"voxblox_tsdf\",\"allocated_blocks\":"+std::to_string(backend_->allocatedBlocks())+",\"loaded_maps\":"+std::to_string(loads_)+",\"distance_field\":\""+backend_->config().distance_field+"\",\"input_sources\":"+inputStatus()+",\"freespace\":"+freeSpaceStatus()+
          ",\"color_mode\":\""+color_mode_+"\",\"update_mesh_every_n_sec\":"+std::to_string(update_mesh_every_n_sec_)+
          ",\"mesh_updates\":"+std::to_string(mesh_updates_)+",\"storage\":\""+storage_+
          "\",\"published_cells\":"+std::to_string(published_cells_)+
          ",\"known_cells_scope\":\"last_published_snapshot\""+
          ",\"snapshot_publish_period\":"+std::to_string(snapshot_publish_period_)+
          ",\"esdf_update_period\":"+std::to_string(esdf_update_period_)+
          ",\"esdf_pending\":"+(esdf_pending_?"true":"false")+
          ",\"esdf_updates\":"+std::to_string(esdf_updates_)+
          ",\"esdf_input_sequence\":"+std::to_string(esdf_input_sequence_)+
          ",\"esdf_completed_sequence\":"+std::to_string(esdf_completed_sequence_)+
          ",\"esdf_observation_stamp_ns\":"+std::to_string(rclcpp::Time(esdf_stamp_).nanoseconds())+
          ",\"pending_observation_scans\":"+std::to_string(pending_observation_scans_)+
          ",\"snapshot_publications\":"+std::to_string(snapshot_publications_)+
          ",\"observation_snapshot_publications\":"+std::to_string(observation_snapshot_publications_)+
          ",\"coalesced_observation_scans\":"+std::to_string(coalesced_observation_scans_)+
          ",\"pointcloud_queue_size\":"+std::to_string(pointcloud_queue_size_)+
          ",\"pointcloud_reliability\":"+jsonString(pointcloud_reliability_)+
          ",\"min_time_between_msgs_sec\":"+std::to_string(min_time_between_msgs_sec_)+
          ",\"timestamp_tolerance_sec\":"+std::to_string(timestamp_tolerance_sec_)+
          ",\"use_tf_transforms\":"+(use_tf_transforms_?"true":"false")+
          ",\"queued_transforms\":"+std::to_string(transform_queue_->size())+
          ",\"received_transforms\":"+std::to_string(received_transforms_)+
          ",\"rejected_transforms\":"+std::to_string(rejected_transforms_)+
          ",\"enable_icp\":"+(icp_refiner_?"true":"false")+
          ",\"icp_runs\":"+std::to_string(icp_runs_)+
          ",\"icp_last_successful_batches\":"+std::to_string(icp_last_batches_)+
          ",\"icp_successful_batches\":"+std::to_string(icp_batches_)+
          ",\"max_block_distance_from_body\":"+std::to_string(max_block_distance_)+
          ",\"pruned_tsdf_blocks\":"+std::to_string(pruned_tsdf_blocks_)+
          ",\"pruned_esdf_blocks\":"+std::to_string(pruned_esdf_blocks_)+
          ",\"pruned_mesh_blocks\":"+std::to_string(pruned_mesh_blocks_)+
          ",\"esdf_configuration\":"+esdfConfiguration()+
          ",\"tsdf_configuration\":"+tsdfConfiguration()+
          ",\"mesh_configuration\":"+meshConfiguration()+
          ",\"received_layers\":"+std::to_string(received_layers_)+
          ",\"rejected_layers\":"+std::to_string(rejected_layers_)+
          ",\"esdf_clear_sphere_operations\":"+std::to_string(esdf_clear_sphere_operations_)+
          ",\"map_source_id\":"+jsonString(map_source_id_)+
          ",\"map_epoch\":"+std::to_string(map_epoch_)+
          ",\"map_update_id\":"+std::to_string(map_update_id_)+
          ",\"map_observation_id\":"+std::to_string(map_observation_id_)+
          ",\"map_change_kind\":"+jsonString(map_change_kind_)+"}";
      });
    reset_service_=create_service<std_srvs::srv::Trigger>("reset_map",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        backend_->reset();known_=0;last_={};scans_=0;rejected_=0;loads_=0;
        freespace_scans_=freespace_rejected_=freespace_throttled_=freespace_queue_dropped_=empty_freespace_=box_clears_=cleared_voxels_=0;
        esdf_clear_sphere_operations_=0;pruned_tsdf_blocks_=pruned_esdf_blocks_=pruned_mesh_blocks_=0;
        last_freespace_={};freespace_pending_.clear();freespace_has_last_accepted_=false;
        freespace_last_stamp_ns_=0;startup_box_done_=false;resetInputStats();clearMesh();
        resetMapEpoch();publish(now(),true,"reset");
        publishLayers(true);
        response->success=true;response->message="Voxblox TSDF layer reset to unknown";
      });
    using FilePath=gbplanner_voxblox::srv::FilePath;
    save_service_=create_service<FilePath>("save_map",
      [this](std::shared_ptr<FilePath::Request> request,std::shared_ptr<FilePath::Response> response){
        try{flushEsdfProducts();backend_->saveMap(request->file_path.empty()?map_file_:request->file_path);
          response->success=true;response->message="Saved original Voxblox TSDF layer";
        }catch(const std::exception& error){response->success=false;response->message=error.what();}
      });
    load_service_=create_service<FilePath>("load_map",
      [this](std::shared_ptr<FilePath::Request> request,std::shared_ptr<FilePath::Response> response){
        try{flushEsdfProducts();backend_->loadMap(request->file_path.empty()?map_file_:request->file_path);
          ++loads_;clearMesh();mesh_dirty_=true;resetMapEpoch();publish(now(),false,"load");
          publishLayers(true);
          response->success=true;response->message="Loaded TSDF blocks and rebuilt derived distance field";
        }catch(const std::exception& error){response->success=false;response->message=error.what();}
      });
    mesh_service_=create_service<std_srvs::srv::Trigger>("generate_mesh",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        try{response->message=generateMesh(mesh_file_);response->success=true;}
        catch(const std::exception& error){response->success=false;response->message=error.what();}
      });
    export_service_=create_service<FilePath>("export_mesh",
      [this](std::shared_ptr<FilePath::Request> request,std::shared_ptr<FilePath::Response> response){
        try{auto file=request->file_path.empty()?mesh_file_:request->file_path;
          if(file.empty())throw std::invalid_argument("Mesh file path is empty");
          response->message=generateMesh(file);response->success=true;
        }catch(const std::exception& error){response->success=false;response->message=error.what();}
      });
    if(!startup_map.empty())publish(now(),false,"load");
    if(update_mesh_every_n_sec_>0.){
      mesh_timer_=create_wall_timer(std::chrono::duration<double>(update_mesh_every_n_sec_),[this](){
        if(!mesh_dirty_)return;
        try{generateMesh("",true);}
        catch(const std::exception& error){RCLCPP_WARN(get_logger(),"Periodic mesh update failed: %s",error.what());}
      });
    }
    RCLCPP_INFO(get_logger(),"Original Voxblox TSDF mapper ready: %zu input(s) -> %s",inputs_.size(),frame_.c_str());
  }
  // Called after spin_once has returned, including rclcpp's callback/message
  // cleanup. Only spins that generated a snapshot add this final boundary.
  void traceExecutorReturn(){
    if(!snapshot_trace_.is_open()||snapshot_trace_returned_==snapshot_trace_sequence_)return;
    traceSnapshot("executor_callback_returned",snapshot_trace_stamp_,snapshot_trace_bytes_,snapshot_trace_sequence_);
    snapshot_trace_returned_=snapshot_trace_sequence_;
  }
 private:
  void traceSnapshot(const char* phase,const builtin_interfaces::msg::Time& stamp,
                     std::size_t bytes=0,uint64_t sequence=0,int source_index=-1,
                     const char* tf_error=nullptr)noexcept{
    if(!snapshot_trace_.is_open())return;
    try{
      const auto steady=std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
      timespec cpu{};clock_gettime(CLOCK_THREAD_CPUTIME_ID,&cpu);
      snapshot_trace_<<std::setprecision(15)<<"{\"phase\":"<<std::quoted(phase)<<",\"steady_s\":"<<steady
        <<",\"thread_cpu_s\":"<<(double(cpu.tv_sec)+double(cpu.tv_nsec)*1e-9)
        <<",\"ros_s\":"<<now().seconds()<<",\"stamp_ns\":"<<(int64_t(stamp.sec)*1000000000LL+stamp.nanosec)
        <<",\"snapshot_sequence\":"<<(sequence?sequence:snapshot_trace_sequence_)
        <<",\"map_source_id\":"<<std::quoted(map_source_id_)<<",\"map_epoch\":"<<map_epoch_
        <<",\"update_id\":"<<map_update_id_<<",\"observation_id\":"<<map_observation_id_
        <<",\"bytes\":"<<bytes<<",\"source_index\":"<<source_index
        <<",\"integrated_scans\":"<<scans_<<",\"pending_observation_scans\":"<<pending_observation_scans_
        <<",\"allocated_blocks\":"<<(backend_?backend_->allocatedBlocks():0)<<",\"input_queues\":[";
      for(std::size_t i=0;i<inputs_.size();++i){
        if(i)snapshot_trace_<<',';
        const auto& source=inputs_[i];
        snapshot_trace_<<"{\"name\":"<<std::quoted(source.name)<<",\"pending\":"<<source.pending.size()
          <<",\"last_received_stamp_ns\":"<<source.last_stamp_ns<<",\"integrated\":"<<source.integrated
          <<",\"queue_dropped\":"<<source.queue_dropped<<'}';
      }
      snapshot_trace_<<']';
      if(backend_&&std::strcmp(phase,"scan_integrate_end")==0){
        const auto& timing=backend_->lastIntegrationTiming();
        snapshot_trace_<<",\"filter_wall_s\":"<<timing.filter_wall_s
          <<",\"tsdf_wall_s\":"<<timing.tsdf_wall_s<<",\"esdf_wall_s\":"<<timing.esdf_wall_s
          <<",\"backend_wall_s\":"<<timing.total_wall_s
          <<",\"esdf_updated\":"<<(timing.esdf_updated?"true":"false");
      }
      if(tf_error)snapshot_trace_<<",\"tf_error\":"<<jsonString(tf_error);
      snapshot_trace_<<"}\n";snapshot_trace_.flush();
    }catch(...){
      // Optional diagnostics must not alter map publication or stop unwinding.
    }
  }
  struct SnapshotTraceEnd {
    VoxbloxMapperNode* node;const char* phase;builtin_interfaces::msg::Time stamp;
    std::size_t bytes{0};uint64_t sequence{0};int source_index{-1};
    ~SnapshotTraceEnd(){node->traceSnapshot(phase,stamp,bytes,sequence,source_index);}
  };
  template<class T>T parameter(const std::string& name,const T& value){
    rcl_interfaces::msg::ParameterDescriptor descriptor;descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  template<class T>T parameterWithLegacyAlias(const std::string& name,const std::string& legacy,
                                               const T& default_value){
    const T primary=parameter<T>(name,default_value),compatibility=parameter<T>(legacy,default_value);
    // Presence matters: an explicit false/default value must override an old
    // spelling too. Comparing values with defaults loses that information.
    const auto& overrides=get_node_parameters_interface()->get_parameter_overrides();
    const bool primary_set=overrides.count(name)>0,compatibility_set=overrides.count(legacy)>0;
    if(primary_set&&compatibility_set&&primary!=compatibility)
      RCLCPP_WARN(get_logger(),"Both '%s' and legacy '%s' are set differently; using '%s'",
        name.c_str(),legacy.c_str(),name.c_str());
    return primary_set?primary:compatibility;
  }
  struct InputSource {
    std::string name,topic,frame;
    uint64_t integrated{0},rejected{0},throttled{0},queue_dropped{0};
    bool has_last_accepted{false};
    int64_t last_stamp_ns{0};
    std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> pending;
  };
  static std::string jsonString(const std::string& value){
    std::string result="\"";
    const char* hex="0123456789abcdef";
    for(unsigned char c:value){
      if(c=='"'||c=='\\'){result+='\\';result+=char(c);}
      else if(c<0x20){result+="\\u00";result+=hex[c>>4];result+=hex[c&15];}
      else result+=char(c);
    }
    return result+"\"";
  }
  std::string inputStatus()const{
    std::ostringstream out;out<<"{";
    for(std::size_t i=0;i<inputs_.size();++i){
      const auto& source=inputs_[i];if(i)out<<",";
      out<<jsonString(source.name)<<":{\"topic\":"<<jsonString(source.topic)
         <<",\"frame\":"<<jsonString(source.frame)
         <<",\"subscription_depth\":"<<cloud_subscriptions_[i]->get_actual_qos().get_rmw_qos_profile().depth
         <<",\"subscription_reliability\":"<<jsonString(reliabilityName(
              cloud_subscriptions_[i]->get_actual_qos().get_rmw_qos_profile().reliability))
         <<",\"integrated_scans\":"<<source.integrated
         <<",\"rejected_scans\":"<<source.rejected
         <<",\"throttled_scans\":"<<source.throttled
         <<",\"queue_dropped_scans\":"<<source.queue_dropped
         <<",\"pending_scans\":"<<source.pending.size()<<"}";
    }
    out<<"}";return out.str();
  }
  void resetInputStats(){
    for(auto& source:inputs_){
      source.integrated=0;source.rejected=0;source.throttled=0;
      source.has_last_accepted=false;source.last_stamp_ns=0;
      source.pending.clear();source.queue_dropped=0;
    }
  }
  rclcpp::SensorDataQoS pointCloudQoS()const{
    rclcpp::SensorDataQoS qos(rclcpp::KeepLast(static_cast<std::size_t>(pointcloud_queue_size_)));
    if(pointcloud_reliability_=="reliable")qos.reliable();
    return qos;
  }
  static const char* reliabilityName(rmw_qos_reliability_policy_t reliability){
    switch(reliability){
      case RMW_QOS_POLICY_RELIABILITY_RELIABLE:return "reliable";
      case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:return "best_effort";
      case RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT:return "system_default";
      default:return "unknown";
    }
  }
  std::string tsdfConfiguration()const{
    const auto& config=backend_->config();std::ostringstream out;
    out.precision(std::numeric_limits<double>::max_digits10);out<<std::boolalpha;
    out<<"{\"method\":"<<jsonString(config.method)
       <<",\"resolution\":"<<config.resolution<<",\"voxels_per_side\":"<<config.voxels_per_side
       <<",\"truncation_distance\":"<<config.truncation_distance
       <<",\"min_range\":"<<config.min_range<<",\"max_range\":"<<config.max_range
       <<",\"max_weight\":"<<config.max_weight<<",\"clearing_ray_weight_factor\":"<<config.clearing_weight
       <<",\"threads\":"<<config.threads<<",\"use_weight_dropoff\":"<<config.weight_dropoff
       <<",\"use_const_weight\":"<<config.use_const_weight.value()
       <<",\"use_symmetric_weight_dropoff\":"<<config.use_symmetric_weight_dropoff
       <<",\"weight_ray_by_range\":"<<config.weight_ray_by_range
       <<",\"voxel_carving_enabled\":"<<config.carving<<",\"allow_clear\":"<<config.allow_clear
       <<",\"integration_order_mode\":"<<jsonString(config.integration_order_mode)
       <<",\"start_voxel_subsampling_factor\":"<<config.start_voxel_subsampling_factor
       <<",\"max_consecutive_ray_collisions\":"<<config.max_consecutive_ray_collisions
       <<",\"clear_checks_every_n_frames\":"<<config.clear_checks_every_n_frames
       <<",\"max_integration_time_s\":"<<config.max_integration_time_s
       <<",\"anti_grazing\":"<<config.anti_grazing<<"}";
    return out.str();
  }
  std::string meshConfiguration()const{
    const auto& config=backend_->config();std::ostringstream out;
    out.precision(std::numeric_limits<double>::max_digits10);out<<std::boolalpha;
    out<<"{\"min_weight\":"<<config.mesh_min_weight<<",\"use_color\":"<<config.mesh_use_color<<"}";
    return out.str();
  }
  std::string esdfConfiguration()const{
    const auto& config=backend_->config();std::ostringstream out;
    out.precision(std::numeric_limits<double>::max_digits10);
    out<<"{\"full_euclidean_distance\":"<<(config.esdf_full_euclidean_distance?"true":"false")
       <<",\"min_distance\":"<<config.esdf_min_distance
       <<",\"max_distance\":"<<config.esdf_max_distance
       <<",\"default_distance\":"<<config.esdf_default_distance
       <<",\"min_diff\":"<<config.esdf_min_diff
       <<",\"add_occupied_crust\":"<<(config.esdf_add_occupied_crust?"true":"false")<<"}";
    return out.str();
  }
  voxblox::Transformation staticTransform(const std::string& name){
    // ROS2 parameters replace ROS1 XmlRpc matrices with a row-major array.
    const auto values=parameter<std::vector<double>>(name,{1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.,0.,0.,0.,0.,1.});
    if(values.size()!=16)throw std::invalid_argument(name+" requires 16 row-major matrix entries");
    Eigen::Matrix4d matrix;
    for(int r=0;r<4;++r)for(int c=0;c<4;++c)matrix(r,c)=values[r*4+c];
    const Eigen::Matrix3d rotation=matrix.topLeftCorner<3,3>();
    if(!matrix.allFinite()||!matrix.row(3).isApprox(Eigen::RowVector4d(0.,0.,0.,1.),1e-6)||
       !(rotation.transpose()*rotation).isApprox(Eigen::Matrix3d::Identity(),1e-6)||
       std::abs(rotation.determinant()-1.)>1e-6)
      throw std::invalid_argument(name+" must be a finite rigid transform");
    const Eigen::Quaternionf q(Eigen::Quaterniond(rotation).normalized().cast<float>());
    voxblox::Transformation result(q,matrix.topRightCorner<3,1>().cast<float>());
    return parameter<bool>("invert_"+name,false)?result.inverse():result;
  }
  void configureTransforms(){
    if(use_tf_transforms_)return;
    // Upstream has one T_B_C for its single camera. Require that same sensor
    // frame on every configured stream; independent extrinsics use TF mode.
    const auto& camera_frame=inputs_.front().frame;
    for(const auto& input:inputs_)if(input.frame!=camera_frame)
      throw std::invalid_argument("Transform-topic mode requires one sensor frame; use TF for multiple extrinsics");
    if(freespace_enabled_)for(const auto& frame:freespace_frames_)if(frame!=camera_frame)
      throw std::invalid_argument("Transform-topic freespace must use the configured sensor frame");
    T_B_D_=staticTransform("T_B_D");T_B_C_=staticTransform("T_B_C");
    transform_sub_=create_subscription<geometry_msgs::msg::TransformStamped>("transform",rclcpp::QoS(40).reliable(),
      [this](geometry_msgs::msg::TransformStamped::ConstSharedPtr message){
        const auto& t=message->transform.translation;const auto& q=message->transform.rotation;
        Eigen::Vector3d p(t.x,t.y,t.z);Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
        if(!p.allFinite()||!rotation.coeffs().allFinite()||rotation.norm()<1e-8||
           message->header.stamp.sec<0||message->header.stamp.nanosec>=1000000000u){
          ++rejected_transforms_;RCLCPP_WARN(get_logger(),"Invalid transform-topic pose");return;
        }
        rotation.normalize();
        const int64_t stamp=int64_t(message->header.stamp.sec)*1000000000LL+message->header.stamp.nanosec;
        transform_queue_->push({stamp,voxblox::Transformation(rotation.cast<float>(),p.cast<float>())});
        ++received_transforms_;
        for(std::size_t i=0;i<inputs_.size();++i)drainInput(i);
        drainFreeSpace();
      });
  }
  void configureIcp(){
    if(!parameter<bool>("enable_icp",false))return;
    voxblox::ICP::Config config;
    config.mini_batch_size=parameter<int>("icp_mini_batch_size",config.mini_batch_size);
    config.min_match_ratio=parameter<double>("icp_min_match_ratio",config.min_match_ratio);
    config.subsample_keep_ratio=parameter<double>("icp_subsample_keep_ratio",config.subsample_keep_ratio);
    config.refine_roll_pitch=parameter<bool>("icp_refine_roll_pitch",config.refine_roll_pitch);
    // Preserve the original misspelling as part of the parameter interface.
    config.inital_translation_weighting=parameter<double>("icp_inital_translation_weighting",config.inital_translation_weighting);
    config.inital_rotation_weighting=parameter<double>("icp_inital_rotation_weighting",config.inital_rotation_weighting);
    const auto threads=parameter<int>("icp_num_threads",config.num_threads?static_cast<int>(config.num_threads):1);
    if(threads<1||threads>64)throw std::invalid_argument("icp_num_threads must be between 1 and 64");
    config.num_threads=threads;
    icp_refiner_=std::make_unique<IcpRefiner>(config,parameter<bool>("accumulate_icp_corrections",true));
    icp_corrected_frame_=parameter<std::string>("icp_corrected_frame","icp_corrected");
    pose_corrected_frame_=parameter<std::string>("pose_corrected_frame","pose_corrected");
    if(icp_corrected_frame_.empty()||pose_corrected_frame_.empty()||
       icp_corrected_frame_==frame_||pose_corrected_frame_==frame_||
       pose_corrected_frame_==icp_corrected_frame_)
      throw std::invalid_argument("ICP output frames must be nonempty and distinct from the map frame");
    for(const auto& input:inputs_)if(input.frame==icp_corrected_frame_||input.frame==pose_corrected_frame_)
      throw std::invalid_argument("ICP output frames must not replace an input sensor frame");
    icp_transform_pub_=create_publisher<geometry_msgs::msg::TransformStamped>("icp_transform",rclcpp::QoS(1).reliable().transient_local());
    icp_broadcaster_=std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  void publishIcpTransforms(const voxblox::Transformation& raw,const voxblox::Transformation& correction,
                            const builtin_interfaces::msg::Time& stamp){
    auto message=[&](const voxblox::Transformation& transform,const std::string& parent,const std::string& child){
      geometry_msgs::msg::TransformStamped result;
      result.header.frame_id=parent;result.child_frame_id=child;result.header.stamp=stamp;
      const auto& p=transform.getPosition();const auto& q=transform.getRotation().toImplementation();
      result.transform.translation.x=p.x();result.transform.translation.y=p.y();result.transform.translation.z=p.z();
      result.transform.rotation.w=q.w();result.transform.rotation.x=q.x();
      result.transform.rotation.y=q.y();result.transform.rotation.z=q.z();return result;
    };
    const auto corrected=message(correction,frame_,icp_corrected_frame_);
    // Same two-link tree as upstream. The transform topic now also carries the
    // acquisition stamp, which the original ROS1 message omitted.
    icp_broadcaster_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      corrected,message(raw,icp_corrected_frame_,pose_corrected_frame_)});
    icp_transform_pub_->publish(corrected);
  }
  void configureLayerTransport(){
    publish_tsdf_layer_=parameter<bool>("publish_tsdf_map",false);
    publish_esdf_layer_=parameter<bool>("publish_esdf_map",false);
    const double interval=parameter<double>("publish_map_every_n_sec",1.0);
    if(!std::isfinite(interval)||interval<0.)
      throw std::invalid_argument("publish_map_every_n_sec must be finite and non-negative");
    // Incremental updates are volatile. Latching only the last delta would
    // silently give a late reader an incomplete map; new readers get RESET.
    const auto qos=rclcpp::QoS(10).reliable().durability_volatile();
    tsdf_layer_pub_=create_publisher<voxblox_msgs::msg::Layer>("tsdf_map_out",qos);
    esdf_layer_pub_=create_publisher<voxblox_msgs::msg::Layer>("esdf_map_out",qos);
    tsdf_layer_sub_=create_subscription<voxblox_msgs::msg::Layer>("tsdf_map_in",qos,
      [this](voxblox_msgs::msg::Layer::ConstSharedPtr message){receiveLayer(*message,false);});
    esdf_layer_sub_=create_subscription<voxblox_msgs::msg::Layer>("esdf_map_in",qos,
      [this](voxblox_msgs::msg::Layer::ConstSharedPtr message){receiveLayer(*message,true);});
    if(std::string(tsdf_layer_sub_->get_topic_name())==tsdf_layer_pub_->get_topic_name()||
       std::string(esdf_layer_sub_->get_topic_name())==esdf_layer_pub_->get_topic_name())
      throw std::invalid_argument("Layer input/output topics must not be remapped onto the same topic");
    publish_layers_service_=create_service<std_srvs::srv::Trigger>("publish_map",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        publishLayers(true,true);response->success=true;
        response->message="Published complete TSDF/ESDF layers with RESET action";
      });
    if(interval>0.)layers_timer_=create_wall_timer(std::chrono::duration<double>(interval),
      [this](){publishLayers(false);});
  }
  template<class Voxel>void publishLayer(const voxblox::Layer<Voxel>& layer,
    const rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr& publisher,
    std::set<std::string>& previous_readers,bool reset){
    std::set<std::string> readers;
    for(const auto& endpoint:get_subscriptions_info_by_topic(publisher->get_topic_name())){
      const auto& id=endpoint.endpoint_gid();
      readers.insert(std::string(id.begin(),id.end()));
    }
    for(const auto& id:readers)if(!previous_readers.count(id))reset=true;
    previous_readers=std::move(readers);
    if(previous_readers.empty())return;
    voxblox_msgs::msg::Layer message;
    serializeLayerAsMsg(layer,!reset,&message,reset?MapDeserializationAction::kReset:MapDeserializationAction::kUpdate);
    // Empty RESET is meaningful: all remote blocks must disappear.
    if(reset||!message.blocks.empty())publisher->publish(message);
    // Upstream serialization does not consume kMap. Retaining it also covers
    // ESDF propagation into previously touched neighbor blocks.
  }
  void publishLayers(bool reset,bool explicit_request=false){
    if(explicit_request)flushEsdfProducts();
    if(publish_tsdf_layer_||explicit_request)
      publishLayer(backend_->tsdfLayer(),tsdf_layer_pub_,tsdf_layer_readers_,reset);
    if((publish_esdf_layer_||explicit_request)&&backend_->esdfLayer())
      publishLayer(*backend_->esdfLayer(),esdf_layer_pub_,esdf_layer_readers_,reset);
  }
  void receiveLayer(const voxblox_msgs::msg::Layer& message,bool esdf){
    try{
      std::string error;
      if(esdf){
        if(!backend_->esdfLayer())throw std::invalid_argument("ESDF layer input requires esdf.enabled");
        if(!validateLayerMsg(message,*backend_->esdfLayer(),&error))throw std::invalid_argument(error);
        flushEsdfProducts();
        auto next=backend_->copyEsdfLayer();
        if(!deserializeMsgToLayer(message,next.get(),&error))throw std::invalid_argument(error);
        backend_->replaceEsdfLayer(std::move(next));
      }else{
        if(!validateLayerMsg(message,backend_->tsdfLayer(),&error))throw std::invalid_argument(error);
        auto next=backend_->copyTsdfLayer();
        if(!deserializeMsgToLayer(message,next.get(),&error))throw std::invalid_argument(error);
        backend_->replaceTsdfLayer(std::move(next));
        clearMesh();mesh_dirty_=true;
      }
      // UPDATE replaces supplied blocks and RESET replaces the whole layer;
      // neither can inherit the old sensor provenance. MERGE retains history
      // but still cannot count imported data as a new physical observation.
      if(message.action!=voxblox_msgs::msg::Layer::ACTION_MERGE)resetMapEpoch();
      // Layer messages carry no acquisition stamp. Imported distance products
      // must not inherit a timestamp from an unrelated local scan.
      esdf_stamp_={};esdf_latest_scan_stamp_={};esdf_pending_=false;
      esdf_input_sequence_=esdf_completed_sequence_=0;
      ++received_layers_;publish(now(),false,"import");
      // TSDF import rebuilds ESDF and can remove derived blocks. Publish a full
      // replacement so readers do not retain removed/hallucinated cells.
      publishLayers(true);
    }catch(const std::exception& error){
      ++rejected_layers_;RCLCPP_WARN(get_logger(),"%s layer rejected: %s",esdf?"ESDF":"TSDF",error.what());
    }
  }
  std::string freeSpaceStatus()const{
    std::ostringstream out;out<<"{\"enabled\":"<<(freespace_enabled_?"true":"false")
      <<",\"integrated_clouds\":"<<freespace_scans_<<",\"rejected_clouds\":"<<freespace_rejected_
      <<",\"throttled_clouds\":"<<freespace_throttled_
      <<",\"pending_clouds\":"<<freespace_pending_.size()
      <<",\"queue_dropped_clouds\":"<<freespace_queue_dropped_
      <<",\"empty_clouds\":"<<empty_freespace_<<",\"accepted_points\":"<<last_freespace_.accepted_points
      <<",\"skipped_points\":"<<last_freespace_.skipped_points<<",\"unknown_box_operations\":"<<box_clears_
      <<",\"unknown_box_cleared_voxels\":"<<cleared_voxels_<<",\"sensor_frames\":[";
    bool separator=false;for(const auto& frame:freespace_frames_){if(separator)out<<',';out<<jsonString(frame);separator=true;}
    out<<"]";
    if(freespace_sub_){
      const auto qos=freespace_sub_->get_actual_qos().get_rmw_qos_profile();
      out<<",\"subscription_depth\":"<<qos.depth
         <<",\"subscription_reliability\":"<<jsonString(reliabilityName(qos.reliability));
    }
    out<<"}";return out.str();
  }
  void configureFreeSpace(){
    freespace_enabled_=parameter<bool>("use_freespace_pointcloud",false);
    std::vector<std::string> frames;
    for(const auto& input:inputs_)frames.push_back(input.frame);
    frames=parameter<std::vector<std::string>>("freespace.sensor_frames",frames);
    for(const auto& frame:frames){
      if(frame.empty())throw std::invalid_argument("Empty freespace sensor frame");
      freespace_frames_.insert(frame);
    }
    if(freespace_enabled_&&freespace_frames_.empty())throw std::invalid_argument("Freespace input needs sensor frames");
    if(freespace_enabled_)freespace_sub_=create_subscription<sensor_msgs::msg::PointCloud2>(
      "freespace_pointcloud",pointCloudQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message){receiveFreeSpace(std::move(message));});
    if(freespace_sub_)for(const auto& input:inputs_)if(input.topic==freespace_sub_->get_topic_name())
      throw std::invalid_argument("Freespace and obstacle-return input topics must differ");
    startup_box_=parameter<bool>("clear_unknown_box_on_start",false);
    odometry_timeout_=parameter<double>("odometry_timeout",2.0);
    const auto size=parameter<std::vector<double>>("robot.size",{.4,.4,.4});
    const auto offset=parameter<std::vector<double>>("robot.center_offset",{0.,0.,0.});
    if(size.size()!=3||offset.size()!=3||!std::isfinite(odometry_timeout_)||odometry_timeout_<=0.)
      throw std::invalid_argument("Invalid robot-clear parameters");
    robot_size_={size[0],size[1],size[2]};robot_offset_={offset[0],offset[1],offset[2]};
    if(!robot_size_.allFinite()||(robot_size_.array()<=0.).any()||!robot_offset_.allFinite())
      throw std::invalid_argument("Invalid robot-clear geometry");
    odometry_sub_=create_subscription<nav_msgs::msg::Odometry>("odometry",10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message){
        const auto& pose=message->pose.pose;const auto& q=pose.orientation;
        const Eigen::Vector3d position(pose.position.x,pose.position.y,pose.position.z);
        const Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
        const double stamp=rclcpp::Time(message->header.stamp).seconds(),age=now().seconds()-stamp;
        odometry_valid_=message->header.frame_id==frame_&&position.allFinite()&&rotation.coeffs().allFinite()&&
          rotation.norm()>1e-8&&stamp>0.&&age>=-.1&&age<=odometry_timeout_;
        if(!odometry_valid_)return;
        robot_position_=position;odometry_stamp_=stamp;
        if(clear_sphere_pose_source_=="odometry"&&backend_->config().esdf_enabled&&
           backend_->config().esdf_clear_sphere_for_planning){
          try{
            backend_->addNewRobotPosition(position);++esdf_clear_sphere_operations_;
            publish(message->header.stamp,false,"augmentation");
          }catch(const std::exception& error){
            RCLCPP_WARN(get_logger(),"ESDF planning clear sphere: %s",error.what());
          }
        }
        if(startup_box_&&!startup_box_done_){
          try{clearRobotUnknownBox();startup_box_done_=true;}
          catch(const std::exception& error){RCLCPP_WARN(get_logger(),"Startup robot box: %s",error.what());}
        }
      });
    clear_box_service_=create_service<std_srvs::srv::Trigger>("clear_robot_unknown_box",
      [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> response){
        try{const auto count=clearRobotUnknownBox();response->success=true;
          response->message="{\"cleared_voxels\":"+std::to_string(count)+",\"known_cells\":"+std::to_string(known_)+"}";
        }catch(const std::exception& error){response->success=false;response->message=error.what();}
      });
    using AugmentFreeBox=planner_msgs::srv::AugmentFreeBox;
    augment_box_service_=create_service<AugmentFreeBox>("augment_free_box",
      [this](std::shared_ptr<AugmentFreeBox::Request> request,
             std::shared_ptr<AugmentFreeBox::Response> response){
        // The single-threaded mapper serializes this mutation with scan
        // integration and publication; the returned id identifies one commit.
        response->source_id=map_source_id_;response->map_epoch=map_epoch_;response->update_id=map_update_id_;
        try{
          if(request->header.frame_id!=frame_)
            throw std::invalid_argument("Augmentation frame must equal mapper frame");
          if(request->header.stamp.sec<0||request->header.stamp.nanosec>=1000000000u||
             (request->header.stamp.sec==0&&request->header.stamp.nanosec==0))
            throw std::invalid_argument("Augmentation requires an explicit valid pose stamp");
          const Eigen::Vector3d center(request->center.x,request->center.y,request->center.z);
          const Eigen::Vector3d size(request->size.x,request->size.y,request->size.z);
          const auto count=backend_->clearUnknownBox(center,size);
          ++box_clears_;cleared_voxels_+=count;response->changed_voxels=count;
          if(count||request->return_snapshot){
            if(count)mesh_dirty_=true;
            auto snapshot=publish(now(),false,count?"augmentation":"snapshot",request->return_snapshot);
            if(snapshot)response->snapshot=std::move(*snapshot);
            if(count)publishLayers(true);
          }
          response->source_id=map_source_id_;response->map_epoch=map_epoch_;response->update_id=map_update_id_;
          response->success=true;response->reason=count?"ok":"no_change";
        }catch(const std::exception& error){
          response->success=false;response->reason=error.what();
        }
      });
  }
  std::size_t clearRobotUnknownBox(){
    const double age=now().seconds()-odometry_stamp_;
    if(!odometry_valid_||age<-.1||age>odometry_timeout_)throw std::runtime_error("Fresh map-frame odometry required");
    const auto count=backend_->clearUnknownBox(robot_position_+robot_offset_,robot_size_);
    ++box_clears_;cleared_voxels_+=count;
    if(count){mesh_dirty_=true;publish(now(),false,"augmentation");publishLayers(true);}
    return count;
  }
  void configureInputs(){
    const auto names=parameter<std::vector<std::string>>("input_sensors",{});
    if(names.size()>16)throw std::invalid_argument("At most 16 input sensors are supported");
    if(names.empty()){
      if(sensor_frame_.empty())throw std::invalid_argument("Empty sensor frame name");
      inputs_.push_back({"legacy","points",sensor_frame_});
    }else{
      std::set<std::string> seen;
      for(const auto& name:names){
        if(name.empty()||!seen.insert(name).second||
           name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_")!=std::string::npos)
          throw std::invalid_argument("Invalid or duplicate input sensor name");
        auto topic=parameter<std::string>("input."+name+".topic","");
        auto frame=parameter<std::string>("input."+name+".frame","");
        if(topic.empty()||frame.empty())throw std::invalid_argument("Each input sensor needs an explicit topic and frame");
        inputs_.push_back({name,topic,frame});
      }
    }
    // One serialized mapper integrates every source into the same backend.
    // Resolve names through ROS first so aliases/remaps cannot subscribe twice
    // to one stream and count/fuse a cloud more than once.
    std::set<std::string> resolved_topics;
    for(std::size_t i=0;i<inputs_.size();++i){
      auto subscription=create_subscription<sensor_msgs::msg::PointCloud2>(
        inputs_[i].topic,pointCloudQoS(),
        [this,i](sensor_msgs::msg::PointCloud2::ConstSharedPtr message){receive(std::move(message),i);});
      const std::string resolved=subscription->get_topic_name();
      if(!resolved_topics.insert(resolved).second)throw std::invalid_argument("Input sensor topics must be unique after ROS name resolution");
      inputs_[i].topic=resolved;cloud_subscriptions_.push_back(std::move(subscription));
    }
  }
  void receive(sensor_msgs::msg::PointCloud2::ConstSharedPtr message,std::size_t source_index){
    const SnapshotTraceEnd returned{this,"scan_receive_returning",message->header.stamp,0,0,int(source_index)};
    traceSnapshot("scan_received",message->header.stamp,message->data.size(),0,int(source_index));
    auto& source=inputs_.at(source_index);
    const int64_t stamp_ns=int64_t(message->header.stamp.sec)*1000000000LL+message->header.stamp.nanosec;
    // The upstream guard uses acquisition time and a strict greater-than
    // comparison. It updates before TF lookup, so a message waiting for TF
    // still advances the guard and is not admitted a second time.
    const int64_t previous_stamp=source.has_last_accepted?source.last_stamp_ns:0;
    if(!(stamp_ns-previous_stamp>min_time_between_msgs_ns_)){
      ++source.throttled;
      RCLCPP_DEBUG(get_logger(),"Scan throttled (%s): acquisition interval %.6f s",
        source.name.c_str(),(stamp_ns-source.last_stamp_ns)*1e-9);
      drainInput(source_index);
      return;
    }
    source.has_last_accepted=true;source.last_stamp_ns=stamp_ns;
    source.pending.push_back(std::move(message));
    traceSnapshot("scan_queued",source.pending.back()->header.stamp,0,0,int(source_index));
    drainInput(source_index);
  }
  void drainInput(std::size_t source_index){
    auto& source=inputs_.at(source_index);
    while(!source.pending.empty()){
      const auto& message=source.pending.front();
      try{
        if(message->header.frame_id!=source.frame)throw std::invalid_argument("Wrong sensor frame");
        traceSnapshot("scan_integrate_begin",message->header.stamp,message->data.size(),0,int(source_index));
        last_=integrateMessage(*message,false);++scans_;++source.integrated;mesh_dirty_=true;
        traceSnapshot("scan_integrate_end",message->header.stamp,message->data.size(),0,int(source_index));
        queueObservation(message->header.stamp);
        RCLCPP_INFO(get_logger(),"Integrated scan %d: %zu returns, %zu last-published known voxels",scans_,last_.accepted_points,known_);
        source.pending.pop_front();
      }catch(const tf2::TransformException& error){
        traceSnapshot("scan_waiting_tf",message->header.stamp,0,0,int(source_index),error.what());
        // Keep the queue head until its timestamped transform becomes
        // available, matching TsdfServer::getNextPointcloudFromQueue().
        RCLCPP_DEBUG(get_logger(),"Waiting for TF for %s cloud: %s",source.name.c_str(),error.what());
        while(source.pending.size()>=kInternalQueueLimit){
          source.pending.pop_front();++source.queue_dropped;
        }
        return;
      }catch(const std::exception& error){
        ++rejected_;++source.rejected;source.pending.pop_front();
        RCLCPP_WARN(get_logger(),"Scan rejected (%s): %s",source.name.c_str(),error.what());
      }
    }
  }
  void receiveFreeSpace(sensor_msgs::msg::PointCloud2::ConstSharedPtr message){
    const int64_t stamp_ns=int64_t(message->header.stamp.sec)*1000000000LL+message->header.stamp.nanosec;
    try{
      const int64_t previous_stamp=freespace_has_last_accepted_?freespace_last_stamp_ns_:0;
      if(!(stamp_ns-previous_stamp>min_time_between_msgs_ns_)){
        ++freespace_throttled_;drainFreeSpace();return;
      }
      // Free-space has an independent acquisition-time baseline in ROS1.
      freespace_has_last_accepted_=true;freespace_last_stamp_ns_=stamp_ns;
      // A sensor whose entire view is blocked produces no clearing endpoints.
      if(message->width==0&&message->height==1&&message->data.empty()&&message->row_step==0){++empty_freespace_;return;}
      freespace_pending_.push_back(std::move(message));
      drainFreeSpace();
    }catch(const std::exception& error){++freespace_rejected_;RCLCPP_WARN(get_logger(),"Freespace cloud rejected: %s",error.what());}
  }
  void drainFreeSpace(){
    while(!freespace_pending_.empty()){
      const auto& message=freespace_pending_.front();
      try{
        if(!freespace_frames_.count(message->header.frame_id))throw std::invalid_argument("Unconfigured freespace frame");
        last_freespace_=integrateMessage(*message,true);++freespace_scans_;mesh_dirty_=true;publish(message->header.stamp,false,"freespace");
        freespace_pending_.pop_front();
      }catch(const tf2::TransformException& error){
        RCLCPP_DEBUG(get_logger(),"Waiting for TF for freespace cloud: %s",error.what());
        while(freespace_pending_.size()>=kInternalQueueLimit){
          freespace_pending_.pop_front();++freespace_queue_dropped_;
        }
        return;
      }catch(const std::exception& error){
        ++freespace_rejected_;freespace_pending_.pop_front();
        RCLCPP_WARN(get_logger(),"Freespace cloud rejected: %s",error.what());
      }
    }
  }
  static uint32_t readU32(const uint8_t* bytes,bool big_endian){
    if(big_endian)return (uint32_t(bytes[0])<<24)|(uint32_t(bytes[1])<<16)|(uint32_t(bytes[2])<<8)|uint32_t(bytes[3]);
    return uint32_t(bytes[0])|(uint32_t(bytes[1])<<8)|(uint32_t(bytes[2])<<16)|(uint32_t(bytes[3])<<24);
  }
  static float readFloat32(const uint8_t* bytes,bool big_endian){
    const uint32_t bits=readU32(bytes,big_endian);float value;std::memcpy(&value,&bits,sizeof(value));return value;
  }
  static voxblox::Color readPackedColor(const uint8_t* bytes,bool big_endian){
    // PCL packs 0xAARRGGBB (RGBA) or 0x00RRGGBB (RGB), even when
    // the carrier datatype is FLOAT32. Surface opacity is not sensor color.
    const uint32_t packed=readU32(bytes,big_endian);
    return voxblox::Color(static_cast<uint8_t>((packed>>16)&0xff),
                          static_cast<uint8_t>((packed>>8)&0xff),
                          static_cast<uint8_t>(packed&0xff));
  }
  TsdfScanResult integrateMessage(const sensor_msgs::msg::PointCloud2& message,bool freespace){
      if(message.header.frame_id.empty()||message.height==0||message.point_step==0)
        throw std::invalid_argument("Sensor frame or shape invalid");
      if(message.header.stamp.sec==0 && message.header.stamp.nanosec==0)
        throw std::invalid_argument("Cloud timestamp must be explicit; zero would request latest TF");
      if(message.width==0||uint64_t(message.width)*message.height>200000 ||
         message.row_step<uint64_t(message.width)*message.point_step||
         message.data.size()!=uint64_t(message.row_step)*message.height)
        throw std::invalid_argument("Cloud data size invalid");
      std::set<uint32_t> field_bytes;
      auto field=[&](const std::string& name){
        const sensor_msgs::msg::PointField* result=nullptr;
        for(const auto& f:message.fields)if(f.name==name){if(result)throw std::invalid_argument("Duplicate cloud field");result=&f;}
        if(!result||result->datatype!=sensor_msgs::msg::PointField::FLOAT32||result->count!=1||
           uint64_t(result->offset)+4>message.point_step)throw std::invalid_argument("Cloud requires float32 XYZ");
        for(uint32_t b=result->offset;b<result->offset+4;++b)
          if(!field_bytes.insert(b).second)throw std::invalid_argument("Overlapping cloud fields");
        return result->offset;
      };
      auto ox=field("x"),oy=field("y"),oz=field("z");
      const sensor_msgs::msg::PointField* color_field=nullptr;
      if(backend_->config().use_color){
        for(const auto& candidate:message.fields){
          if(candidate.name!="rgb"&&candidate.name!="rgba")continue;
          if(color_field)throw std::invalid_argument("Cloud contains both rgb and rgba fields");
          color_field=&candidate;
        }
        if(color_field&&color_field->count!=1){
          throw std::invalid_argument("Packed RGB/RGBA field must have count one");
        }
        if(color_field&&color_field->datatype!=sensor_msgs::msg::PointField::FLOAT32&&
           color_field->datatype!=sensor_msgs::msg::PointField::UINT32&&
           color_field->datatype!=sensor_msgs::msg::PointField::INT32)
          throw std::invalid_argument("Packed RGB/RGBA field must be float32, uint32 or int32");
        if(color_field&&uint64_t(color_field->offset)+4>message.point_step)
          throw std::invalid_argument("Packed RGB/RGBA field exceeds point step");
        if(color_field)for(uint32_t b=color_field->offset;b<color_field->offset+4;++b)
          if(!field_bytes.insert(b).second)throw std::invalid_argument("Overlapping cloud fields");
      }
      const bool have_colors=color_field!=nullptr&&!freespace;
      // Each source uses its own acquisition timestamp, not a synchronized
      // bundle or the newest timestamp of another sensor. An older interleaved
      // cloud remains valid if TF has that time; the snapshot retains its stamp.
      Eigen::Vector3d origin;Eigen::Quaterniond rotation;
      if(use_tf_transforms_){
        const auto transform=tf_buffer_->lookupTransform(frame_,message.header.frame_id,rclcpp::Time(message.header.stamp));
        const auto& t=transform.transform.translation;const auto& q=transform.transform.rotation;
        origin={t.x,t.y,t.z};rotation=Eigen::Quaterniond(q.w,q.x,q.y,q.z);
      }else{
        voxblox::Transformation dynamic;
        const int64_t stamp=int64_t(message.header.stamp.sec)*1000000000LL+message.header.stamp.nanosec;
        if(!transform_queue_->lookup(stamp,&dynamic))
          throw tf2::ExtrapolationException("Waiting for transform-topic samples around acquisition time");
        const auto transform=sensorTransform(dynamic,T_B_D_,T_B_C_);
        origin=transform.getPosition().cast<double>();
        rotation=transform.getRotation().toImplementation().cast<double>();
      }
      if(!rotation.coeffs().allFinite()||rotation.norm()<1e-8)throw std::invalid_argument("Invalid sensor transform");
      rotation.normalize();std::vector<Eigen::Vector3d> endpoints;
      endpoints.reserve(uint64_t(message.width)*message.height);
      std::vector<voxblox::Color> colors;
      if(have_colors)colors.reserve(uint64_t(message.width)*message.height);
      for(uint32_t row=0;row<message.height;++row)for(uint32_t col=0;col<message.width;++col){
        const auto* data=message.data.data()+uint64_t(row)*message.row_step+uint64_t(col)*message.point_step;
        const float x=readFloat32(data+ox,message.is_bigendian);
        const float y=readFloat32(data+oy,message.is_bigendian);
        const float z=readFloat32(data+oz,message.is_bigendian);
        endpoints.emplace_back(x,y,z);
        if(have_colors)colors.emplace_back(readPackedColor(data+color_field->offset,message.is_bigendian));
      }
      Eigen::Vector3d integration_origin=origin;Eigen::Quaterniond integration_rotation=rotation;
      if(icp_refiner_){
        voxblox::Pointcloud points;points.reserve(endpoints.size());
        // The original PCL conversion removes nonfinite points before ICP;
        // range filtering remains the TSDF integrator's responsibility.
        for(const auto& point:endpoints)if(point.allFinite())points.push_back(point.cast<float>());
        const voxblox::Transformation raw(voxblox::Rotation(rotation.cast<float>()),origin.cast<float>());
        const auto refinement=icp_refiner_->refine(backend_->tsdfLayer(),points,raw);
        ++icp_runs_;icp_last_batches_=refinement.successful_batches;icp_batches_+=icp_last_batches_;
        integration_origin=refinement.refined.getPosition().cast<double>();
        integration_rotation=refinement.refined.getRotation().toImplementation().cast<double>();
        publishIcpTransforms(raw,refinement.correction,message.header.stamp);
      }
      if(voxel_scan_capture_)voxel_scan_capture_->captureBefore(*backend_);
      const auto result=freespace?backend_->integrateFreeSpace(integration_origin,integration_rotation,endpoints):
        (have_colors?backend_->integrate(integration_origin,integration_rotation,endpoints,colors):
                     backend_->integrate(integration_origin,integration_rotation,endpoints));
      esdf_latest_scan_stamp_=message.header.stamp;
      if(backend_->esdfLayer()){
        ++esdf_input_sequence_;
        esdf_pending_=esdf_update_period_>0.;
        if(!esdf_pending_){esdf_stamp_=message.header.stamp;esdf_completed_sequence_=esdf_input_sequence_;++esdf_updates_;}
      }
      if(voxel_scan_capture_)voxel_scan_capture_->captureAfter(*backend_,message,origin,rotation,
        integration_origin,integration_rotation,freespace,++voxel_capture_scan_sequence_);
      // Pruning uses the raw pose even when ICP refines the integration pose.
      // ESDF propagation, when requested, drains synchronously. In periodic
      // mode only dirty TSDF flags remain pending; no worker/ESDF queue is live.
      voxblox::BlockIndexList removed_tsdf;
      const auto removed_tsdf_count=backend_->removeDistantTsdfBlocks(origin,max_block_distance_,&removed_tsdf);
      const auto removed_mesh_count=pruneMesh(origin,removed_tsdf);
      // EsdfServer::newPoseCallback receives T_G_C after a successful scan.
      // Odometry-driven augmentation is retained only as an explicit ROS2 mode.
      if(clear_sphere_pose_source_=="sensor"&&backend_->config().esdf_enabled&&
         backend_->config().esdf_clear_sphere_for_planning){
        backend_->addNewRobotPosition(origin);++esdf_clear_sphere_operations_;
      }
      const auto removed_esdf_count=backend_->removeDistantEsdfBlocks(origin,max_block_distance_);
      pruned_tsdf_blocks_+=removed_tsdf_count;pruned_esdf_blocks_+=removed_esdf_count;
      pruned_mesh_blocks_+=removed_mesh_count;
      if(removed_tsdf_count||removed_esdf_count||removed_mesh_count){
        // Layer has no per-block deletion opcode. A complete replacement also
        // removes remote blocks, including when the retained layer is empty.
        publishLayers(true);visualization_refresh_needed_=true;
      }
      return result;
  }
  std::size_t pruneMesh(const Eigen::Vector3d& position,const voxblox::BlockIndexList& removed_tsdf){
    const auto before=mesh_layer_->getNumberOfAllocatedMeshes();
    if(!before)return 0;
    bool changed=false;
    voxblox::BlockIndexList cached;mesh_layer_->getAllAllocatedMeshes(&cached);
    std::set<std::array<int,3>> cached_keys;
    for(const auto& index:cached){
      if(!removed_tsdf.empty())cached_keys.insert({index.x(),index.y(),index.z()});
      if((mesh_layer_->getMeshByIndex(index).origin-position.cast<float>()).squaredNorm()>
         double(max_block_distance_)*max_block_distance_)changed=true;
    }
    if(!changed&&removed_tsdf.empty())return 0;
    // Original MeshLayer keeps empty blocks until their deletion is transmitted.
    mesh_layer_->clearDistantMesh(position.cast<float>(),max_block_distance_);
    voxblox::BlockIndexList neighbors;
    std::set<std::array<int,3>> seen;
    for(const auto& removed:removed_tsdf)for(int z=0;z<=1;++z)for(int y=0;y<=1;++y)for(int x=0;x<=1;++x){
      if(!(x||y||z))continue;
      const auto minimum=std::numeric_limits<voxblox::IndexElement>::min();
      if((x&&removed.x()==minimum)||(y&&removed.y()==minimum)||(z&&removed.z()==minimum))continue;
      const voxblox::BlockIndex index=removed-voxblox::BlockIndex(x,y,z);
      if(backend_->tsdfLayer().hasBlock(index)&&cached_keys.count({index.x(),index.y(),index.z()})&&
         seen.insert({index.x(),index.y(),index.z()}).second)neighbors.push_back(index);
    }
    if(!neighbors.empty()){
      // Boundary triangles belong to the lower block. Refresh only previously
      // displayed neighbors, even when periodic meshing is disabled, so deleted
      // TSDF support cannot leave cached triangles behind.
      backend_->updateMeshBlocks(mesh_layer_.get(),neighbors);changed=true;
    }
    if(changed){
      publishMeshBlocks(true);
      voxblox::Mesh mesh;voxblox::convertMeshLayerToMesh(*mesh_layer_,&mesh,true);
      publishMeshMarker(mesh);
    }
    return before-mesh_layer_->getNumberOfAllocatedMeshes();
  }
  void clearMesh(){
    // The original Mesh protocol has no RESET action. Each previously shown
    // block needs its own empty update or a receiver retains the old surface.
    voxblox::BlockIndexList indices;mesh_layer_->getAllAllocatedMeshes(&indices);
    voxblox_msgs::msg::Mesh deleted;deleted.header.frame_id=frame_;deleted.header.stamp=now();
    deleted.block_edge_length=mesh_layer_->block_size();
    for(const auto& index:indices){
      voxblox_msgs::msg::MeshBlock block;
      for(int axis=0;axis<3;++axis)block.index[axis]=index[axis];
      deleted.mesh_blocks.push_back(std::move(block));
    }
    if(!deleted.mesh_blocks.empty())mesh_blocks_pub_->publish(deleted);
    mesh_layer_->clear();mesh_full_rebuild_=true;
    visualization_msgs::msg::Marker marker;marker.header.frame_id=frame_;marker.header.stamp=now();
    marker.ns="voxblox_surface";marker.id=0;marker.action=visualization_msgs::msg::Marker::DELETE;
    mesh_pub_->publish(marker);mesh_dirty_=false;
  }
  void publishMeshBlocks(bool only_updated){
    voxblox_msgs::msg::Mesh message;message.header.frame_id=frame_;message.header.stamp=now();
    generateVoxbloxMeshMsg(mesh_layer_.get(),mesh_color_mode_,&message,only_updated);
    if(!message.mesh_blocks.empty())mesh_blocks_pub_->publish(message);
  }
  std::string generateMesh(const std::string& file,bool only_updated=false){
    const auto mesh_stamp=now();
    traceSnapshot("mesh_generate_begin",mesh_stamp);
    SnapshotTraceEnd returned{this,"mesh_generate_end",mesh_stamp};
    // A load/transport update can preserve TSDF blocks whose kMesh flag was
    // consumed earlier. Clearing their cached meshes requires one full pass.
    const bool incremental=only_updated&&!mesh_full_rebuild_;
    traceSnapshot("mesh_update_begin",mesh_stamp);
    backend_->updateMeshLayer(mesh_layer_.get(),incremental);
    traceSnapshot("mesh_update_end",mesh_stamp);
    // Marker and PLY are complete connected meshes, while the original Mesh
    // topic sends block updates from this same persistent Marching Cubes layer.
    voxblox::Mesh mesh;voxblox::convertMeshLayerToMesh(*mesh_layer_,&mesh,true);
    if(!file.empty())backend_->saveMesh(file,mesh);
    publishMeshBlocks(incremental);
    publishMeshMarker(mesh);mesh_dirty_=false;mesh_full_rebuild_=false;++mesh_updates_;
    if(only_updated&&publish_pointclouds_&&!publish_pointclouds_on_update_)
      // A periodic ESDF already publishes its own completed layer on the ROS
      // clock. The wall-clock mesh timer refreshes TSDF products, without
      // traversing and resending that same (possibly older) ESDF a second time.
      publishDistanceProducts(now(),true,esdf_update_period_==0.);
    return "{\"vertices\":"+std::to_string(mesh.vertices.size())+",\"triangles\":"+
      std::to_string(mesh.indices.size()/3)+"}";
  }
  void publishMeshMarker(const voxblox::Mesh& mesh){
    visualization_msgs::msg::Marker marker;marker.header.frame_id=frame_;marker.header.stamp=now();
    marker.ns="voxblox_surface";marker.id=0;marker.type=visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action=mesh.indices.empty()?visualization_msgs::msg::Marker::DELETE:visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w=1.;marker.scale.x=marker.scale.y=marker.scale.z=1.;
    marker.color.r=.25;marker.color.g=.65;marker.color.b=.85;marker.color.a=1.;
    marker.points.reserve(mesh.indices.size());
    marker.colors.reserve(mesh.indices.size());
    for(auto index:mesh.indices){const auto& vertex=mesh.vertices.at(index);geometry_msgs::msg::Point p;
      p.x=vertex.x();p.y=vertex.y();p.z=vertex.z();marker.points.push_back(p);
      const auto color=mesh_message_detail::vertexColor(mesh,mesh_color_mode_,index);
      std_msgs::msg::ColorRGBA rgba;rgba.r=color[0];rgba.g=color[1];rgba.b=color[2];rgba.a=1.f;
      marker.colors.push_back(rgba);
    }
    mesh_pub_->publish(marker);
  }
  void resetMapEpoch(){
    // Keep the process identity and publication sequence comparable across
    // resets. A late older epoch cannot roll back a newer service response.
    ++map_epoch_;map_observation_id_=0;
    map_observation_stamp_={};
    pending_observation_scans_=0;pending_observation_stamp_={};
    snapshot_point_counts_={};
    esdf_point_count_=0;
    esdf_pending_=false;esdf_stamp_={};esdf_latest_scan_stamp_={};
    esdf_input_sequence_=esdf_completed_sequence_=0;
  }
  void updatePendingEsdf(){
    if(!esdf_pending_)return;
    const auto stamp=esdf_latest_scan_stamp_;
    traceSnapshot("esdf_update_begin",stamp);
    backend_->updateEsdf();
    esdf_stamp_=stamp;esdf_pending_=false;esdf_completed_sequence_=esdf_input_sequence_;++esdf_updates_;
    traceSnapshot("esdf_update_end",stamp);
  }
  void flushEsdfProducts(){
    if(!esdf_pending_)return;
    updatePendingEsdf();publishEsdfVoxels();publishEsdfProducts(publish_pointclouds_,esdf_stamp_);
  }
  void publishEsdfVoxels(){
    if(!backend_->esdfLayer())return;
    traceSnapshot("esdf_voxels_generate_begin",esdf_stamp_);
    SnapshotTraceEnd returned{this,"esdf_voxels_generate_end",esdf_stamp_};
    auto cloud=distance_product_detail::emptyCloud();
    cloud.fields[3].name="distance";
    cloud.header.frame_id=frame_;cloud.header.stamp=esdf_stamp_;
    PointcloudBuffer<16> buffer(cloud.data,esdf_point_count_);
    auto emit=[&](const Eigen::Vector3d& center,const voxblox::EsdfVoxel* voxel){
      if(!voxel||!voxel->observed||!std::isfinite(voxel->distance))return;
      const float values[]={float(center.x()),float(center.y()),float(center.z()),voxel->distance};
      buffer.append(values);
    };
    const auto& config=backend_->config();
    if(storage_=="sparse"){
      voxblox::BlockIndexList blocks;backend_->esdfLayer()->getAllAllocatedBlocks(&blocks);
      for(const auto& index:blocks){
        const auto& block=backend_->esdfLayer()->getBlockByIndex(index);
        for(std::size_t i=0;i<block.num_voxels();++i)
          emit(block.computeCoordinatesFromLinearIndex(i).cast<double>(),&block.getVoxelByLinearIndex(i));
      }
    }else{
      for(int z=0;z<config.dimensions.z();++z)for(int y=0;y<config.dimensions.y();++y)for(int x=0;x<config.dimensions.x();++x){
        const Eigen::Vector3d center=config.origin+config.resolution*Eigen::Vector3d(x+.5,y+.5,z+.5);
        emit(center,backend_->esdfVoxel(center));
      }
    }
    esdf_point_count_=buffer.finish();cloud.width=esdf_point_count_;cloud.row_step=cloud.width*cloud.point_step;
    returned.bytes=cloud.data.size();
    esdf_pub_->publish(cloud);
  }
  void publishEsdfProducts(bool full_products,const builtin_interfaces::msg::Time& stamp){
    if(!backend_->esdfLayer())return;
    traceSnapshot("esdf_products_begin",stamp);
    SnapshotTraceEnd returned{this,"esdf_products_end",stamp};
    auto send=[&](sensor_msgs::msg::PointCloud2 cloud,
                  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher){
      returned.bytes+=cloud.data.size();
      cloud.header.frame_id=frame_;cloud.header.stamp=stamp;publisher->publish(cloud);
    };
    if(full_products)send(createDistancePointcloudFromEsdfLayer(*backend_->esdfLayer()),esdf_cloud_pub_);
    if(publish_slices_)send(createDistancePointcloudFromEsdfLayerSlice(
      *backend_->esdfLayer(),2,slice_level_),esdf_slice_pub_);
    if(publish_traversable_)send(createFreePointcloudFromEsdfLayer(
      *backend_->esdfLayer(),traversability_radius_),traversable_pub_);
  }
  void queueObservation(const builtin_interfaces::msg::Time& stamp){
    // Integration is never throttled here. Equal or regressing stamps from
    // independent sensors remain real observations; steady time alone decides
    // when their current full map should be serialized.
    ++pending_observation_scans_;pending_observation_stamp_=stamp;
    traceSnapshot("observation_queued",stamp);
    if(snapshot_publish_period_==0.)publish(stamp,false,"observation");
    else flushPendingObservation();
    traceSnapshot("observation_queue_returned",stamp);
  }
  void flushPendingObservation(){
    if(!pending_observation_scans_)return;
    if(last_snapshot_publish_steady_!=std::chrono::steady_clock::time_point{}&&
       std::chrono::duration<double>(std::chrono::steady_clock::now()-last_snapshot_publish_steady_).count()
         <snapshot_publish_period_)return;
    // Capture by value: publish consumes and resets pending_observation_stamp_.
    const auto trace_stamp=pending_observation_stamp_;
    traceSnapshot("flush_ready",trace_stamp);
    publish(pending_observation_stamp_,false,"observation");
    traceSnapshot("flush_returned",trace_stamp);
  }
  std::optional<planner_msgs::msg::VoxelMapUpdate> publish(
      const builtin_interfaces::msg::Time& stamp,bool explicit_products=false,
      const std::string& change_kind="snapshot",bool return_snapshot=false){
    if(explicit_products)updatePendingEsdf();
    // These mutations synchronously replace/rebuild the distance field. They
    // also need to replace cached ESDF products even if no later scan arrives.
    const bool replaced_esdf=change_kind=="reset"||change_kind=="load"||change_kind=="import"||change_kind=="augmentation";
    if(esdf_update_period_>0.&&change_kind=="augmentation"){
      esdf_pending_=false;esdf_stamp_=esdf_latest_scan_stamp_;
      esdf_completed_sequence_=esdf_input_sequence_;++esdf_updates_;
    }
    const auto trace_sequence=snapshot_trace_.is_open()?++snapshot_trace_sequence_:0;
    // Declared before the large local buffers: its destructor runs AFTER their
    // destruction. The stamp is copied because the pending member is reset.
    SnapshotTraceEnd returned{this,"snapshot_locals_destroyed",stamp,0,trace_sequence};
    if(snapshot_trace_.is_open()){snapshot_trace_stamp_=stamp;snapshot_trace_bytes_=0;}
    traceSnapshot("snapshot_build_begin",returned.stamp,0,trace_sequence);
    auto cloud=[&](const std::vector<std::string>& fields){
      sensor_msgs::msg::PointCloud2 result;result.header.frame_id=frame_;result.header.stamp=stamp;
      result.height=1;result.is_dense=true;result.point_step=fields.size()*4;
      for(std::size_t i=0;i<fields.size();++i){sensor_msgs::msg::PointField field;
        field.name=fields[i];field.offset=i*4;field.datatype=sensor_msgs::msg::PointField::FLOAT32;field.count=1;
        result.fields.push_back(field);}return result;
    };
    auto occupancy=cloud({"x","y","z","occupancy","tsdf_distance","tsdf_weight"});
    occupancy.fields[3].datatype=sensor_msgs::msg::PointField::INT8;
    auto tsdf=cloud({"x","y","z","distance","weight"});auto esdf=cloud({"x","y","z","distance"});
    PointcloudBuffer<24> occupancy_buffer(occupancy.data,snapshot_point_counts_[0]);
    PointcloudBuffer<20> tsdf_buffer(tsdf.data,snapshot_point_counts_[1]);
    PointcloudBuffer<16> esdf_buffer(esdf.data,snapshot_point_counts_[2]);
    known_=0;
    auto emit=[&](const Eigen::Vector3d& center,VoxelStatus state,
                  const voxblox::TsdfVoxel* voxel,const voxblox::EsdfVoxel* distance){
      float values[6]={float(center.x()),float(center.y()),float(center.z()),0.f,0.f,0.f};
      if(state!=VoxelStatus::Unknown)++known_;
      // ESDF and TSDF have different observed sets. Ship their union rather
      // than discarding TSDF-only cells or inventing distance for ESDF-only cells.
      if(state!=VoxelStatus::Unknown||voxel||distance){
        if(voxel){values[4]=voxel->distance;values[5]=voxel->weight;}
        auto* record=occupancy_buffer.append(values);
        const int8_t value=static_cast<int8_t>(state);
        std::memcpy(record+12,&value,1);
      }
      if(voxel){
        values[3]=voxel->distance;values[4]=voxel->weight;tsdf_buffer.append(values);
      }
      if(distance&&esdf_update_period_==0.){
        values[3]=distance->distance;esdf_buffer.append(values);
      }
    };
    if(storage_=="sparse"){
      backend_->forEachObservedVoxel([&](const Eigen::Vector3i& index,VoxelStatus state,
        const voxblox::TsdfVoxel* observed_tsdf,const voxblox::EsdfVoxel* observed_esdf){
        const Eigen::Vector3d center=(index.cast<double>().array()+.5).matrix()*backend_->config().resolution;
        emit(center,state,observed_tsdf,observed_esdf);
      });
    }else{
      const auto map=backend_->snapshot();
      for(int z=0;z<map->dimensions().z();++z)for(int y=0;y<map->dimensions().y();++y)for(int x=0;x<map->dimensions().x();++x){
        const auto center=map->center({x,y,z});const auto state=map->voxel(center);
        const auto* voxel=backend_->voxel(center);
        if(TsdfBackend::classify(voxel,backend_->config().resolution,
                                backend_->config().occupancy_factor)==VoxelStatus::Unknown)voxel=nullptr;
        const auto* distance=backend_->esdfVoxel(center);
        if(distance&&(!distance->observed||!std::isfinite(distance->distance)))distance=nullptr;
        emit(center,state,voxel,distance);
      }
    }
    snapshot_point_counts_={occupancy_buffer.finish(),tsdf_buffer.finish(),esdf_buffer.finish()};
    occupancy.width=snapshot_point_counts_[0];tsdf.width=snapshot_point_counts_[1];esdf.width=snapshot_point_counts_[2];
    for(auto* message:{&occupancy,&tsdf,&esdf})message->row_step=message->width*message->point_step;
    published_cells_=occupancy.width;
    returned.bytes=occupancy.data.size();
    if(snapshot_trace_.is_open())snapshot_trace_bytes_=returned.bytes;
    traceSnapshot("snapshot_build_end",returned.stamp,returned.bytes,trace_sequence);
    traceSnapshot("voxel_map_publish_begin",returned.stamp,occupancy.data.size(),trace_sequence);
    map_pub_->publish(occupancy);
    traceSnapshot("voxel_map_publish_end",returned.stamp,occupancy.data.size(),trace_sequence);
    traceSnapshot("tsdf_voxels_publish_begin",returned.stamp,tsdf.data.size(),trace_sequence);
    tsdf_pub_->publish(tsdf);
    traceSnapshot("tsdf_voxels_publish_end",returned.stamp,tsdf.data.size(),trace_sequence);
    if(backend_->config().esdf_enabled&&esdf_update_period_==0.){
      traceSnapshot("esdf_voxels_publish_begin",returned.stamp,esdf.data.size(),trace_sequence);
      esdf_pub_->publish(esdf);
      traceSnapshot("esdf_voxels_publish_end",returned.stamp,esdf.data.size(),trace_sequence);
    }
    if(esdf_update_period_>0.&&(explicit_products||replaced_esdf))
      publishEsdfVoxels();
    if(esdf_update_period_>0.&&replaced_esdf&&!explicit_products)
      publishEsdfProducts(publish_pointclouds_,esdf_stamp_);
    ++map_update_id_;map_change_kind_=change_kind;
    if(pending_observation_scans_){
      // An explicit snapshot/augmentation also includes all integrated scans.
      // Advance sensor provenance once for that published batch, not once per
      // timer tick or synthetic operation; observation_id stays <= update_id.
      ++map_observation_id_;map_observation_stamp_=pending_observation_stamp_;
    }
    planner_msgs::msg::VoxelMapUpdate update;
    update.source_id=map_source_id_;update.map_epoch=map_epoch_;update.update_id=map_update_id_;
    update.observation_id=map_observation_id_;update.observation_stamp=map_observation_stamp_;
    update.change_kind=change_kind;
    // Move the existing 24-byte point buffer after legacy publication. Always
    // publish the envelope, including without readers, so transient-local
    // subscribers cannot receive an old cached map after a quiet interval.
    update.map=std::move(occupancy);
    traceSnapshot("voxel_map_update_publish_begin",returned.stamp,update.map.data.size(),trace_sequence);
    map_update_pub_->publish(update);
    traceSnapshot("voxel_map_update_publish_end",returned.stamp,update.map.data.size(),trace_sequence);
    traceSnapshot("distance_products_begin",returned.stamp,0,trace_sequence);
    publishDistanceProducts(stamp,explicit_products||publish_pointclouds_on_update_||visualization_refresh_needed_);
    traceSnapshot("distance_products_end",returned.stamp,0,trace_sequence);
    visualization_refresh_needed_=false;
    ++snapshot_publications_;
    if(pending_observation_scans_){
      ++observation_snapshot_publications_;
      coalesced_observation_scans_+=pending_observation_scans_-1;
      pending_observation_scans_=0;pending_observation_stamp_={};
    }
    last_snapshot_publish_steady_=std::chrono::steady_clock::now();
    traceSnapshot("snapshot_returning",returned.stamp,returned.bytes,trace_sequence);
    if(return_snapshot)return std::move(update);
    return std::nullopt;
  }
  void publishDistanceProducts(const builtin_interfaces::msg::Time& stamp,bool full_products,
                               bool include_esdf=true){
    traceSnapshot("tsdf_products_begin",stamp);
    auto send=[&](sensor_msgs::msg::PointCloud2 cloud,
                  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher){
      cloud.header.frame_id=frame_;cloud.header.stamp=stamp;publisher->publish(cloud);
    };
    // Full visualization is opt-in, matching the original server flags. The
    // planning snapshot must not acquire four extra full-layer traversals on
    // every scan when no visualization was requested.
    if(full_products){
      send(createSurfacePointcloudFromTsdfLayer(backend_->tsdfLayer()),surface_cloud_pub_);
      send(createDistancePointcloudFromTsdfLayer(backend_->tsdfLayer()),tsdf_cloud_pub_);
      auto occupied=createOccupancyBlocksFromTsdfLayer(backend_->tsdfLayer(),
        backend_->tsdfLayer().voxel_size()*occupancy_min_distance_factor_);
      for(auto& marker:occupied.markers){marker.header.frame_id=frame_;marker.header.stamp=stamp;}
      occupied_nodes_pub_->publish(occupied);
    }
    if(publish_slices_){
      send(createDistancePointcloudFromTsdfLayerSlice(backend_->tsdfLayer(),2,slice_level_),tsdf_slice_pub_);
    }
    traceSnapshot("tsdf_products_end",stamp);
    if(include_esdf&&(esdf_update_period_==0.||full_products))
      publishEsdfProducts(full_products,esdf_update_period_==0.?stamp:esdf_stamp_);
  }
  std::string frame_,sensor_frame_,map_file_,mesh_file_,color_mode_,storage_;std::unique_ptr<TsdfBackend> backend_;
  double esdf_update_period_{0.};
  bool esdf_pending_{false};
  uint64_t esdf_updates_{0};
  uint64_t esdf_input_sequence_{0},esdf_completed_sequence_{0};
  std::size_t esdf_point_count_{0};
  builtin_interfaces::msg::Time esdf_stamp_,esdf_latest_scan_stamp_;
  rclcpp::TimerBase::SharedPtr esdf_timer_;
  std::ofstream snapshot_trace_;
  std::unique_ptr<VoxelScanCapture> voxel_scan_capture_;
  uint64_t voxel_capture_scan_sequence_{0};
  uint64_t snapshot_trace_sequence_{0},snapshot_trace_returned_{0};
  builtin_interfaces::msg::Time snapshot_trace_stamp_;
  std::size_t snapshot_trace_bytes_{0};
  std::array<std::size_t,3> snapshot_point_counts_{};
  std::string clear_sphere_pose_source_;
  bool use_tf_transforms_{true};
  std::unique_ptr<TransformQueue> transform_queue_;
  voxblox::Transformation T_B_D_,T_B_C_;
  uint64_t received_transforms_{0},rejected_transforms_{0};
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr transform_sub_;
  std::unique_ptr<IcpRefiner> icp_refiner_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> icp_broadcaster_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr icp_transform_pub_;
  std::string icp_corrected_frame_,pose_corrected_frame_;
  uint64_t icp_runs_{0},icp_last_batches_{0},icp_batches_{0};
  bool publish_slices_{false},publish_traversable_{false};
  bool publish_pointclouds_{false},publish_pointclouds_on_update_{false};
  bool visualization_refresh_needed_{false};
  double max_block_distance_{std::numeric_limits<voxblox::FloatingPoint>::max()};
  std::size_t pruned_tsdf_blocks_{0},pruned_esdf_blocks_{0},pruned_mesh_blocks_{0};
  double slice_level_{.5},traversability_radius_{1.0};
  double occupancy_min_distance_factor_{1.0};
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr surface_cloud_pub_,tsdf_cloud_pub_,esdf_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr occupied_nodes_pub_;
  std::unique_ptr<voxblox::MeshLayer> mesh_layer_;
  bool mesh_full_rebuild_{true};
  MeshColorMode mesh_color_mode_{MeshColorMode::Color};
  std::set<std::string> mesh_readers_;
  rclcpp::Publisher<voxblox_msgs::msg::Mesh>::SharedPtr mesh_blocks_pub_;
  rclcpp::TimerBase::SharedPtr mesh_readers_timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tsdf_slice_pub_,esdf_slice_pub_,traversable_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_clouds_service_;
  int pointcloud_queue_size_{1000};
  std::string pointcloud_reliability_{"best_effort"};
  double min_time_between_msgs_sec_{0.},timestamp_tolerance_sec_{0.001};
  int64_t min_time_between_msgs_ns_{0};
  rclcpp::TimerBase::SharedPtr mesh_timer_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
  rclcpp::TimerBase::SharedPtr layers_timer_;
  bool publish_tsdf_layer_{false},publish_esdf_layer_{false};
  uint64_t received_layers_{0},rejected_layers_{0};
  std::set<std::string> tsdf_layer_readers_,esdf_layer_readers_;
  rclcpp::Publisher<voxblox_msgs::msg::Layer>::SharedPtr tsdf_layer_pub_,esdf_layer_pub_;
  rclcpp::Subscription<voxblox_msgs::msg::Layer>::SharedPtr tsdf_layer_sub_,esdf_layer_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr publish_layers_service_;
  double update_mesh_every_n_sec_{0.};bool mesh_dirty_{false};uint64_t mesh_updates_{0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::vector<InputSource> inputs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> cloud_subscriptions_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr freespace_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_,tsdf_pub_,esdf_pub_;
  rclcpp::Publisher<planner_msgs::msg::VoxelMapUpdate>::SharedPtr map_update_pub_;
  std::string map_source_id_{newMapSourceId()},map_change_kind_{"snapshot"};
  uint64_t map_epoch_{1},map_update_id_{0},map_observation_id_{0};
  builtin_interfaces::msg::Time map_observation_stamp_;
  double snapshot_publish_period_{0.};
  std::chrono::steady_clock::time_point last_snapshot_publish_steady_{};
  uint64_t pending_observation_scans_{0},snapshot_publications_{0};
  uint64_t observation_snapshot_publications_{0},coalesced_observation_scans_{0};
  builtin_interfaces::msg::Time pending_observation_stamp_;
  rclcpp::TimerBase::SharedPtr snapshot_timer_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_,reset_service_,mesh_service_,clear_box_service_;
  rclcpp::Service<planner_msgs::srv::AugmentFreeBox>::SharedPtr augment_box_service_;
  rclcpp::Service<gbplanner_voxblox::srv::FilePath>::SharedPtr save_service_,load_service_,export_service_;
  int scans_{0},rejected_{0},loads_{0};std::size_t known_{0},published_cells_{0};TsdfScanResult last_;
  bool freespace_enabled_{false},startup_box_{false},startup_box_done_{false},odometry_valid_{false};
  std::set<std::string> freespace_frames_;
  static constexpr std::size_t kInternalQueueLimit=10;
  std::size_t freespace_scans_{0},freespace_rejected_{0},freespace_throttled_{0},freespace_queue_dropped_{0},empty_freespace_{0},box_clears_{0},cleared_voxels_{0},esdf_clear_sphere_operations_{0};
  bool freespace_has_last_accepted_{false};
  int64_t freespace_last_stamp_ns_{0};
  std::deque<sensor_msgs::msg::PointCloud2::ConstSharedPtr> freespace_pending_;
  TsdfScanResult last_freespace_;
  Eigen::Vector3d robot_position_{0,0,0},robot_size_{.4,.4,.4},robot_offset_{0,0,0};
  double odometry_stamp_{0},odometry_timeout_{2.};
};
}
int main(int argc,char** argv){
  rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT,requestStop);
  std::signal(SIGTERM,requestStop);
  try{
    auto node=std::make_shared<gbplanner2::VoxbloxMapperNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    // A signal must not invalidate the context while a callback is querying
    // graph subscribers or publishing a map. Finish it, then destroy the
    // executor/node (including the TF listener) before shutting DDS down.
    while(rclcpp::ok()&&!stop_requested){
      executor.spin_once(std::chrono::milliseconds(20));
      node->traceExecutorReturn();
    }
  }
  catch(const std::exception& error){std::cerr<<"Mapper failed: "<<error.what()<<std::endl;rclcpp::shutdown();return 1;}
  rclcpp::shutdown();return 0;
}
