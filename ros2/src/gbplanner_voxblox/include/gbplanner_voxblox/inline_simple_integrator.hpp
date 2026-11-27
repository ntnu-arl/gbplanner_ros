#pragma once

// Single-worker dispatch adapted from Voxblox SimpleTsdfIntegrator.
// Copyright (c) 2016, ETHZ ASL. BSD-3-Clause; see LICENSE.voxblox.
#include <voxblox/integrator/tsdf_integrator.h>

namespace gbplanner2 {

// The mapper already serializes calls to the backend. With one integration
// worker, invoking the original kernel here preserves its point order while
// avoiding a new thread and copies of the entire point/color arrays per scan.
// The vendor sources and the multi-worker dispatch remain unchanged.
class InlineSimpleTsdfIntegrator final : public voxblox::SimpleTsdfIntegrator {
 public:
  using voxblox::SimpleTsdfIntegrator::SimpleTsdfIntegrator;

  void integratePointCloud(const voxblox::Transformation& transform,
                           const voxblox::Pointcloud& points,
                           const voxblox::Colors& colors,
                           const bool freespace_points=false) override {
    if(config_.integrator_threads!=1) {
      voxblox::SimpleTsdfIntegrator::integratePointCloud(
          transform,points,colors,freespace_points);
      return;
    }
    voxblox::timing::Timer integrate_timer("integrate/simple");
    CHECK_EQ(points.size(),colors.size());
    std::unique_ptr<voxblox::ThreadSafeIndex> index_getter(
        voxblox::ThreadSafeIndexFactory::get(config_.integration_order_mode,points));
    // Use the upstream ray traversal, weighting, color fusion and block locks
    // verbatim. Only execution placement changes; no points are subsampled.
    integrateFunction(transform,points,colors,freespace_points,index_getter.get());
    integrate_timer.Stop();
    voxblox::timing::Timer insertion_timer("inserting_missed_blocks");
    updateLayerWithStoredBlocks();
    insertion_timer.Stop();
  }
};

}  // namespace gbplanner2
