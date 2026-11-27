/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_RENDERING_OPTIX_OPTIXNODE_HH_
#define GZ_RENDERING_OPTIX_OPTIXNODE_HH_

#include "gz/rendering/base/BaseNode.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixRenderTypes.hh"
#include "gz/rendering/optix/OptixObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_OPTIX_VISIBLE OptixNode :
      public BaseNode<OptixObject>
    {
      protected: OptixNode();

      public: virtual ~OptixNode();

      public: virtual bool HasParent() const override;

      public: virtual NodePtr Parent() const override;

      public: virtual void PreRender() override;

      public: virtual optix::Transform OptixTransform() const;

      // Documentation inherited.
      public: virtual math::Vector3d LocalScale() const override;

      // Documentation inherited.
      public: virtual bool InheritScale() const override;

      // Documentation inherited.
      public: virtual void SetInheritScale(bool _inherit) override;

      // Documentation inherited.
      protected: virtual void SetLocalScaleImpl(
                     const math::Vector3d &_scale) override;

      protected: virtual math::Pose3d RawLocalPose() const override;

      protected: virtual void SetRawLocalPose(const math::Pose3d &_pose)
          override;

      protected: virtual void WritePoseToDevice();

      protected: virtual void WritePoseToDeviceImpl();

      protected: virtual void SetParent(OptixNodePtr _parent);

      protected: virtual void Init() override;

      protected: virtual NodeStorePtr Children() const override;

      protected: virtual bool AttachChild(NodePtr _child) override;

      protected: virtual bool DetachChild(NodePtr _child) override;

      protected: OptixNodePtr parent;

      protected: optix::Transform optixTransform;

      protected: optix::Group optixGroup;

      protected: optix::Acceleration optixAccel;

      protected: math::Pose3d pose;

      protected: bool poseDirty;

      protected: OptixNodeStorePtr children;

      protected: math::Vector3d scale = math::Vector3d::One;

      protected: bool inheritScale = true;

      private: OptixNodePtr SharedThis();

      // TODO remove the need for a visual friend class
      private: friend class OptixVisual;
    };
    }
  }
}
#endif
