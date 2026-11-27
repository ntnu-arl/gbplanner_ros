/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINENODE_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINENODE_HH_

#include "ignition/rendering/base/BaseNode.hh"

#include "LuxCoreEngineObject.hh"
#include "LuxCoreEngineRenderTypes.hh"

namespace ignition {
namespace rendering {
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

class LuxCoreEngineNode : public BaseNode<LuxCoreEngineObject>
{
  /// \brief Constructor
  protected: LuxCoreEngineNode();

  /// \brief Destructor
  public: virtual ~LuxCoreEngineNode();

  // Documentation inherited.
  public: virtual bool HasParent() const override;

  // Documentation inherited.
  public: virtual NodePtr Parent() const override;

  // Documentation inherited.
  public: virtual void PreRender() override;

  // Documentation inherited.
  public: virtual math::Vector3d LocalPosition() const override;

  // Documentation inherited.
  public: virtual math::Vector3d LocalScale() const override;

  // Documentation inherited
  public: virtual bool InheritScale() const override;

  // Documentation inherited
  public: virtual void SetInheritScale(bool _inherit) override;

  // Documentation inherited
  protected: virtual void SetLocalScaleImpl(
      const math::Vector3d &_scale) override;

  // Documentation inherited
  protected: virtual math::Pose3d RawLocalPose() const override;

  // Documentation inherited
  protected: virtual void SetRawLocalPose(const math::Pose3d &_pose) override;

  /// \brief Set the parent node
  /// \param[in] _parent Node to set as parent
  protected: virtual void SetParent(LuxCoreEngineNodePtr _parent);

  /// \brief Initialize Node
  protected: virtual void Init();

  // Documentation inherited
  protected: virtual NodeStorePtr Children() const override;

  // Documentation inherited
  protected: virtual bool AttachChild(NodePtr _child) override;

  // Documentation inherited
  protected: virtual bool DetachChild(NodePtr _child) override;

  /// \brief Parent node
  protected: LuxCoreEngineNodePtr parent;

  /// \brief Pose of node in local frame
  protected: math::Pose3d pose;

  /// \brief True if pose is dirty, false otherwise
  protected: bool poseDirty{false};

  /// \brief Scale of the node
  protected: math::Vector3d scale = math::Vector3d::One;

  /// \brief True to inherite scale of parent
  protected: bool inheritScale = true;
};

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition

#endif
