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
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINECAMERA_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINECAMERA_HH_

#include <luxcore/luxcore.h>

#include "ignition/rendering/base/BaseCamera.hh"

#include "LuxCoreEngineRenderTarget.hh"
#include "LuxCoreEngineRenderTypes.hh"
#include "LuxCoreEngineSensor.hh"

namespace ignition {
namespace rendering {

inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

/// \brief LuxCore implementation of the camera class
class LuxCoreEngineCamera : public BaseCamera<LuxCoreEngineSensor>
{
  /// \brief Constructor
  public: LuxCoreEngineCamera();

  /// \brief Destructor
  public: virtual ~LuxCoreEngineCamera();

  // Documentation inherited.
  public: virtual void Render() override;

  // Documentation inherited.
  public: virtual void Update() override;

  // Documentation inherited.
  public: virtual void SetLocalPosition(double _x, double _y, double _z) override;

  // Documentation inherited.
  public: virtual void SetLocalRotation(double _r, double _p, double _y) override;

  // Documentation inherited.
  public: virtual void SetHFOV(const math::Angle &_hfov) override;

  // Documentation inherited.
  protected: virtual RenderTargetPtr RenderTarget() const override;

  /// \brief Pointer to the render target
  protected: LuxCoreEngineRenderTargetPtr renderTarget;

  /// \brief Pointer to the lux core render senssion
  protected: luxcore::RenderSession *renderSessionLux{nullptr};

  /// \brief Position x in local frame
  protected: float localPositionX{0.0f};

  /// \brief Position y in local frame
  protected: float localPositionY{0.0f};

  /// \brief Position z in local frame
  protected: float localPositionZ{0.0f};

  /// \brief Roll rotation in local frame
  protected: float localRotationR{0.0f};

  /// \brief Pitch rotation in local frame
  protected: float localRotationP{0.0f};

  /// \brief Yaw rotation in local frame
  protected: float localRotationY{0.0f};
};

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition

#endif
