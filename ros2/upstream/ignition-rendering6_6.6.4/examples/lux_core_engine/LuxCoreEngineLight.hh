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
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINELIGHT_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINELIGHT_HH_

#include <luxcore/luxcore.h>

#include "ignition/rendering/base/BaseLight.hh"

#include "LuxCoreEngineNode.hh"
#include "LuxCoreEngineRenderTypes.hh"

namespace ignition {
namespace rendering {
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

/// \brief LuxCore implementation of the light class
class LuxCoreEngineLight : public BaseLight<LuxCoreEngineNode>
{
  // Documentation inherited.
  public: virtual math::Color DiffuseColor() const override;

  // Documentation inherited.
  public: virtual void SetDiffuseColor(double _r, double _g, double _b,
                                       double _a = 1.0) override;

  // Documentation inherited.
  public: virtual void SetDiffuseColor(const math::Color &_color) override;

  // Documentation inherited.
  public: virtual math::Color SpecularColor() const override;

  // Documentation inherited.
  public: virtual void SetSpecularColor(const math::Color &_color) override;

  // Documentation inherited.
  public: virtual double AttenuationConstant() const override;

  // Documentation inherited.
  public: virtual void SetAttenuationConstant(double _value) override;

  // Documentation inherited.
  public: virtual double AttenuationLinear() const override;

  // Documentation inherited.
  public: virtual void SetAttenuationLinear(double _value) override;

  // Documentation inherited.
  public: virtual double AttenuationQuadratic() const override;

  // Documentation inherited.
  public: virtual void SetAttenuationQuadratic(double _value) override;

  // Documentation inherited.
  public: virtual double AttenuationRange() const override;

  // Documentation inherited.
  public: virtual void SetAttenuationRange(double _range) override;

  // Documentation inherited.
  public: virtual bool CastShadows() const override;

  // Documentation inherited.
  public: virtual void SetCastShadows(bool _castShadows) override;

  // Documentation inherited.
  public: virtual double Intensity() const override;

  // Documentation inherited.
  public: virtual void SetIntensity(double _intensity) override;

  /// \brief Update Lux engine SDL
  public: virtual void UpdateLuxSDL() = 0;

  /// \brief Type of light
  protected: std::string lightType;

  /// \brief R color gain
  protected: float gainR{0.0f};

  /// \brief G color gain
  protected: float gainG{0.0f};

  /// \brief B color gain
  protected: float gainB{0.0f};
};

/// \brief LuxCore implementation of the directional light class
class LuxCoreEngineDirectionalLight
    : public BaseDirectionalLight<LuxCoreEngineLight>
{
  /// \brief Constructor
  /// \param[in] _lightType Type of light
  public:LuxCoreEngineDirectionalLight(std::string lightType);

  // Documentation Inherited
  public: virtual math::Vector3d Direction() const override;

  /// Documentation Inherited
  public: virtual void SetDirection(double _x, double _y, double _z) override;

  /// Documentation Inherited
  public: virtual void SetDirection(const math::Vector3d &_dir) override;

  // Documentation inherited.
  public: virtual void UpdateLuxSDL() override;

  /// \brief X direction of the light
  protected: float directionX{0.0f};

  /// \brief Y direction of the light
  protected: float directionY{0.0f};

  /// \brief Z direction of the light
  protected: float directionZ{0.0f};
};

/// \brief LuxCore implementation of the point light class
class LuxCoreEnginePointLight : public BasePointLight<LuxCoreEngineLight>
{
  /// \brief Constructor
  /// \param[in] _lightType Type of light
  public: LuxCoreEnginePointLight(std::string lightType);

  // Documentation Inherited
  public: virtual void SetLocalPosition(double _x, double _y, double _z)
              override;

  // Documentation inherited.
  public: virtual void UpdateLuxSDL() override;

  /// \brief X position in local frame
  protected: float localPositionX{0.0f};

  /// \brief Y position in local frame
  protected: float localPositionY{0.0f};

  /// \brief Z position in local frame
  protected: float localPositionZ{0.0f};
};

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition

#endif
