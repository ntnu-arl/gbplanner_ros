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
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINEMATERIAL_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINEMATERIAL_HH_

#include "ignition/rendering/base/BaseMaterial.hh"

#include "LuxCoreEngineObject.hh"
#include "LuxCoreEngineRenderTypes.hh"

namespace ignition {
namespace rendering {
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

/// \brief LuxCore implementation of the material class
class LuxCoreEngineMaterial : public BaseMaterial<LuxCoreEngineObject>
{
  /// \brief Constructor
  protected: LuxCoreEngineMaterial();

  /// \brief Destructor
  public: virtual ~LuxCoreEngineMaterial();

  public: void UpdateLuxSDL();

  // Documentation inherited
  public: void SetDiffuse(const double _r, const double _g, const double _b,
                          const double _a = 1.0) override;

  // Documentation inherited
  public: void SetEmissive(const double _r, const double _g, const double _b,
                           const double _a = 1.0) override;

  // Documentation inherited
  public: void SetShaderType(enum ShaderType _type) override;

  /// \brief R compoment of the diffuse color
  protected: float diffuseR{0.0f};

  /// \brief G compoment of the diffuse color
  protected: float diffuseG{0.0f};

  /// \brief B compoment of the diffuse color
  protected: float diffuseB{0.0f};

  /// \brief R compoment of the emissive color
  protected: float emissiveR{0.0f};

  /// \brief G compoment of the emissive color
  protected: float emissiveG{0.0f};

  /// \brief B compoment of the emissive color
  protected: float emissiveB{0.0f};

  /// \brief Type of material
  protected: std::string materialType;

  private: friend class LuxCoreEngineScene;
};

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition

#endif
