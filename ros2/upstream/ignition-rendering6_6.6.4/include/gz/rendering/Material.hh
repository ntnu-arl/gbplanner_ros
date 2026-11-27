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
#ifndef GZ_RENDERING_MATERIAL_HH_
#define GZ_RENDERING_MATERIAL_HH_

#include <string>
#include <gz/math/Color.hh>
#include <gz/common/Material.hh>
#include "gz/rendering/config.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/Object.hh"
#include "gz/rendering/ShaderType.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief An enum for the type of material
    enum IGNITION_RENDERING_VISIBLE MaterialType
    {
      /// \brief Classic shading, i.e. variants of Phong
      MT_CLASSIC = 0,

      /// \brief Physically Based Shading
      MT_PBS = 1
    };

    /// \class Material Material.hh ignition/rendering/Material.hh
    /// \brief Represents a surface material of a Geometry
    class IGNITION_RENDERING_VISIBLE Material :
      public virtual Object
    {
      /// \brief Deconstructor
      public: virtual ~Material() { }

      /// \brief Determine if lighting affects this material
      /// \return True if lighting affects this material
      public: virtual bool LightingEnabled() const = 0;

      /// \brief Specify if lighting affects this material
      /// \param[in] _enabled True if lighting affects this material
      public: virtual void SetLightingEnabled(const bool _enabled) = 0;

      /// \brief Specify if depth buffer checking is enabled
      /// \param[in] _enabled True if depth check is enabled
      public: virtual void SetDepthCheckEnabled(bool _enabled) = 0;

      /// \brief Determine if depth buffer checking is enabled
      /// \return True if depth check is enabled
      public: virtual bool DepthCheckEnabled() const = 0;

      /// \brief Specify if depth buffer writing is enabled
      /// \param[in] _enabled True if depth buffer writing is enabled
      public: virtual void SetDepthWriteEnabled(bool _enabled) = 0;

      /// \brief Determine if depth buffer writing is enabled
      /// \return True if depth buffer writing is enabled
      public: virtual bool DepthWriteEnabled() const = 0;

      /// \brief Get the ambient color
      /// \return The ambient color
      public: virtual math::Color Ambient() const = 0;

      /// \brief Set the ambient color
      /// \param[in] _r Red value
      /// \param[in] _g Green value
      /// \param[in] _b Blue value
      /// \param[in] _a Alpha value
      public: virtual void SetAmbient(const double _r, const double _g,
                  const double _b, const double _a = 1.0) = 0;

      /// \brief Set the ambient color
      /// \param[in] _color New ambient color
      public: virtual void SetAmbient(const math::Color &_color) = 0;

      /// \brief Get the diffuse color
      /// \return The diffuse color
      public: virtual math::Color Diffuse() const = 0;

      /// \brief Set the diffuse color
      /// \param[in] _r Red value
      /// \param[in] _g Green value
      /// \param[in] _b Blue value
      /// \param[in] _a Alpha value
      public: virtual void SetDiffuse(const double _r, const double _g,
                  const double _b, const double _a = 1.0) = 0;

      /// \brief Set the diffuse color
      /// \param[in] _color New diffuse color
      public: virtual void SetDiffuse(const math::Color &_color) = 0;

      /// \brief Get the specular color
      /// \return The specular color
      public: virtual math::Color Specular() const = 0;

      /// \brief Set the specular color
      /// \param[in] _r Red value
      /// \param[in] _g Green value
      /// \param[in] _b Blue value
      /// \param[in] _a Alpha value
      public: virtual void SetSpecular(const double _r, const double _g,
                  const double _b, const double _a = 1.0) = 0;

      /// \brief Set the specular color
      /// \param[in] _color New specular color
      public: virtual void SetSpecular(const math::Color &_color) = 0;

      /// \brief Get the emissive color
      /// \return The emissive color
      public: virtual math::Color Emissive() const = 0;

      /// \brief Set the emissive color
      /// \param[in] _r Red value
      /// \param[in] _g Green value
      /// \param[in] _b Blue value
      /// \param[in] _a Alpha value
      public: virtual void SetEmissive(const double _r, const double _g,
                  const double _b, const double _a = 1.0) = 0;

      /// \brief Set the emissive color
      /// \param[in] _color New emissive color
      public: virtual void SetEmissive(const math::Color &_color) = 0;

      /// \brief Get the shininess value
      /// \return The shininess value
      public: virtual double Shininess() const = 0;

      /// \brief Set the shininess value
      /// \param[in] _shininess New shininess value
      public: virtual void SetShininess(const double _shininess) = 0;

      /// \brief Get the transparency value
      /// \return The transparency value
      public: virtual double Transparency() const = 0;

      /// \brief Configuration for Depth Material
      /// \param[in] far Far distance for normalized output
      /// \param[in] near Near distance for normalized output
      public: virtual void SetDepthMaterial(const double far,
            const double near) = 0;

      /// \brief Set the transparency value
      /// \param[in] _transparency New transparency value
      public: virtual void SetTransparency(const double _transparency) = 0;

      /// \brief Set the material to use the alpha channel from the textures
      /// \param[in] _enabled Enable alpha channel based rendering
      /// \param[in] _alpha Set the alpha threshold value
      /// \param[in] _twoSided Enable two sided rendering
      public: virtual void SetAlphaFromTexture(bool _enabled,
                  double _alpha = 0.5, bool _twoSided = true) = 0;

      /// \brief Get the enable alpha from textures
      /// \return The enable alpha value
      public: virtual bool TextureAlphaEnabled() const = 0;

      /// \brief Get the alpha threshold
      /// \return The alpha threshold value
      public: virtual double AlphaThreshold() const = 0;

      /// \brief Get the enable two sided rendering value
      /// \return The enable two sided rendering value
      public: virtual bool TwoSidedEnabled() const = 0;

      /// \brief Get the reflectivity value
      /// \return The reflectivity value
      public: virtual double Reflectivity() const = 0;

      /// \brief Set the reflectivity value
      /// \param[in] _reflectivity New reflectivity value
      public: virtual void SetReflectivity(const double _reflectivity) = 0;

      /// \brief Determine if this material casts shadows
      /// \return True if this material casts shadows
      public: virtual bool CastShadows() const = 0;

      /// \brief Specify if this material casts shadows
      /// \param[in] _castShadows True if this material casts shadows
      public: virtual void SetCastShadows(const bool _castShadows) = 0;

      /// \brief Determine if this material receives shadows
      /// \return True if this material receives shadows
      public: virtual bool ReceiveShadows() const = 0;

      /// \brief Specify if this material receives shadows
      /// \param[in] _receiveShadows True if this material receives shadows
      public: virtual void SetReceiveShadows(const bool _receiveShadows) = 0;

      /// \brief Determine if this material has a reflection
      /// \return True if this material has a reflection
      public: virtual bool ReflectionEnabled() const = 0;

      /// \brief Specify if this material has a reflection
      /// \param[in] _enabled True if this material has a reflection
      public: virtual void SetReflectionEnabled(const bool _enabled) = 0;

      /// \brief Determine if this material has a texture
      /// \return True if this material has a texture
      public: virtual bool HasTexture() const = 0;

      /// \brief Get the URI of the texture file
      /// \return URI of the texture file
      public: virtual std::string Texture() const = 0;

      /// \brief Set the material texture
      /// \param[in] _texture URI of the new texture file
      public: virtual void SetTexture(const std::string &_texture) = 0;

      /// \brief Removes any texture mapped to this material
      public: virtual void ClearTexture() = 0;

      /// \brief Determine if this material has a normal map
      /// \return True if this material has a normal map
      public: virtual bool HasNormalMap() const = 0;

      /// \brief Get the URI of the normal map file
      /// \return URI of the normal map file
      public: virtual std::string NormalMap() const = 0;

      /// \brief Set the material normal map
      /// \param[in] _normalMap URI of the new normal map file
      public: virtual void SetNormalMap(const std::string &_normalMap) = 0;

      /// \brief Removes any normal map mapped to this material
      public: virtual void ClearNormalMap() = 0;

      /// \brief Determine if this material has a roughness map
      /// \return True if this material has a roughness map
      public: virtual bool HasRoughnessMap() const = 0;

      /// \brief Get the URI of the roughness map file
      /// \return URI of the roughness map file
      public: virtual std::string RoughnessMap() const = 0;

      /// \brief Set the material roughness map
      /// \param[in] _roughnessMap URI of the new roughness map file
      public: virtual void SetRoughnessMap(
        const std::string &_roughnessMap) = 0;

      /// \brief Removes any roughness map mapped to this material
      public: virtual void ClearRoughnessMap() = 0;

      /// \brief Determine if this material has a metalness map
      /// \return True if this material has a metalness map
      public: virtual bool HasMetalnessMap() const = 0;

      /// \brief Get the URI of the metalness map file
      /// \return URI of the metalness map file
      public: virtual std::string MetalnessMap() const = 0;

      /// \brief Set the material metalness map
      /// \param[in] _metalnessMap URI of the new metalness map file
      public: virtual void SetMetalnessMap(
        const std::string &_metalnessMap) = 0;

      /// \brief Removes any metalness map mapped to this material
      public: virtual void ClearMetalnessMap() = 0;

      /// \brief Determine if this material has a environment map
      /// \return True if this material has a environment map
      public: virtual bool HasEnvironmentMap() const = 0;

      /// \brief Get the URI of the environment map file
      /// \return URI of the environment map file
      public: virtual std::string EnvironmentMap() const = 0;

      /// \brief Set the material environment map
      /// \param[in] _metalnessMap URI of the new environment map file
      public: virtual void SetEnvironmentMap(
        const std::string &_metalnessMap) = 0;

      /// \brief Removes any environment map mapped to this material
      public: virtual void ClearEnvironmentMap() = 0;

      /// \brief Determine if this material has an emissive map
      /// \return True if this material has an emissive map
      public: virtual bool HasEmissiveMap() const = 0;

      /// \brief Get the URI of the emissive map file
      /// \return URI of the emissive map file
      public: virtual std::string EmissiveMap() const = 0;

      /// \brief Set the material emissive map
      /// \param[in] _emissiveMap URI of the new emissive map file
      public: virtual void SetEmissiveMap(
        const std::string &_emissiveMap) = 0;

      /// \brief Removes any emissive map mapped to this material
      public: virtual void ClearEmissiveMap() = 0;

      /// \brief Determine if this material has a light map
      /// \return True if this material has a light map
      public: virtual bool HasLightMap() const = 0;

      /// \brief Get the URI of the light map file
      /// \return URI of the light map file
      public: virtual std::string LightMap() const = 0;

      /// \brief Get the texture coordinate set used by lightmap
      /// \return texture coordinate set of the light map
      public: virtual unsigned int LightMapTexCoordSet() const = 0;

      /// \brief Set the material light map
      /// \param[in] _lightMap URI of the new light map file
      /// \param[in] _uvSet Texture coordinate set to use
      public: virtual void SetLightMap(const std::string &_lightMap,
          unsigned int _uvSet = 0u) = 0;

      /// \brief Removes any light map mapped to this material
      public: virtual void ClearLightMap() = 0;

      /// \brief Set the render order. When polygons are coplanar, you can get
      /// problems with 'depth fighting' where the pixels from the two polys
      /// compete for the same screen pixel. This param help to avoid this
      /// problem.
      /// \param[in] _renderOrder Render order to set to
      public: virtual void SetRenderOrder(const float _renderOrder) = 0;

      /// \brief Get the render order value of this material.
      /// \return Material render order
      public: virtual float RenderOrder() const = 0;

      /// \brief Set the roughness value. Only affects material of type MT_PBS
      /// \param[in] _roughness Roughness to set to
      public: virtual void SetRoughness(const float _roughness) = 0;

      /// \brief Get the roughness value of this material.
      /// \return Material roughness
      public: virtual float Roughness() const = 0;

      /// \brief Set the metalness value. Only affects material of type MT_PBS
      /// \param[in] _metalness Metalness to set to
      public: virtual void SetMetalness(const float _metalness) = 0;

      /// \brief Get the metalness value of this material
      /// \return Material metalness
      public: virtual float Metalness() const = 0;

      /// \brief Removes any metalness map mapped to this material
      public: virtual enum MaterialType Type() const = 0;

      /// \brief Get the ShaderType value
      /// \return The ShaderType value
      public: virtual enum ShaderType ShaderType() const = 0;

      /// \brief Set the ShaderType value
      /// \param[in] _type New ShaderType value
      public: virtual void SetShaderType(enum ShaderType _type) = 0;

      /// \brief Clone this material
      /// \return New cloned material
      public: virtual MaterialPtr Clone(const std::string &_name = "")
                const = 0;

      /// \brief Copy properties from given Material
      /// \param[in] _material Source Material to be copied from
      public: virtual void CopyFrom(ConstMaterialPtr _material) = 0;

      /// \brief Copy properties from given Material
      /// \param[in] _material Source Material to be copied from
      public: virtual void CopyFrom(const common::Material &_material) = 0;

      /// \brief Get path to the vertex shader
      /// \return Path to vertex shader
      public: virtual std::string VertexShader() const = 0;

      /// \brief Get params for the vertex shader
      /// \return editable parameters
      public: virtual ShaderParamsPtr VertexShaderParams() = 0;

      /// \brief Set the vertex shader
      /// \param[in] _path path to a file containing a glsl shader
      public: virtual void SetVertexShader(const std::string &_path) = 0;

      /// \brief Get path to the fragment shader
      /// \return Path to fragment shader
      public: virtual std::string FragmentShader() const = 0;

      /// \brief Get params for the fragment shader
      /// \return editable parameters
      public: virtual ShaderParamsPtr FragmentShaderParams() = 0;

      /// \brief Set the fragment shader
      /// \param[in] _path path to a file containing a glsl shader
      public: virtual void SetFragmentShader(const std::string &_path) = 0;
    };
    }
  }
}
#endif
