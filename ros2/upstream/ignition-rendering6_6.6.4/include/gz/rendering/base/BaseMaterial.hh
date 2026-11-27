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
#ifndef GZ_RENDERING_BASE_BASEMATERIAL_HH_
#define GZ_RENDERING_BASE_BASEMATERIAL_HH_

#include <string>

#include "gz/common/Console.hh"

#include "gz/rendering/Material.hh"
#include "gz/rendering/Scene.hh"
#include "gz/rendering/ShaderType.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Default pbr material properties
    static const common::Pbr kDefaultPbr;

    template <class T>
    class BaseMaterial :
      public virtual Material,
      public virtual T
    {
      protected: BaseMaterial();

      public: virtual ~BaseMaterial();

      // Documentation inherited
      public: virtual MaterialPtr Clone(const std::string &_name = "") const
                  override;

      // Documentation inherited
      public: virtual void SetAmbient(const double _r, const double _g,
                  const double _b, const double _a = 1.0) override;

      // Documentation inherited
      public: virtual void SetAmbient(const math::Color &_color) override;

      // Documentation inherited
      public: virtual void SetDiffuse(const double _r, const double _g,
                  const double _b, const double _a = 1.0) override;

      // Documentation inherited
      public: virtual void SetDiffuse(const math::Color &_color) override;

      // Documentation inherited
      public: virtual void SetSpecular(const double _r, const double _g,
                  const double _b, const double _a = 1.0) override;

      // Documentation inherited
      public: virtual void SetSpecular(const math::Color &_color) override;

      // Documentation inherited
      public: virtual void SetEmissive(const double _r, const double _g,
                  const double _b, const double _a = 1.0) override;

      // Documentation inherited
      public: virtual void SetEmissive(const math::Color &_color) override;

      // Documentation inherited
      public: virtual void SetTransparency(const double _transparency) override;

      // Documentation inherited
      public: virtual void SetAlphaFromTexture(bool _enabled,
          double _alpha = 0.5, bool _twoSided = true) override;

      // Documentation inherited
      public: bool TextureAlphaEnabled() const override;

      // Documentation inherited
      public: double AlphaThreshold() const override;

      // Documentation inherited
      public: bool TwoSidedEnabled() const override;

      // Documentation inherited
      public: virtual void SetShininess(const double _shininess) override;

      // Documentation inherited
      public: virtual void SetReflectivity(const double _reflectivity) override;

      // Documentation inherited
      public: virtual void SetCastShadows(const bool _castShadows) override;

      // Documentation inherited
      public: virtual void SetReceiveShadows(const bool _receiveShadows)
                  override;

      // Documentation inherited
      public: virtual void SetReflectionEnabled(const bool _enabled) override;

      // Documentation inherited
      public: virtual void SetLightingEnabled(const bool _enabled) override;

      // Documentation inherited.
      public: virtual void SetDepthCheckEnabled(bool _enabled) override;

      // Documentation inherited.
      public: virtual void SetDepthWriteEnabled(bool _enabled) override;

      // Documentation inherited
      public: virtual math::Color Ambient() const override;

      // Documentation inherited
      public: virtual math::Color Diffuse() const override;

      // Documentation inherited
      public: virtual math::Color Specular() const override;

       // Documentation inherited
      public: virtual math::Color Emissive() const override;

      // Documentation inherited
      public: virtual double Transparency() const override;

      // Documentation inherited
      public: virtual double Reflectivity() const override;

      // Documentation inherited
      public: virtual double Shininess() const override;

      // Documentation inherited
      public: virtual bool CastShadows() const override;

      // Documentation inherited
      public: virtual bool ReceiveShadows() const override;

      // Documentation inherited
      public: virtual bool LightingEnabled() const override;

      // Documentation inherited
      public: virtual bool DepthCheckEnabled() const override;

      // Documentation inherited
      public: virtual bool DepthWriteEnabled() const override;

      // Documentation inherited
      public: virtual bool ReflectionEnabled() const override;

      // Documentation inherited
      public: virtual bool HasTexture() const override;

      // Documentation inherited
      public: virtual std::string Texture() const override;

      // Documentation inherited
      public: virtual void SetTexture(const std::string &_texture) override;

      // Documentation inherited
      public: virtual void ClearTexture() override;

      // Documentation inherited
      public: virtual bool HasNormalMap() const override;

      // Documentation inherited
      public: virtual std::string NormalMap() const override;

      // Documentation inherited
      public: virtual void SetNormalMap(const std::string &_normalMap)
          override;

      // Documentation inherited
      public: virtual void ClearNormalMap() override;

      // Documentation inherited
      public: virtual bool HasRoughnessMap() const override;

      // Documentation inherited
      public: virtual std::string RoughnessMap() const override;

      // Documentation inherited
      public: virtual void SetRoughnessMap(const std::string &_roughnessMap)
          override;

      // Documentation inherited
      public: virtual void ClearRoughnessMap() override;

      // Documentation inherited
      public: virtual bool HasMetalnessMap() const override;

      // Documentation inherited
      public: virtual std::string MetalnessMap() const override;

      // Documentation inherited
      public: virtual void SetMetalnessMap(const std::string &_metalnessMap)
          override;

      // Documentation inherited
      public: virtual void ClearMetalnessMap() override;

      // Documentation inherited
      public: virtual bool HasEnvironmentMap() const override;

      // Documentation inherited
      public: virtual std::string EnvironmentMap() const override;

      // Documentation inherited
      public: virtual void SetEnvironmentMap(const std::string &_metalnessMap)
          override;

      // Documentation inherited
      public: virtual void ClearEnvironmentMap() override;

      // Documentation inherited
      public: virtual bool HasEmissiveMap() const override;

      // Documentation inherited
      public: virtual std::string EmissiveMap() const override;

      // Documentation inherited
      public: virtual void SetEmissiveMap(const std::string &_emissiveMap)
          override;

      // Documentation inherited
      public: virtual void ClearEmissiveMap() override;

      // Documentation inherited
      public: virtual bool HasLightMap() const override;

      // Documentation inherited
      public: virtual std::string LightMap() const override;

      // Documentation inherited
      public: virtual unsigned int LightMapTexCoordSet() const override;

      // Documentation inherited
      public: virtual void SetLightMap(const std::string &_lightMap,
          unsigned int _uvSet = 0u) override;

      // Documentation inherited
      public: virtual void ClearLightMap() override;

      // Documentation inherited
      public: virtual void SetRenderOrder(const float _renderOrder) override;

      // Documentation inherited
      public: virtual float RenderOrder() const override;

      // Documentation inherited
      public: virtual void SetRoughness(const float _roughness) override;

      // Documentation inherited
      public: virtual float Roughness() const override;

      // Documentation inherited
      public: virtual void SetMetalness(const float _metalness) override;

      // Documentation inherited
      public: virtual float Metalness() const override;

      // Documentation inherited
      public: virtual MaterialType Type() const override;

      public: virtual void SetShaderType(enum ShaderType _type) override
             {
               (void)_type;
               // no op
             }

      // Documentation inherited
      public: virtual enum ShaderType ShaderType() const override
             {
               return ST_PIXEL;
             }

      // Documentation inherited.
      // \sa Material::SetDepthMaterial()
      public: virtual void SetDepthMaterial(const double far,
                  const double near) override;

      // Documentation inherited.
      // \sa Material::VertexShader() const
      public: virtual std::string VertexShader() const override;

      // Documentation inherited.
      // \sa Material::VertexShaderParams()
      public: virtual ShaderParamsPtr VertexShaderParams() override;

      // Documentation inherited.
      // \sa Material::SetVertexShader(const std::string &)
      public: virtual void SetVertexShader(const std::string &_path) override;

      // Documentation inherited.
      // \sa Material::FragmentShader() const
      public: virtual std::string FragmentShader() const override;

      // Documentation inherited.
      // \sa Material::FragmentShaderParams()
      public: virtual ShaderParamsPtr FragmentShaderParams() override;

      // Documentation inherited.
      // \sa Material::SetFragmentShader(const std::string &)
      public: virtual void SetFragmentShader(const std::string &_path) override;

      // Documentation inherited.
      public: virtual void CopyFrom(ConstMaterialPtr _material) override;

      // Documentation inherited.
      public: virtual void CopyFrom(const common::Material &_material) override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      protected: virtual void Reset();

      /// \brief Ambient color
      protected: math::Color ambient;

      /// \brief Diffuse color
      protected: math::Color diffuse;

      /// \brief Specular color
      protected: math::Color specular;

      /// \brief Emissive color
      protected: math::Color emissive;

      /// \brief Transparent. 1: fully transparent, 0: opaque
      protected: double transparency = 0.0;

      /// \brief Enable alpha channel based texture transparency
      protected: bool textureAlphaEnabled = false;

      /// \brief Threshold for alpha channel rejection
      protected: double alphaThreshold = 0.5;

      /// \brief Enable two sided rendering
      protected: bool twoSidedEnabled = false;

      /// \brief Material render order
      protected: double renderOrder = 0.0;

      /// \brief Shininess factor
      protected: double shininess = 0.0;

      /// \brief Reflectivity
      protected: double reflectivity = 0.0;

      /// \brief Flag to indicate if dynamic lighting is enabled
      protected: bool lightingEnabled = false;

      /// \brief Flag to indicate if depth buffer checking is enabled
      protected: bool depthCheckEnabled = true;

      /// \brief Flag to indicate if depth buffer writing is enabled
      protected: bool depthWriteEnabled = true;

      /// \brief Flag to indicate if reflection is enabled
      protected: bool reflectionEnabled = false;

      /// \brief True if material receives shadows
      protected: bool receiveShadows = true;

      /// \brief Set to true to enable object with this material to cast shadows
      protected: bool castShadows = true;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseMaterial<T>::BaseMaterial()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseMaterial<T>::~BaseMaterial()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetAmbient(const double _r, const double _g,
        const double _b, const double _a)
    {
      this->SetAmbient(math::Color(_r, _g, _b, _a));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetAmbient(const math::Color &_color)
    {
      this->ambient = _color;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetDiffuse(const double _r, const double _g,
        const double _b, const double _a)
    {
      this->SetDiffuse(math::Color(_r, _g, _b, _a));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetDiffuse(const math::Color &_color)
    {
      this->diffuse = _color;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetSpecular(const double _r, const double _g,
        const double _b, const double _a)
    {
      this->SetSpecular(math::Color(_r, _g, _b, _a));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetSpecular(const math::Color &_color)
    {
      this->specular = _color;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetEmissive(const double _r, const double _g,
        const double _b, const double _a)
    {
      this->SetEmissive(math::Color(_r, _g, _b, _a));
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetEmissive(const math::Color &_color)
    {
      this->emissive = _color;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetShininess(const double _shininess)
    {
      this->shininess = _shininess;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetTransparency(const double _transparency)
    {
      this->transparency = _transparency;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetAlphaFromTexture(bool _enabled, double _alpha,
                                       bool _twoSided)
    {
      this->textureAlphaEnabled = _enabled;
      this->alphaThreshold = _alpha;
      this->twoSidedEnabled = _twoSided;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::TextureAlphaEnabled() const
    {
      return this->textureAlphaEnabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseMaterial<T>::AlphaThreshold() const
    {
      return this->alphaThreshold;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::TwoSidedEnabled() const
    {
      return this->twoSidedEnabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetReflectivity(const double _reflectivity)
    {
      this->reflectivity = _reflectivity;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetReflectionEnabled(const bool  _enabled)
    {
      this->reflectionEnabled = _enabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetLightingEnabled(const bool _enabled)
    {
      this->lightingEnabled = _enabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetDepthCheckEnabled(bool _enabled)
    {
      this->depthCheckEnabled = _enabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetDepthWriteEnabled(bool _enabled)
    {
      this->depthWriteEnabled = _enabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetCastShadows(const bool _castShadows)
    {
      this->castShadows = _castShadows;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetReceiveShadows(const bool _receive)
    {
      this->receiveShadows = _receive;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetRenderOrder(const float _renderorder)
    {
      this->renderOrder = _renderorder;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Color BaseMaterial<T>::Ambient() const
    {
      return this->ambient;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Color BaseMaterial<T>::Diffuse() const
    {
      return this->diffuse;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Color BaseMaterial<T>::Specular() const
    {
      return this->specular;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Color BaseMaterial<T>::Emissive() const
    {
      return this->emissive;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseMaterial<T>::Shininess() const
    {
      return this->shininess;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseMaterial<T>::Transparency() const
    {
      return this->transparency;
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseMaterial<T>::RenderOrder() const
    {
      return this->renderOrder;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseMaterial<T>::Reflectivity() const
    {
      return this->reflectivity;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::CastShadows() const
    {
      return this->castShadows;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::ReceiveShadows() const
    {
      return this->receiveShadows;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::LightingEnabled() const
    {
      return this->lightingEnabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::DepthCheckEnabled() const
    {
      return this->depthCheckEnabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::DepthWriteEnabled() const
    {
      return this->depthWriteEnabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::ReflectionEnabled() const
    {
      return this->reflectionEnabled;
    }

    //////////////////////////////////////////////////
    template <class T>
    MaterialType BaseMaterial<T>::Type() const
    {
      return MT_CLASSIC;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::VertexShader() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    ShaderParamsPtr BaseMaterial<T>::VertexShaderParams()
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetVertexShader(const std::string &/*_path*/)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::FragmentShader() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    ShaderParamsPtr BaseMaterial<T>::FragmentShaderParams()
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetFragmentShader(const std::string &/*_path*/)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::HasTexture() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::Texture() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetTexture(const std::string &)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::ClearTexture()
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::HasNormalMap() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::NormalMap() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetNormalMap(const std::string &)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::ClearNormalMap()
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::HasRoughnessMap() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::RoughnessMap() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetRoughnessMap(const std::string &)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::ClearRoughnessMap()
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::HasMetalnessMap() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::MetalnessMap() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetMetalnessMap(const std::string &)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::ClearMetalnessMap()
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::HasEnvironmentMap() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::EnvironmentMap() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetEnvironmentMap(const std::string &)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::ClearEnvironmentMap()
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::HasEmissiveMap() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::EmissiveMap() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetEmissiveMap(const std::string &)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::ClearEmissiveMap()
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseMaterial<T>::HasLightMap() const
    {
      return false;
    }

    //////////////////////////////////////////////////
    template <class T>
    std::string BaseMaterial<T>::LightMap() const
    {
      return std::string();
    }

    //////////////////////////////////////////////////
    template <class T>
    unsigned int BaseMaterial<T>::LightMapTexCoordSet() const
    {
      return 0u;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetLightMap(const std::string &, unsigned int)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::ClearLightMap()
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetRoughness(const float)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseMaterial<T>::Roughness() const
    {
      return 0.0f;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetMetalness(const float)
    {
      // no op
    }

    //////////////////////////////////////////////////
    template <class T>
    float BaseMaterial<T>::Metalness() const
    {
      return 0.0f;
    }

    //////////////////////////////////////////////////
    template <class T>
    MaterialPtr BaseMaterial<T>::Clone(const std::string &_name) const
    {
      auto baseShared = this->shared_from_this();

      auto thisShared =
          std::dynamic_pointer_cast<const BaseMaterial<T>>(baseShared);

      MaterialPtr material = T::Scene()->CreateMaterial(_name);
      material->CopyFrom(thisShared);
      return material;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::CopyFrom(ConstMaterialPtr _material)
    {
      this->SetLightingEnabled(_material->LightingEnabled());
      this->SetAmbient(_material->Ambient());
      this->SetDiffuse(_material->Diffuse());
      this->SetSpecular(_material->Specular());
      this->SetEmissive(_material->Emissive());
      this->SetRenderOrder(_material->RenderOrder());
      this->SetShininess(_material->Shininess());
      this->SetAlphaFromTexture(_material->TextureAlphaEnabled(),
          _material->AlphaThreshold(), _material->TwoSidedEnabled());
      // override transparency / blend setting after setting alpha from texture
      this->SetTransparency(_material->Transparency());
      // override depth check / depth write after setting transparency
      this->SetDepthCheckEnabled(_material->DepthCheckEnabled());
      this->SetDepthWriteEnabled(_material->DepthWriteEnabled());
      this->SetReflectivity(_material->Reflectivity());
      this->SetCastShadows(_material->CastShadows());
      this->SetReceiveShadows(_material->ReceiveShadows());
      this->SetReflectionEnabled(_material->ReflectionEnabled());
      this->SetTexture(_material->Texture());
      this->SetNormalMap(_material->NormalMap());
      this->SetRoughnessMap(_material->RoughnessMap());
      this->SetMetalnessMap(_material->MetalnessMap());
      this->SetRoughness(_material->Roughness());
      this->SetMetalness(_material->Metalness());
      this->SetEnvironmentMap(_material->EnvironmentMap());
      this->SetEmissiveMap(_material->EmissiveMap());
      this->SetLightMap(_material->LightMap(),
          _material->LightMapTexCoordSet());
      this->SetShaderType(_material->ShaderType());
      this->SetVertexShader(_material->VertexShader());
      this->SetFragmentShader(_material->FragmentShader());
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::CopyFrom(const common::Material &_material)
    {
      this->SetLightingEnabled(_material.Lighting());
      this->SetAmbient(_material.Ambient());
      this->SetDiffuse(_material.Diffuse());
      this->SetSpecular(_material.Specular());
      this->SetEmissive(_material.Emissive());
      this->SetShininess(_material.Shininess());
      this->SetTransparency(_material.Transparency());
      this->SetAlphaFromTexture(_material.TextureAlphaEnabled(),
          _material.AlphaThreshold(), _material.TwoSidedEnabled());
      this->SetRenderOrder(_material.RenderOrder());
      // TODO(anyone): update common::Material
      this->SetReflectivity(0);
      this->SetTexture(_material.TextureImage());
      // TODO(anyone): update common::Material
      this->SetCastShadows(true);
      // TODO(anyone): update common::Material
      this->SetReceiveShadows(true);
      // TODO(anyone): update common::Material
      this->SetReflectionEnabled(true);
      // TODO(anyone): update common::Material
      this->ClearNormalMap();
      // TODO(anyone): update common::Material
      this->SetShaderType(ST_PIXEL);

      const common::Pbr *pbrMat = _material.PbrMaterial();
      if (!pbrMat)
        pbrMat = &kDefaultPbr;
      this->SetNormalMap(pbrMat->NormalMap());
      this->SetRoughnessMap(pbrMat->RoughnessMap());
      this->SetMetalnessMap(pbrMat->MetalnessMap());
      this->SetRoughness(pbrMat->Roughness());
      this->SetMetalness(pbrMat->Metalness());
      this->SetEnvironmentMap(pbrMat->EnvironmentMap());
      this->SetEmissiveMap(pbrMat->EmissiveMap());
      this->SetLightMap(pbrMat->LightMap(), pbrMat->LightMapTexCoordSet());
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::PreRender()
    {
      // do nothing
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::SetDepthMaterial(const double /*far*/,
          const double /*near*/)
    {
      // do nothing
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseMaterial<T>::Reset()
    {
      this->SetLightingEnabled(true);
      this->SetDepthCheckEnabled(true);
      this->SetDepthWriteEnabled(true);
      this->SetAmbient(0.3, 0.3, 0.3);
      this->SetDiffuse(1.0, 1.0, 1.0);
      this->SetSpecular(0.2, 0.2, 0.2);
      this->SetEmissive(0, 0, 0);
      this->SetRenderOrder(0);
      this->SetShininess(1.5);
      this->SetTransparency(0);
      this->SetReflectivity(0);
      this->SetCastShadows(true);
      this->SetReceiveShadows(true);
      this->SetReflectionEnabled(true);
      this->ClearTexture();
      this->ClearNormalMap();
      this->ClearRoughnessMap();
      this->ClearMetalnessMap();
      this->ClearEmissiveMap();
      this->ClearLightMap();
      this->SetRoughness(kDefaultPbr.Roughness());
      this->SetMetalness(kDefaultPbr.Metalness());
      this->SetShaderType(ST_PIXEL);
    }
    }
  }
}
#endif
