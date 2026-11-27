/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

// Note this include is placed in the src file because
// otherwise ogre produces compile errors
#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <Hlms/Pbs/OgreHlmsPbs.h>
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <Hlms/Unlit/OgreHlmsUnlit.h>
#include <Hlms/Unlit/OgreHlmsUnlitDatablock.h>
#include <OgreHighLevelGpuProgram.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreHlmsManager.h>
#include <OgreItem.h>
#include <OgreMaterialManager.h>
#include <OgrePixelFormatGpuUtils.h>
#include <OgreTechnique.h>
#include <OgreTextureGpuManager.h>
#include <Vao/OgreVaoManager.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Image.hh>

#include "gz/rendering/GraphicsAPI.hh"
#include "gz/rendering/ShaderParams.hh"
#include "gz/rendering/ShaderType.hh"
#include "gz/rendering/ogre2/Ogre2Material.hh"
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2RenderEngine.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"


/// \brief Private data for the Ogre2Material class
class gz::rendering::Ogre2MaterialPrivate
{
  /// \brief Ogre stores the name using hashes. This variable will
  /// store the material hash name
  public: std::string hashName;

  /// \brief Path to vertex shader program.
  public: std::string vertexShaderPath;

  /// \brief Path to fragment shader program.
  public: std::string fragmentShaderPath;

  /// \brief Parameters to be bound to the vertex shader
  public: ShaderParamsPtr vertexShaderParams;

  /// \brief Parameters to be bound to the fragment shader
  public: ShaderParamsPtr fragmentShaderParams;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Material::Ogre2Material()
  : dataPtr(std::make_unique<Ogre2MaterialPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2Material::~Ogre2Material()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2Material::Destroy()
{
  if (!this->Scene()->IsInitialized())
    return;

  if (!this->ogreDatablock)
    return;

  this->ogreHlmsPbs->destroyDatablock(this->ogreDatablockId);
  this->ogreDatablock = nullptr;

  if (this->ogreUnlitDatablock)
  {
    this->ogreUnlitDatablock->getCreator()->destroyDatablock(
        this->ogreUnlitDatablock->getName());
    this->ogreUnlitDatablock = nullptr;
  }

  // remove low level ogre material used by render targets
  if (this->ogreMaterial)
  {
    Ogre::MaterialManager &matManager = Ogre::MaterialManager::getSingleton();
    matManager.remove(this->ogreMaterial);
    this->ogreMaterial.reset();
  }

  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::TextureGpuManager *textureManager =
    root->getRenderSystem()->getTextureGpuManager();

  textureManager->setStagingTextureMaxBudgetBytes(
    8u * 1024u * 1024u);
  textureManager->setWorkerThreadMaxPreloadBytes(
    8u * 1024u * 1024u);
  textureManager->setWorkerThreadMaxPerStagingTextureRequestBytes(
    4u * 1024u * 1024u);

  Ogre::TextureGpuManager::BudgetEntryVec budget;
  textureManager->setWorkerThreadMinimumBudget( budget );
  Ogre::HlmsManager *hlmsManager = root->getHlmsManager();

  Ogre::TextureGpu* textureToRemove = nullptr;
  bool textureIsUse = false;
  // Check each material from each Hlms (except low level) to see if their
  // material is currently in use. If it's not, then its textures may be
  // not either
  for (size_t i = Ogre::HLMS_PBS; i < Ogre::HLMS_MAX; ++i)
  {
    Ogre::Hlms *hlms = hlmsManager->getHlms(static_cast<Ogre::HlmsTypes>(i));

    if(hlms)
    {
      const Ogre::Hlms::HlmsDatablockMap &datablocks = hlms->getDatablockMap();

      Ogre::Hlms::HlmsDatablockMap::const_iterator itor = datablocks.begin();
      Ogre::Hlms::HlmsDatablockMap::const_iterator end  = datablocks.end();

      while (itor != end)
      {
        if (i == Ogre::HLMS_PBS)
        {
          Ogre::HlmsPbsDatablock *derivedDatablock =
            static_cast<Ogre::HlmsPbsDatablock*>(itor->second.datablock);
          for (size_t texUnit = 0; texUnit < Ogre::NUM_PBSM_TEXTURE_TYPES;
            ++texUnit)
          {
            // Check each texture from the material
            Ogre::TextureGpu *tex = derivedDatablock->getTexture(texUnit);
            if (tex)
            {
              // If getLinkedRenderables is empty, then the material is
              // not in use, and thus so is potentially the texture
              if (!itor->second.datablock->getLinkedRenderables().empty())
              {
                if (tex->getNameStr() == this->textureName)
                {
                  textureIsUse = true;
                }
              }
              else
              {
                if (tex->getNameStr() == this->textureName)
                {
                  textureToRemove = tex;
                }
              }
            }
          }
        }
        ++itor;
      }
    }
  }

  if (textureToRemove && !textureIsUse)
  {
    Ogre2ScenePtr s = std::dynamic_pointer_cast<Ogre2Scene>(this->Scene());
    s->ClearMaterialsCache(this->textureName);
    this->Scene()->UnregisterMaterial(this->name);
    textureManager->destroyTexture(textureToRemove);
  }

  Ogre2ScenePtr s = std::dynamic_pointer_cast<Ogre2Scene>(this->Scene());
  Ogre::SceneManager *sceneManager = s->OgreSceneManager();
  sceneManager->shrinkToFitMemoryPools();

  Ogre::VaoManager *vaoManager = textureManager->getVaoManager();
  vaoManager->cleanupEmptyPools();
}

//////////////////////////////////////////////////
math::Color Ogre2Material::Diffuse() const
{
  Ogre::Vector3 color =
      this->ogreDatablock->getDiffuse();
  return math::Color(color.x, color.y, color.z, this->diffuse.A());
}

//////////////////////////////////////////////////
void Ogre2Material::SetDiffuse(const math::Color &_color)
{
  BaseMaterial::SetDiffuse(_color);
  this->ogreDatablock->setDiffuse(
      Ogre::Vector3(_color.R(), _color.G(), _color.B()));
  this->UpdateTransparency();
}

//////////////////////////////////////////////////
math::Color Ogre2Material::Specular() const
{
  Ogre::Vector3 color =
      this->ogreDatablock->getSpecular();
  return math::Color(color.x, color.y, color.z, 1.0);
}

//////////////////////////////////////////////////
void Ogre2Material::SetSpecular(const math::Color &_color)
{
  this->ogreDatablock->setSpecular(
      Ogre::Vector3(_color.R(), _color.G(), _color.B()));
}

//////////////////////////////////////////////////
math::Color Ogre2Material::Emissive() const
{
  Ogre::Vector3 color =
      this->ogreDatablock->getEmissive();
  return math::Color(color.x, color.y, color.z, 1.0);
}

//////////////////////////////////////////////////
void Ogre2Material::SetEmissive(const math::Color &_color)
{
  this->ogreDatablock->setEmissive(
      Ogre::Vector3(_color.R(), _color.G(), _color.B()));
}

//////////////////////////////////////////////////
void Ogre2Material::SetTransparency(const double _transparency)
{
  this->transparency = std::min(std::max(_transparency, 0.0), 1.0);
  this->UpdateTransparency();
}

//////////////////////////////////////////////////
void Ogre2Material::UpdateTransparency()
{
  Ogre::HlmsPbsDatablock::TransparencyModes mode;
  double opacity = (1.0 - this->transparency) * this->diffuse.A();
  if (math::equal(opacity, 1.0))
    mode = Ogre::HlmsPbsDatablock::None;
  else
    mode = Ogre::HlmsPbsDatablock::Transparent;

  // from ogre documentation: 0 = full transparency and 1 = fully opaque
  this->ogreDatablock->setTransparency(opacity, mode);

  // set transparent objects to be in a higher render queue group
  // so they blend properly with heightmaps (render queue 11)
  auto renderables = this->ogreDatablock->getLinkedRenderables();
  for (auto & renderable : renderables)
  {
    auto subItem = dynamic_cast<Ogre::SubItem *>(renderable);
    if (subItem)
    {
      if (mode == Ogre::HlmsPbsDatablock::None)
      {
        // by default, ogre items are in render queue 10
        // these are hardcoded in ogre-next and there does not seem to be
        // an enum of function to retrieve this default render queue group
        subItem->getParent()->setRenderQueueGroup(10);
      }
      else
      {
        // put in render queue group 200
        // v2 entities can be placed in groups 0-99 or 200-224
        subItem->getParent()->setRenderQueueGroup(200);
      }
    }
  }
}

//////////////////////////////////////////////////
void Ogre2Material::SetAlphaFromTexture(bool _enabled,
    double _alpha, bool _twoSided)
{
  BaseMaterial::SetAlphaFromTexture(_enabled, _alpha, _twoSided);
  if (_enabled)
  {
    this->ogreDatablock->setAlphaTest(Ogre::CMPF_GREATER_EQUAL);
    Ogre::HlmsBlendblock block;
    block.setBlendType(Ogre::SBT_TRANSPARENT_ALPHA);
    this->ogreDatablock->setBlendblock(block);
  }
  else
  {
    this->ogreDatablock->setAlphaTest(Ogre::CMPF_ALWAYS_PASS);
  }
  this->ogreDatablock->setAlphaTestThreshold(_alpha);
  this->ogreDatablock->setTwoSidedLighting(_twoSided);
}

//////////////////////////////////////////////////
float Ogre2Material::RenderOrder() const
{
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());
  return macroblock.mDepthBiasConstant;
}

//////////////////////////////////////////////////
void Ogre2Material::SetRenderOrder(const float _renderOrder)
{
  this->renderOrder = _renderOrder;
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());

  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::RenderSystem *renderSystem = root->getRenderSystem();

  if (renderSystem->isReverseDepth())
  {
    // Reverse depth needs 100x scale AND ends up being superior
    // See https://github.com/ignitionrobotics/ign-rendering/
    // issues/427#issuecomment-991800352
    // and see https://www.youtube.com/watch?v=s2XdH3fYUac
    macroblock.mDepthBiasConstant = _renderOrder * 100.0f;
  }
  else
  {
    macroblock.mDepthBiasConstant = _renderOrder;
  }
  this->ogreDatablock->setMacroblock(macroblock);
}

//////////////////////////////////////////////////
bool Ogre2Material::ReceiveShadows() const
{
  return this->ogreDatablock->getReceiveShadows();
}

//////////////////////////////////////////////////
void Ogre2Material::SetReceiveShadows(const bool _receiveShadows)
{
  this->ogreDatablock->setReceiveShadows(_receiveShadows);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasTexture() const
{
  return !this->textureName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::Texture() const
{
  return this->textureName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetTexture(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearTexture();
    return;
  }

  this->textureName = _name;
  this->SetTextureMapImpl(this->textureName, Ogre::PBSM_DIFFUSE);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearTexture()
{
  this->textureName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_DIFFUSE, this->textureName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasNormalMap() const
{
  return !this->normalMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::NormalMap() const
{
  return this->normalMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetNormalMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearNormalMap();
    return;
  }

  this->normalMapName = _name;
  this->SetTextureMapImpl(this->normalMapName, Ogre::PBSM_NORMAL);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearNormalMap()
{
  this->normalMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_NORMAL, this->normalMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasRoughnessMap() const
{
  return !this->roughnessMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::RoughnessMap() const
{
  return this->roughnessMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetRoughnessMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearRoughnessMap();
    return;
  }

  this->roughnessMapName = _name;
  this->SetTextureMapImpl(this->roughnessMapName, Ogre::PBSM_ROUGHNESS);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearRoughnessMap()
{
  this->roughnessMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_ROUGHNESS, this->roughnessMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasMetalnessMap() const
{
  return !this->metalnessMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::MetalnessMap() const
{
  return this->metalnessMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetMetalnessMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearMetalnessMap();
    return;
  }

  this->metalnessMapName = _name;
  this->SetTextureMapImpl(this->metalnessMapName, Ogre::PBSM_METALLIC);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearMetalnessMap()
{
  this->metalnessMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_METALLIC, this->metalnessMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasEnvironmentMap() const
{
  return !this->environmentMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::EnvironmentMap() const
{
  return this->environmentMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetEnvironmentMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearEnvironmentMap();
    return;
  }

  this->environmentMapName = _name;
  this->SetTextureMapImpl(this->environmentMapName, Ogre::PBSM_REFLECTION);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearEnvironmentMap()
{
  this->environmentMapName = "";
  this->ogreDatablock->setTexture(
    Ogre::PBSM_REFLECTION, this->environmentMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasEmissiveMap() const
{
  return !this->emissiveMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::EmissiveMap() const
{
  return this->emissiveMapName;
}

//////////////////////////////////////////////////
void Ogre2Material::SetEmissiveMap(const std::string &_name)
{
  if (_name.empty())
  {
    this->ClearEmissiveMap();
    return;
  }

  this->emissiveMapName = _name;
  this->SetTextureMapImpl(this->emissiveMapName, Ogre::PBSM_EMISSIVE);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearEmissiveMap()
{
  this->emissiveMapName = "";
  this->ogreDatablock->setTexture(Ogre::PBSM_EMISSIVE, this->emissiveMapName);
}

//////////////////////////////////////////////////
bool Ogre2Material::HasLightMap() const
{
  return !this->lightMapName.empty();
}

//////////////////////////////////////////////////
std::string Ogre2Material::LightMap() const
{
  return this->lightMapName;
}

//////////////////////////////////////////////////
unsigned int Ogre2Material::LightMapTexCoordSet() const
{
  return this->lightMapUvSet;
}

//////////////////////////////////////////////////
void Ogre2Material::SetLightMap(const std::string &_name, unsigned int _uvSet)
{
  if (_name.empty())
  {
    this->ClearLightMap();
    return;
  }

  this->lightMapName = _name;
  this->lightMapUvSet = _uvSet;

  // in ign-rendering5 + ogre 2.1, we reserved detail map 0 for light map
  // and set a blend mode (PBSM_BLEND_OVERLAY AND PBSM_BLEND_MULTIPLY2X
  // produces better results) to blend with base albedo map. However, this
  // creates unwanted red highlights with ogre 2.2. So switching to use the
  // emissive map slot and calling setUseEmissiveAsLightmap(true)
  // Ogre::PbsTextureTypes type = Ogre::PBSM_DETAIL0;
  // this->ogreDatablock->setDetailMapBlendMode(0, Ogre::PBSM_BLEND_OVERLAY);
  Ogre::PbsTextureTypes type = Ogre::PBSM_EMISSIVE;

  // lightmap usually uses a different tex coord set
  this->SetTextureMapImpl(this->lightMapName, type);
  this->ogreDatablock->setTextureUvSource(type, this->lightMapUvSet);
  this->ogreDatablock->setUseEmissiveAsLightmap(true);
}

//////////////////////////////////////////////////
void Ogre2Material::ClearLightMap()
{
  this->lightMapName = "";
  this->lightMapUvSet = 0u;

  // in ogre 2.2, we swtiched to use the emissive map slot for light map
  if (this->ogreDatablock->getUseEmissiveAsLightmap())
    this->ogreDatablock->setTexture(Ogre::PBSM_EMISSIVE, this->lightMapName);
  this->ogreDatablock->setUseEmissiveAsLightmap(false);
}

//////////////////////////////////////////////////
void Ogre2Material::SetRoughness(const float _roughness)
{
  this->ogreDatablock->setRoughness(_roughness);
}

//////////////////////////////////////////////////
float Ogre2Material::Roughness() const
{
  return this->ogreDatablock->getRoughness();
}

//////////////////////////////////////////////////
void Ogre2Material::SetMetalness(const float _metalness)
{
  this->ogreDatablock->setMetalness(_metalness);
}

//////////////////////////////////////////////////
float Ogre2Material::Metalness() const
{
  return this->ogreDatablock->getMetalness();
}

//////////////////////////////////////////////////
void Ogre2Material::PreRender()
{
  this->UpdateShaderParams();
}

//////////////////////////////////////////////////
void Ogre2Material::UpdateShaderParams()
{
  if (this->dataPtr->vertexShaderParams &&
      this->dataPtr->vertexShaderParams->IsDirty())
  {
    Ogre::GpuProgramParametersSharedPtr ogreParams;
    auto mat = this->Material();
    auto pass = mat->getTechnique(0u)->getPass(0);
    ogreParams = pass->getVertexProgramParameters();
    this->UpdateShaderParams(this->dataPtr->vertexShaderParams, ogreParams);
    this->dataPtr->vertexShaderParams->ClearDirty();
  }
  if (this->dataPtr->fragmentShaderParams &&
      this->dataPtr->fragmentShaderParams->IsDirty())
  {
    Ogre::GpuProgramParametersSharedPtr ogreParams;
    auto mat = this->Material();
    auto pass = mat->getTechnique(0u)->getPass(0);
    ogreParams = pass->getFragmentProgramParameters();
    this->UpdateShaderParams(this->dataPtr->fragmentShaderParams, ogreParams);
    this->dataPtr->fragmentShaderParams->ClearDirty();
  }
}

//////////////////////////////////////////////////
void Ogre2Material::UpdateShaderParams(ConstShaderParamsPtr _params,
    Ogre::GpuProgramParametersSharedPtr _ogreParams)
{
  for (const auto &name_param : *_params)
  {
    auto *constantDef =
        Ogre::GpuProgramParameters::getAutoConstantDefinition(name_param.first);
    if (constantDef)
    {
      _ogreParams->setNamedAutoConstant(name_param.first, constantDef->acType);
      continue;
    }

    if (!_ogreParams->_findNamedConstantDefinition(name_param.first) &&
        !(Ogre2RenderEngine::Instance()->GraphicsAPI() !=
            GraphicsAPI::OPENGL &&
            (ShaderParam::PARAM_TEXTURE == name_param.second.Type() ||
             ShaderParam::PARAM_TEXTURE_CUBE == name_param.second.Type())))
    {
      ignwarn << "Unable to find GPU program parameter: "
              << name_param.first << std::endl;
      continue;
    }

    if (ShaderParam::PARAM_FLOAT == name_param.second.Type())
    {
      float value;
      name_param.second.Value(&value);
      _ogreParams->setNamedConstant(name_param.first, value);
    }
    else if (ShaderParam::PARAM_INT == name_param.second.Type())
    {
      int value;
      name_param.second.Value(&value);
      _ogreParams->setNamedConstant(name_param.first, value);
    }
    else if (ShaderParam::PARAM_FLOAT_BUFFER == name_param.second.Type())
    {
      std::shared_ptr<void> buffer;
      name_param.second.Buffer(buffer);
      uint32_t count = name_param.second.Count();

      // multiple other than 4 is currently only supported by GLSL
      uint32_t multiple = 1;
      _ogreParams->setNamedConstant(name_param.first,
          reinterpret_cast<float*>(buffer.get()), count, multiple);
    }
    else if (ShaderParam::PARAM_INT_BUFFER == name_param.second.Type())
    {
      std::shared_ptr<void> buffer;
      name_param.second.Buffer(buffer);
      uint32_t count = name_param.second.Count();

      // multiple other than 4 is currently only supported by GLSL
      uint32_t multiple = 1;
      _ogreParams->setNamedConstant(name_param.first,
        reinterpret_cast<int*>(buffer.get()), count, multiple);
    }
    else if (ShaderParam::PARAM_TEXTURE == name_param.second.Type() ||
             ShaderParam::PARAM_TEXTURE_CUBE == name_param.second.Type())
    {
      // add the textures to the resource path
      std::string value;
      uint32_t uvSetIndex = 0;
      name_param.second.Value(value, uvSetIndex);
      ShaderParam::ParamType type = name_param.second.Type();

      std::string baseName = value;
      std::string dirPath = value;
      if (common::isFile(value))
      {
        baseName = common::basename(value);
        size_t idx = value.rfind(baseName);
        if (idx != std::string::npos)
        {
          dirPath = value.substr(0, idx);
          if (!dirPath.empty() &&
            !Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(
            dirPath))
          {
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                dirPath, "FileSystem", "General");
          }
        }
      }
      else
      {
        ignerr << "Shader param texture not found: " << value << std::endl;
        continue;
      }

      // get the material and create the texture unit state it does not exist
      auto mat = this->Material();
      auto pass = mat->getTechnique(0u)->getPass(0);
      auto texUnit = pass->getTextureUnitState(name_param.first);
      if (!texUnit)
      {
        texUnit = pass->createTextureUnitState();
        texUnit->setName(name_param.first);
      }
      // make sure to cast to int before calling setNamedConstant later
      // to set the texture index
      int texIndex = static_cast<int>(pass->getTextureUnitStateIndex(texUnit));

      // set texture coordinate set
      texUnit->setTextureCoordSet(uvSetIndex);

      // set to wrap mode otherwise default is clamp mode
      Ogre::HlmsSamplerblock samplerBlockRef;
      samplerBlockRef.mU = Ogre::TAM_WRAP;
      samplerBlockRef.mV = Ogre::TAM_WRAP;
      samplerBlockRef.mW = Ogre::TAM_WRAP;
      texUnit->setSamplerblock(samplerBlockRef);

      // regular 2d texture
      if (type == ShaderParam::ParamType::PARAM_TEXTURE)
      {
        texUnit->setTextureName(baseName, Ogre::TextureTypes::Type2D);
      }
      // cube maps
      else if (type == ShaderParam::ParamType::PARAM_TEXTURE_CUBE)
      {
        texUnit->setCubicTextureName(baseName, true);
      }
      else
      {
        ignerr << "Unrecognized texture type set for shader param: "
               << name_param.first << std::endl;
        continue;
      }
      // set the texture map index
      _ogreParams->setNamedConstant(name_param.first, &texIndex, 1, 1);
    }
  }
}

//////////////////////////////////////////////////
Ogre::MaterialPtr Ogre2Material::Material()
{
  if (!this->ogreMaterial)
  {
    // low level ogre material used by render targets
    Ogre::MaterialManager &matManager = Ogre::MaterialManager::getSingleton();
    this->ogreMaterial = matManager.create(this->name,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  }

  return this->ogreMaterial;
}

//////////////////////////////////////////////////
Ogre::HlmsPbsDatablock *Ogre2Material::Datablock() const
{
  return this->ogreDatablock;
}

//////////////////////////////////////////////////
void Ogre2Material::SetTextureMapImpl(const std::string &_texture,
  Ogre::PbsTextureTypes _type)
{
  // FIXME(anyone) need to keep baseName = _texture for all meshes. Refer to
  // https://github.com/ignitionrobotics/ign-rendering/issues/139
  // for more details
  std::string baseName = _texture;
  if (common::isFile(_texture))
  {
    baseName = common::basename(_texture);
    size_t idx = _texture.rfind(baseName);
    if (idx != std::string::npos)
    {
      std::string dirPath = _texture.substr(0, idx);
      if (!dirPath.empty() &&
          !Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(
          dirPath))
      {
        Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
            dirPath, "FileSystem", "General");
      }
    }
  }
  else
  {
    return;
  }

  // temp workaround check if the model is a OBJ file
  {
    size_t idx = _texture.rfind("meshes");
    if (idx != std::string::npos)
    {
      std::string objFile =
        common::joinPaths(_texture.substr(0, idx), "meshes", "model.obj");
      if (common::isFile(objFile))
        baseName = _texture;
    }
  }

  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::TextureGpuManager *textureMgr =
      root->getRenderSystem()->getTextureGpuManager();

  // workaround for grayscale emissive texture
  // convert to RGB otherwise the emissive map is rendered red
  if (_type == Ogre::PBSM_EMISSIVE &&
      !this->ogreDatablock->getUseEmissiveAsLightmap())
  {
    common::Image img(_texture);
    // check for 8 bit pixel
    if (img.BPP() == 8u)
    {
      std::string parentPath = common::parentPath(_texture);
      // set a custom name for the rgb texture by appending ign_ prefix
      std::string rgbTexName = "ign_" + baseName;
      baseName = rgbTexName;
      auto tex = textureMgr->findTextureNoThrow(rgbTexName);
      if (!tex)
      {
        ignmsg << "Grayscale emissive texture detected. Converting to RGB: "
               << rgbTexName << std::endl;
        // need to be 4 channels for gpu texture
        unsigned int channels = 4u;
        unsigned int size = img.Width() * img.Height() * channels;
        unsigned char *data = new unsigned char[size];
        for (unsigned int i = 0; i < img.Height(); ++i)
        {
          for (unsigned int j = 0; j < img.Width(); ++j)
          {
            // flip Y
            math::Color c = img.Pixel(j, img.Height() - i - 1u);
            unsigned int idx = i * img.Width() * channels + j * channels;
            data[idx] = static_cast<uint8_t>(c.R() * 255u);
            data[idx + 1u] = static_cast<uint8_t>(c.R() * 255u);
            data[idx + 2u] = static_cast<uint8_t>(c.R() * 255u);
            data[idx + 3u] = 255u;
          }
        }

        // create the gpu texture
        Ogre::uint32 textureFlags = 0;
        textureFlags |= Ogre::TextureFlags::AutomaticBatching;
        if (this->ogreDatablock->suggestUsingSRGB(_type))
            textureFlags |= Ogre::TextureFlags::PrefersLoadingFromFileAsSRGB;
        Ogre::TextureGpu *texture = textureMgr->createOrRetrieveTexture(
            rgbTexName,
            Ogre::GpuPageOutStrategy::Discard,
            textureFlags | Ogre::TextureFlags::ManualTexture,
            Ogre::TextureTypes::Type2D,
            Ogre::ResourceGroupManager::AUTODETECT_RESOURCE_GROUP_NAME,
            0u);

        texture->setPixelFormat(Ogre::PFG_RGBA8_UNORM_SRGB);
        texture->setTextureType(Ogre::TextureTypes::Type2D);
        texture->setNumMipmaps(1u);
        texture->setResolution(img.Width(), img.Height());
        texture->scheduleTransitionTo(Ogre::GpuResidency::Resident);
        texture->waitForData();

        // upload raw color image data to gpu texture
        Ogre::Image2 image;
        image.loadDynamicImage(data, false, texture);
        image.uploadTo(texture, 0, 0);
        delete [] data;
      }
    }
  }

  Ogre::HlmsSamplerblock samplerBlockRef;
  samplerBlockRef.mU = Ogre::TAM_WRAP;
  samplerBlockRef.mV = Ogre::TAM_WRAP;
  samplerBlockRef.mW = Ogre::TAM_WRAP;

  this->ogreDatablock->setTexture(_type, baseName, &samplerBlockRef);
  auto tex = textureMgr->findTextureNoThrow(baseName);

  if (tex)
  {
    tex->waitForMetadata();
    this->dataPtr->hashName = tex->getName().getFriendlyText();
  }

  // disable alpha from texture if texture does not have an alpha channel
  // otherwise this becomes a transparent material
  if (_type == Ogre::PBSM_DIFFUSE)
  {
    bool isGrayscale = (Ogre::PixelFormatGpuUtils::getNumberOfComponents(
            tex->getPixelFormat()) == 1u);

    if (this->TextureAlphaEnabled() || isGrayscale)
    {
      if (tex)
      {
        tex->scheduleTransitionTo(Ogre::GpuResidency::Resident);
        tex->waitForData();

        // only enable alpha from texture if texture has alpha component
        if (this->TextureAlphaEnabled() &&
            !Ogre::PixelFormatGpuUtils::hasAlpha(tex->getPixelFormat()))
        {
          this->SetAlphaFromTexture(false, this->AlphaThreshold(),
              this->TwoSidedEnabled());
        }

        // treat grayscale texture as RGB
        if (isGrayscale)
        {
          this->ogreDatablock->setUseDiffuseMapAsGrayscale(true);
        }
      }
    }
  }
}

//////////////////////////////////////////////////////
Ogre::TextureGpu* Ogre2Material::Texture(const std::string &_name)
{
  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::TextureGpuManager *textureMgr =
    root->getRenderSystem()->getTextureGpuManager();

  auto tex = textureMgr->findTextureNoThrow(_name);
  return tex;
}

//////////////////////////////////////////////////
void Ogre2Material::Init()
{
  BaseMaterial::Init();

  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::HlmsManager *hlmsManager = root->getHlmsManager();
  this->ogreHlmsPbs = static_cast<Ogre::HlmsPbs*>(
      hlmsManager->getHlms(Ogre::HLMS_PBS));

  if (!this->ogreHlmsPbs)
  {
    ignerr << "Ogre HLMS PBS not ready. Is Ogre2 Render Engine initiallized?"
           << std::endl;
    return;
  }
  this->ogreDatablockId = this->Scene()->Name() + "::" + this->name;
  this->ogreDatablock = static_cast<Ogre::HlmsPbsDatablock *>(
      this->ogreHlmsPbs->createDatablock(
      this->ogreDatablockId, this->name,
      Ogre::HlmsMacroblock(), Ogre::HlmsBlendblock(), Ogre::HlmsParamVec()));

  // use metal workflow as default
  this->ogreDatablock->setWorkflow(Ogre::HlmsPbsDatablock::MetallicWorkflow);

  this->Reset();
}

//////////////////////////////////////////////////
enum MaterialType Ogre2Material::Type() const
{
  return MaterialType::MT_PBS;
}

//////////////////////////////////////////////////
bool Ogre2Material::DepthCheckEnabled() const
{
  const Ogre::HlmsMacroblock *macroblock = this->ogreDatablock->getMacroblock();
  return macroblock->mDepthCheck;
}

//////////////////////////////////////////////////
void Ogre2Material::SetDepthCheckEnabled(bool _enabled)
{
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());
  macroblock.mDepthCheck = _enabled;
  this->ogreDatablock->setMacroblock(macroblock);
}

//////////////////////////////////////////////////
bool Ogre2Material::DepthWriteEnabled() const
{
  const Ogre::HlmsMacroblock *macroblock = this->ogreDatablock->getMacroblock();
  return macroblock->mDepthWrite;
}

//////////////////////////////////////////////////
void Ogre2Material::SetDepthWriteEnabled(bool _enabled)
{
  Ogre::HlmsMacroblock macroblock(
      *this->ogreDatablock->getMacroblock());
  macroblock.mDepthWrite = _enabled;
  this->ogreDatablock->setMacroblock(macroblock);
}

//////////////////////////////////////////////////
Ogre::HlmsUnlitDatablock *Ogre2Material::UnlitDatablock()
{
  if (!this->ogreUnlitDatablock)
  {
    // Create the internal UNLIT material.
    Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
    Ogre::HlmsManager *hlmsManager = root->getHlmsManager();
    Ogre::HlmsUnlit *ogreHlmsUnlit = static_cast<Ogre::HlmsUnlit *>(
        hlmsManager->getHlms(Ogre::HLMS_UNLIT));
    if (!ogreHlmsUnlit)
    {
      ignerr << "Ogre HLMS UNLIT not ready. Is Ogre2 Render Engine "
             << "initiallized?" << std::endl;
      return nullptr;
    }

    std::string unlitName = this->name + "::Unlit";
    this->ogreUnlitDatablock =
        static_cast<Ogre::HlmsUnlitDatablock *>(
        ogreHlmsUnlit->createDatablock(
        unlitName, unlitName,
        Ogre::HlmsMacroblock(), Ogre::HlmsBlendblock(), Ogre::HlmsParamVec()));
  }

  this->FillUnlitDatablock(this->ogreUnlitDatablock);
  return this->ogreUnlitDatablock;
}

//////////////////////////////////////////////////
void Ogre2Material::FillUnlitDatablock(Ogre::HlmsUnlitDatablock *_datablock)
    const
{
  if (!this->textureName.empty())
  {
    std::string baseName = common::basename(this->textureName);

    Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();
    Ogre::TextureGpuManager *textureMgr =
      root->getRenderSystem()->getTextureGpuManager();
    Ogre::TextureGpu *texture = textureMgr->createOrRetrieveTexture(baseName,
          Ogre::GpuPageOutStrategy::Discard,
          Ogre::TextureFlags::ManualTexture,
          Ogre::TextureTypes::Type2D);

    _datablock->setTexture(0, texture);
  }

  auto samplerblock = this->ogreDatablock->getSamplerblock(Ogre::PBSM_DIFFUSE);
  if (samplerblock)
    _datablock->setSamplerblock(0, *samplerblock);
  _datablock->setMacroblock(this->ogreDatablock->getMacroblock());
  _datablock->setBlendblock(this->ogreDatablock->getBlendblock());

  _datablock->setUseColour(true);
  Ogre::Vector3 c = this->ogreDatablock->getDiffuse();
  _datablock->setColour(Ogre::ColourValue(c.x, c.y, c.z));
}

//////////////////////////////////////////////////
void Ogre2Material::SetVertexShader(const std::string &_path)
{
  if (_path.empty())
    return;

  if (!common::exists(_path))
  {
    ignerr << "Vertex shader path does not exist: " << _path << std::endl;
    return;
  }

  std::string baseName = common::basename(_path);
  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(_path,
  "FileSystem", "General", false);

  Ogre::HighLevelGpuProgramPtr vertexShader =
    Ogre::HighLevelGpuProgramManager::getSingletonPtr()->createProgram(
        "_ign_" + baseName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        "glsl", Ogre::GpuProgramType::GPT_VERTEX_PROGRAM);

  vertexShader->setSourceFile(_path);

  Ogre::GpuProgramParametersSharedPtr params =
      vertexShader->getDefaultParameters();

  vertexShader->load();

  assert(vertexShader->isLoaded());
  assert(!(vertexShader->hasCompileError()));
  assert(vertexShader->isSupported());

  auto mat = this->Material();
  auto pass = mat->getTechnique(0u)->getPass(0);
  pass->setVertexProgram(vertexShader->getName());
  mat->compile();
  mat->load();

  this->dataPtr->vertexShaderPath = _path;
  this->dataPtr->vertexShaderParams.reset(new ShaderParams);
}

//////////////////////////////////////////////////
std::string Ogre2Material::VertexShader() const
{
  return this->dataPtr->vertexShaderPath;
}

//////////////////////////////////////////////////
ShaderParamsPtr Ogre2Material::VertexShaderParams()
{
  return this->dataPtr->vertexShaderParams;
}

//////////////////////////////////////////////////
void Ogre2Material::SetFragmentShader(const std::string &_path)
{
  if (_path.empty())
    return;

  if (!common::exists(_path))
  {
    ignerr << "Fragment shader path does not exist: " << _path << std::endl;
    return;
  }

  Ogre::ResourceGroupManager::getSingleton().addResourceLocation(_path,
  "FileSystem", "General", false);

  std::string baseName = common::basename(_path);
  Ogre::HighLevelGpuProgramPtr fragmentShader =
    Ogre::HighLevelGpuProgramManager::getSingleton().createProgram(
        "_ign_" + baseName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        "glsl", Ogre::GpuProgramType::GPT_FRAGMENT_PROGRAM);

  fragmentShader->setSourceFile(_path);
  fragmentShader->load();

  assert(fragmentShader->isLoaded());
  assert(!(fragmentShader->hasCompileError()));
  assert(fragmentShader->isSupported());

  auto mat = this->Material();
  auto pass = mat->getTechnique(0u)->getPass(0);
  pass->setFragmentProgram(fragmentShader->getName());
  mat->compile();
  mat->load();
  this->dataPtr->fragmentShaderPath = _path;
  this->dataPtr->fragmentShaderParams.reset(new ShaderParams);
}

//////////////////////////////////////////////////
std::string Ogre2Material::FragmentShader() const
{
  return this->dataPtr->fragmentShaderPath;
}

//////////////////////////////////////////////////
ShaderParamsPtr Ogre2Material::FragmentShaderParams()
{
  return this->dataPtr->fragmentShaderParams;
}
