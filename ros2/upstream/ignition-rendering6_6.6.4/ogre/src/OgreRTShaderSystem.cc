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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <mutex>
#include <set>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/ogre/OgreRenderEngine.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreMesh.hh"
#include "gz/rendering/ogre/OgreRTShaderSystem.hh"

class gz::rendering::OgreRTShaderSystemPrivate
{
  /// \brief The shader generator.
  public: Ogre::RTShader::ShaderGenerator *shaderGenerator = nullptr;

  /// \brief Used to generate shadows.
  public: Ogre::RTShader::SubRenderState *shadowRenderState = nullptr;

  /// \brief All the entites being used.
  public: std::set<OgreSubMesh *> entities;

  /// \brief True if initialized.
  public: bool initialized;

  /// \brief True if shadows have been applied.
  public: bool shadowsApplied;

  /// \brief All the scenes.
  public: std::vector<OgreScenePtr> scenes;

  /// \brief Mutex used to protext the entities list.
  public: std::mutex entityMutex;

  /// \brief Parallel Split Shadow Map (PSSM) camera setup
  public: Ogre::ShadowCameraSetupPtr pssmSetup;

  /// \brief Flag to indicate that shaders need to be updated.
  public: bool updateShaders = false;

  /// \brief Size of the Parallel Split Shadow Map (PSSM) shadow texture
  /// at closest layer.
  public: unsigned int shadowTextureSize = 1024u;

  /// \brief Flag to indicate shadows need to be reapplied
  public: bool resetShadows = false;

  /// \brief thread that shader system is created in
  public: std::thread::id threadId;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreRTShaderSystem::OgreRTShaderSystem()
  : dataPtr(new OgreRTShaderSystemPrivate)
{
  this->dataPtr->initialized = false;
  this->dataPtr->shadowsApplied = false;
#if OGRE_VERSION_LT_1_11_0
  this->dataPtr->pssmSetup.setNull();
#else
  this->dataPtr->pssmSetup.reset();
#endif

  this->dataPtr->threadId = std::this_thread::get_id();
}

//////////////////////////////////////////////////
OgreRTShaderSystem::~OgreRTShaderSystem()
{
  if (std::this_thread::get_id() == this->dataPtr->threadId)
  {
    this->Fini();
  }
}

//////////////////////////////////////////////////
bool OgreRTShaderSystem::Init()
{
  // Only initialize if using FORWARD rendering
  if (OgreRenderEngine::Instance()->RenderPathType() !=
      OgreRenderEngine::FORWARD)
  {
    return false;
  }

  if (Ogre::RTShader::ShaderGenerator::initialize())
  {
    std::string coreLibsPath, cachePath;
    if (!this->Paths(coreLibsPath, cachePath))
    {
      ignerr << "Cannot find OGRE rtshaderlib. "
             << "Shadows will be disabled." << std::endl;
      return false;
    }

    this->dataPtr->initialized = true;
    // Get the shader generator pointer
    this->dataPtr->shaderGenerator =
        Ogre::RTShader::ShaderGenerator::getSingletonPtr();

    // Add the shader libs resource location
    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
        coreLibsPath, "FileSystem");

    // Set shader cache path.
    this->dataPtr->shaderGenerator->setShaderCachePath(cachePath);

    this->dataPtr->shaderGenerator->setTargetLanguage("glsl");
  }
  else
  {
    ignerr << "RT Shader system failed to initialize" << std::endl;
    return false;
  }
  return false;
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::Fini()
{
  if (!this->dataPtr->initialized)
    return;

  // Restore default scheme.
  Ogre::MaterialManager::getSingleton().setActiveScheme(
      Ogre::MaterialManager::DEFAULT_SCHEME_NAME);

  // Finalize RTShader system.
  if (this->dataPtr->shaderGenerator != nullptr)
  {
    // On Windows, we're using 1.9RC1, which doesn't have a bunch of changes.
#if (OGRE_VERSION < ((1 << 16) | (9 << 8) | 0))
    Ogre::RTShader::ShaderGenerator::finalize();
#else
    Ogre::RTShader::ShaderGenerator::destroy();
#endif
    this->dataPtr->shaderGenerator = nullptr;
  }
#if OGRE_VERSION_LT_1_11_0
  this->dataPtr->pssmSetup.setNull();
#else
  this->dataPtr->pssmSetup.reset();
#endif
  this->dataPtr->entities.clear();
  this->dataPtr->scenes.clear();
  this->dataPtr->shadowsApplied = false;
  this->dataPtr->initialized = false;
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::AddScene(OgreScenePtr _scene)
{
  if (!this->dataPtr->initialized)
    return;

  // Set the scene manager
  this->dataPtr->shaderGenerator->addSceneManager(_scene->OgreSceneManager());
  this->dataPtr->shaderGenerator->createScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  this->dataPtr->scenes.push_back(_scene);
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::RemoveScene(OgreScenePtr _scene)
{
  if (!this->dataPtr->initialized)
    return;

  std::vector<OgreScenePtr>::iterator iter;
  for (iter = this->dataPtr->scenes.begin();
      iter != this->dataPtr->scenes.end(); ++iter)
    if ((*iter) == _scene)
      break;

  if (iter != this->dataPtr->scenes.end())
  {
    this->dataPtr->scenes.erase(iter);
    this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
    this->dataPtr->shaderGenerator->removeSceneManager(
        _scene->OgreSceneManager());
    this->dataPtr->shaderGenerator->removeAllShaderBasedTechniques();
    this->dataPtr->shaderGenerator->flushShaderCache();
  }
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::RemoveScene(const std::string &_scene)
{
  if (!this->dataPtr->initialized)
    return;

  for (auto iter : this->dataPtr->scenes)
  {
    if (iter->Name() == _scene)
    {
      this->RemoveScene(iter);
      return;
    }
  }
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::AttachEntity(OgreSubMesh *_subMesh)
{
  if (!this->dataPtr->initialized)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->entityMutex);
  this->dataPtr->entities.insert(_subMesh);
  this->dataPtr->updateShaders = true;
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::DetachEntity(OgreSubMesh *_subMesh)
{
  if (!this->dataPtr->initialized)
    return;

  std::lock_guard<std::mutex> lock(this->dataPtr->entityMutex);
  // Remove shaders
  auto it = this->dataPtr->entities.find(_subMesh);
  if (it != this->dataPtr->entities.end())
  {
    this->RemoveShaders(_subMesh);
    this->dataPtr->entities.erase(it);
  }
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::Clear()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->entityMutex);
  this->dataPtr->entities.clear();
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::AttachViewport(Ogre::Viewport *_viewport,
    OgreScenePtr _scene)
{
  if (!OgreRTShaderSystem::Instance()->IsInitialized() || _scene == nullptr ||
      _viewport == nullptr)
    return;

  _viewport->setMaterialScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::DetachViewport(Ogre::Viewport *_viewport,
    OgreScenePtr _scene)
{
  if (!OgreRTShaderSystem::Instance()->IsInitialized())
    return;

  // (louise) Is this "detaching" the viewport?
  if (_viewport && _scene && _scene->IsInitialized())
    _viewport->setMaterialScheme(_scene->Name());
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::UpdateShaders()
{
  this->dataPtr->updateShaders = true;
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::RemoveShaders(OgreSubMesh *_subMesh)
{
  if (!this->dataPtr->initialized)
    return;

  Ogre::SubEntity *curSubEntity = _subMesh->OgreSubEntity();
  const Ogre::String &curMaterialName = curSubEntity->getMaterialName();
  for (const auto &s : this->dataPtr->scenes)
  {
    try
    {
#ifdef OGRE_VERSION_LT_1_12_0
      this->dataPtr->shaderGenerator->removeShaderBasedTechnique(
          curMaterialName,
      #ifndef OGRE_VERSION_LT_1_11_0
          curSubEntity->getMaterial()->getGroup(),
      #endif
          Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
          s->Name() +
          Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
#else
      auto mat = curSubEntity->getMaterial();

      // get source technique from technique scheme name
      // see findSourceTechnique in
      // ogre/Components/RTShaderSystem/src/OgreShaderGenerator.cpp
      Ogre::Technique* srcTechnique = nullptr;
      std::string srcTechniqueSchemeName =
          Ogre::MaterialManager::DEFAULT_SCHEME_NAME;
      for (auto it = mat->getTechniques().begin();
          it != mat->getTechniques().end(); ++it)
      {
        Ogre::Technique *curTechnique = *it;

        if (curTechnique->getSchemeName() == srcTechniqueSchemeName)
        {
          bool hasFixedFunctionPass = false;
          for (unsigned int i = 0; i < curTechnique->getNumPasses(); ++i)
          {
            if (!curTechnique->getPass(i)->isProgrammable())
            {
              hasFixedFunctionPass = true;
              break;
            }
          }
          if (hasFixedFunctionPass)
          {
            srcTechnique = curTechnique;
            break;
          }
        }
      }

      this->dataPtr->shaderGenerator->removeShaderBasedTechnique(srcTechnique,
          s->Name() + Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
#endif
    }
    catch(Ogre::Exception &e)
    {
      ignerr << "Unable to remove shader technique for material["
        << curMaterialName << "]\n";
    }
  }
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::GenerateShaders(OgreSubMesh *subMesh)
{
  if (!this->dataPtr->initialized)
  {
    return;
  }

  Ogre::SubEntity* curSubEntity = subMesh->OgreSubEntity();

  OgreMaterialPtr material =
      std::dynamic_pointer_cast<OgreMaterial>(subMesh->Material());

  if (!material)
  {
    return;
  }

  std::string shaderTypeName = ShaderUtil::Name(material->ShaderType());
  std::string normalMapName = material->NormalMap();

  const Ogre::String& curMaterialName = curSubEntity->getMaterialName();
  bool success = false;

  for (unsigned int s = 0; s < this->dataPtr->scenes.size(); s++)
  {
    try
    {
      success = this->dataPtr->shaderGenerator->createShaderBasedTechnique(
#if OGRE_VERSION_LT_1_11_0
          curMaterialName,
#else
          *material->Material(),
#endif
          Ogre::MaterialManager::DEFAULT_SCHEME_NAME,
          this->dataPtr->scenes[s]->Name() +
          Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
    }
    catch(Ogre::Exception &e)
    {
      ignerr << "Unable to create shader technique for material["
        << curMaterialName << "]\n";
      success = false;
    }

    // Setup custom shader sub render states according to current setup.
    if (success)
    {
      // Grab the first pass render state.
      // NOTE:For more complicated samples iterate over the passes and build
      // each one of them as desired.
      Ogre::RTShader::RenderState* renderState =
        this->dataPtr->shaderGenerator->getRenderState(
            this->dataPtr->scenes[s]->Name() +
            Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
            curMaterialName,
#ifndef OGRE_VERSION_LT_1_11_0
            material->Material()->getGroup(),
#endif
            0);

      // Remove all sub render states.
      renderState->reset();

      if (shaderTypeName == "normal_map_object_space")
      {
        Ogre::RTShader::SubRenderState* subRenderState =
          this->dataPtr->shaderGenerator->createSubRenderState(
              Ogre::RTShader::NormalMapLighting::Type);

        Ogre::RTShader::NormalMapLighting* normalMapSubRS =
          static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

        normalMapSubRS->setNormalMapSpace(
            Ogre::RTShader::NormalMapLighting::NMS_OBJECT);

        normalMapSubRS->setNormalMapTextureName(normalMapName);
        renderState->addTemplateSubRenderState(normalMapSubRS);
      }
      else if (shaderTypeName == "normal_map_tangent_space")
      {
        Ogre::RTShader::SubRenderState* subRenderState =
          this->dataPtr->shaderGenerator->createSubRenderState(
              Ogre::RTShader::NormalMapLighting::Type);

        Ogre::RTShader::NormalMapLighting* normalMapSubRS =
          static_cast<Ogre::RTShader::NormalMapLighting*>(subRenderState);

        normalMapSubRS->setNormalMapSpace(
            Ogre::RTShader::NormalMapLighting::NMS_TANGENT);

        normalMapSubRS->setNormalMapTextureName(normalMapName);

        renderState->addTemplateSubRenderState(normalMapSubRS);
      }
      else if (shaderTypeName == "vertex")
      {
        Ogre::RTShader::SubRenderState *perPerVertexLightModel =
          this->dataPtr->shaderGenerator->createSubRenderState(
              Ogre::RTShader::FFPLighting::Type);

        renderState->addTemplateSubRenderState(perPerVertexLightModel);
      }
      else
      {
        Ogre::RTShader::SubRenderState *perPixelLightModel =
          this->dataPtr->shaderGenerator->createSubRenderState(
              Ogre::RTShader::PerPixelLighting::Type);

        renderState->addTemplateSubRenderState(perPixelLightModel);
      }


      // Invalidate this material in order to re-generate its shaders.
      this->dataPtr->shaderGenerator->invalidateMaterial(
          this->dataPtr->scenes[s]->Name() +
          Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME,
          curMaterialName);
    }
  }
}

//////////////////////////////////////////////////
bool OgreRTShaderSystem::Paths(std::string &coreLibsPath,
    std::string &cachePath)
{
  const char *env = std::getenv("IGN_RENDERING_RESOURCE_PATH");
  std::string resourcePath = (env) ? std::string(env) :
      IGN_RENDERING_RESOURCE_PATH;

  // path to look for ogre media files
  std::vector<std::string> paths;

  // install path
  std::string mediaPath = common::joinPaths(resourcePath, "ogre", "media",
      "rtshaderlib150");
  paths.push_back(mediaPath);

  // src path
  mediaPath = common::joinPaths(resourcePath, "ogre", "src", "media",
      "rtshaderlib150");
  paths.push_back(mediaPath);

  for (auto const &p : paths)
  {
    if (common::exists(p))
    {
      coreLibsPath = p;
      // setup patch name for rt shader cache in tmp
      const char *homeEnv = std::getenv(IGN_HOMEDIR);
      std::string tmpDir = (homeEnv) ? std::string(homeEnv) : std::string(".");

      tmpDir = common::joinPaths(tmpDir, ".ignition", "rendering",
          "ogre-rtshader");
      // Get the user
      std::string user = "nobody";
      const char* userEnv = std::getenv("USER");
      if (userEnv)
        user = std::string(userEnv);
      cachePath = common::joinPaths(tmpDir, user + "-rtshaderlibcache");
      // Create the directory
      if (!common::createDirectories(cachePath))
      {
        ignerr << "Unable to create ogre RTShader cache directories: "
            << cachePath << std::endl;
      }
      break;
    }
  }

  // Core shader lib not found -> shader generating will fail.
  if (coreLibsPath.empty())
  {
    ignerr << "Unable to find shader lib. Shader generating will fail."
      << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::RemoveShadows(OgreScenePtr _scene)
{
  if (!this->dataPtr->initialized || !this->dataPtr->shadowsApplied)
    return;

  _scene->OgreSceneManager()->setShadowTechnique(Ogre::SHADOWTYPE_NONE);

  _scene->OgreSceneManager()->setShadowCameraSetup(
      Ogre::ShadowCameraSetupPtr());

  Ogre::RTShader::RenderState* schemeRenderState =
    this->dataPtr->shaderGenerator->getRenderState(
        _scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  schemeRenderState->removeTemplateSubRenderState(
      this->dataPtr->shadowRenderState);

  this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);
  this->UpdateShaders();

  this->dataPtr->shadowsApplied = false;
}

//////////////////////////////////////////////////
void OgreRTShaderSystem::ApplyShadows(OgreScenePtr _scene)
{
  if (!this->dataPtr->initialized || this->dataPtr->shadowsApplied)
    return;

  Ogre::SceneManager *sceneMgr = _scene->OgreSceneManager();

  // Grab the scheme render state.
  Ogre::RTShader::RenderState* schemRenderState =
    this->dataPtr->shaderGenerator->getRenderState(_scene->Name() +
        Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_ADDITIVE_INTEGRATED);

  // 3 textures per directional light
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_DIRECTIONAL, 3);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_POINT, 0);
  sceneMgr->setShadowTextureCountPerLightType(Ogre::Light::LT_SPOTLIGHT, 0);
  sceneMgr->setShadowTextureCount(3);
  sceneMgr->setShadowTextureConfig(0,
      this->dataPtr->shadowTextureSize, this->dataPtr->shadowTextureSize,
      Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureConfig(1,
      this->dataPtr->shadowTextureSize/2, this->dataPtr->shadowTextureSize/2,
      Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureConfig(2,
      this->dataPtr->shadowTextureSize/2, this->dataPtr->shadowTextureSize/2,
      Ogre::PF_FLOAT32_R);
  sceneMgr->setShadowTextureSelfShadow(false);
  sceneMgr->setShadowCasterRenderBackFaces(true);

#if OGRE_VERSION_LT_1_11_0
  // Set up caster material - this is just a standard depth/shadow map caster
  sceneMgr->setShadowTextureCasterMaterial("PSSM/shadow_caster");
#else
  Ogre::MaterialPtr mat =
    Ogre::MaterialManager::getSingleton().getByName("PSSM/shadow_caster");
  sceneMgr->setShadowTextureCasterMaterial(mat);
#endif

  // Disable fog on the caster pass.
  //  Ogre::MaterialPtr passCaterMaterial =
  //   Ogre::MaterialManager::getSingleton().getByName("PSSM/shadow_caster");
  // Ogre::Pass* pssmCasterPass =
  // passCaterMaterial->getTechnique(0)->getPass(0);
  // pssmCasterPass->setFog(true);

  // shadow camera setup
#if OGRE_VERSION_LT_1_11_0
  if (this->dataPtr->pssmSetup.isNull())
#else
  if (this->dataPtr->pssmSetup == nullptr)
#endif
  {
    this->dataPtr->pssmSetup =
        Ogre::ShadowCameraSetupPtr(new Ogre::PSSMShadowCameraSetup());
  }

  double shadowFarDistance = 500;
  double cameraNearClip = 0.01;
  sceneMgr->setShadowFarDistance(shadowFarDistance);

  Ogre::PSSMShadowCameraSetup *cameraSetup =
      dynamic_cast<Ogre::PSSMShadowCameraSetup *>(
      this->dataPtr->pssmSetup.get());

  cameraSetup->calculateSplitPoints(3, cameraNearClip, shadowFarDistance);
  cameraSetup->setSplitPadding(4);
  cameraSetup->setOptimalAdjustFactor(0, 2);
  cameraSetup->setOptimalAdjustFactor(1, 1);
  cameraSetup->setOptimalAdjustFactor(2, .5);

  sceneMgr->setShadowCameraSetup(this->dataPtr->pssmSetup);

  // These values do not seem to help at all. Leaving here until I have time
  // to properly fix shadow z-fighting.
  // cameraSetup->setOptimalAdjustFactor(0, 4);
  // cameraSetup->setOptimalAdjustFactor(1, 1);
  // cameraSetup->setOptimalAdjustFactor(2, 0.5);

  this->dataPtr->shadowRenderState =
      this->dataPtr->shaderGenerator->createSubRenderState(
      Ogre::RTShader::IntegratedPSSM3::Type);
  Ogre::RTShader::IntegratedPSSM3 *pssm3SubRenderState =
      static_cast<Ogre::RTShader::IntegratedPSSM3 *>(
      this->dataPtr->shadowRenderState);

  const Ogre::PSSMShadowCameraSetup::SplitPointList &srcSplitPoints =
    cameraSetup->getSplitPoints();

  Ogre::RTShader::IntegratedPSSM3::SplitPointList dstSplitPoints;

  for (unsigned int i = 0; i < srcSplitPoints.size(); ++i)
  {
    dstSplitPoints.push_back(srcSplitPoints[i]);
  }

  pssm3SubRenderState->setSplitPoints(dstSplitPoints);
  schemRenderState->addTemplateSubRenderState(this->dataPtr->shadowRenderState);

  this->dataPtr->shaderGenerator->invalidateScheme(_scene->Name() +
      Ogre::RTShader::ShaderGenerator::DEFAULT_SCHEME_NAME);

  this->UpdateShaders();

  this->dataPtr->shadowsApplied = true;
}

//////////////////////////////////////////////////
Ogre::PSSMShadowCameraSetup
    *OgreRTShaderSystem::PSSMShadowCameraSetup() const
{
  return dynamic_cast<Ogre::PSSMShadowCameraSetup *>(
      this->dataPtr->pssmSetup.get());
}

/////////////////////////////////////////////////
void OgreRTShaderSystem::Update()
{
  if (!this->dataPtr->initialized)
    return;

  if (this->dataPtr->resetShadows)
  {
    for (auto & s : this->dataPtr->scenes)
    {
      this->RemoveShadows(s);
      this->ApplyShadows(s);
    }
    this->dataPtr->resetShadows = false;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->entityMutex);

  if (this->dataPtr->updateShaders)
  {
    // Update all the shaders
    for (auto iter = this->dataPtr->entities.begin();
        iter != this->dataPtr->entities.end(); ++iter)
      this->GenerateShaders(*iter);

    this->dataPtr->updateShaders = false;
  }
}

/////////////////////////////////////////////////
bool OgreRTShaderSystem::SetShadowTextureSize(const unsigned int _size)
{
  // check if texture size is a power of 2
  if (!math::isPowerOfTwo(_size))
  {
    ignerr << "Shadow texture size must be a power of 2" << std::endl;
    return false;
  }

  this->dataPtr->shadowTextureSize = _size;
  this->dataPtr->resetShadows = true;
  return true;
}

/////////////////////////////////////////////////
unsigned int OgreRTShaderSystem::ShadowTextureSize() const
{
  return this->dataPtr->shadowTextureSize;
}

/////////////////////////////////////////////////
bool OgreRTShaderSystem::IsInitialized() const
{
  return this->dataPtr->initialized;
}
