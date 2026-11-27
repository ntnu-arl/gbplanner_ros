/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#if (_WIN32)
  /* Needed for std::min */
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
#endif

#ifdef __APPLE__
  #define GL_SILENCE_DEPRECATION
  #include <OpenGL/gl.h>
#else
#ifndef _WIN32
  #include <GL/gl.h>
#endif
#endif

#include <math.h>

#include <algorithm>
#include <limits>
#include <string>
#include <unordered_map>
#include <variant>

#ifdef _MSC_VER
#pragma warning(push, 0)
#endif
#include <Hlms/Pbs/OgreHlmsPbsDatablock.h>
#include <Hlms/Unlit/OgreHlmsUnlitDatablock.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/math/Helpers.hh>

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2Includes.hh"
#include "gz/rendering/ogre2/Ogre2Material.hh"
#include "gz/rendering/ogre2/Ogre2ParticleEmitter.hh"
#include "gz/rendering/ogre2/Ogre2RenderEngine.hh"
#include "gz/rendering/ogre2/Ogre2RenderTarget.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2Sensor.hh"
#include "gz/rendering/ogre2/Ogre2ThermalCamera.hh"
#include "gz/rendering/ogre2/Ogre2Visual.hh"

#include <gz/common/Image.hh>

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
/// \brief Helper class for switching the ogre item's material to heat source
/// material when a thermal camera is being rendered.
class Ogre2ThermalCameraMaterialSwitcher : public Ogre::Camera::Listener
{
  /// \brief constructor
  /// \param[in] _scene the scene manager responsible for rendering
  /// \param[in] _name the name of the thermal camera
  public: explicit Ogre2ThermalCameraMaterialSwitcher(Ogre2ScenePtr _scene,
              const std::string & _name);

  /// \brief destructor
  public: ~Ogre2ThermalCameraMaterialSwitcher() = default;

  /// \brief Set image format
  /// \param[in] _format Image format
  public: void SetFormat(PixelFormat _format);

  /// \brief Set temperature linear resolution
  /// \param[in] _resolution Temperature linear resolution
  public: void SetLinearResolution(double _resolution);

  /// \brief Callback when a camara is about to be rendered
  /// \param[in] _cam Ogre camera pointer which is about to render
  private: virtual void cameraPreRenderScene(
    Ogre::Camera * _cam) override;

  /// \brief Callback when a camera is finisned being rendered
  /// \param[in] _cam Ogre camera pointer which has already render
  private: virtual void cameraPostRenderScene(
    Ogre::Camera * _cam) override;

  /// \brief Scene manager
  private: Ogre2ScenePtr scene = nullptr;

  /// \brief Pointer to the heat source material
  private: Ogre::MaterialPtr heatSourceMaterial;

  /// \brief Pointer to the "base" heat signature material.
  /// All renderable items with a heat signature texture use their own
  /// copy of this base material, with the item's specific heat
  /// signature texture applied to it
  private: Ogre::MaterialPtr baseHeatSigMaterial;

  /// \brief A map of all items that have a heat signature material.
  /// The key is the item's ID, and the value is the heat signature
  /// for that item.
  private: std::unordered_map<Ogre::IdType, Ogre::MaterialPtr>
            heatSignatureMaterials;

  /// \brief The name of the thermal camera sensor
  private: const std::string name;

  /// \brief The thermal camera
  private: const Ogre::Camera* ogreCamera{nullptr};

  /// \brief Custom parameter index of temperature data in an ogre subitem.
  /// This has to match the custom index specifed in ThermalHeatSource material
  /// script in media/materials/scripts/thermal_camera.material
  private: const unsigned int customParamIdx = 10u;

  /// \brief A map of ogre sub item pointer to their original hlms material
  private: std::unordered_map<Ogre::SubItem *, Ogre::HlmsDatablock *>
      datablockMap;

  /// \brief A map of ogre sub item pointer to their original low level material
  private: std::map<Ogre::SubItem *, Ogre::MaterialPtr> thermalMaterialMap;

  /// \brief linear temperature resolution. Defaults to 10mK
  private: double resolution = 0.01;

  /// \brief thermal camera image format
  private: PixelFormat format = PF_L16;

  /// \brief thermal camera image bit depth
  private: unsigned int bitDepth = 16u;
};
}
}
}

/// \internal
/// \brief Private data for the Ogre2ThermalCamera class
class gz::rendering::Ogre2ThermalCameraPrivate
{
  /// \brief Outgoing thermal data, used by newThermalFrame event.
  public: uint16_t *thermalImage = nullptr;

  /// \brief maximum value used for data outside sensor range
  public: uint16_t dataMaxVal = std::numeric_limits<uint16_t>::max();

  /// \brief minimum value used for data outside sensor range
  public: uint16_t dataMinVal = 0u;

  /// \brief 1st pass compositor workspace definition
  public: std::string ogreCompositorWorkspaceDef;

  /// \brief 1st pass compositor node definition
  public: std::string ogreCompositorNodeDef;

  /// \brief 1st pass compositor workspace. One for each cubemap camera
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace;

  /// \brief Thermal textures.
  public: Ogre::TextureGpu *ogreThermalTexture{nullptr};

  /// \brief Dummy render texture for the thermal data
  public: RenderTexturePtr thermalTexture = nullptr;

  /// \brief The thermal material
  public: Ogre::MaterialPtr thermalMaterial;

  /// \brief Event used to signal thermal image data
  public: gz::common::EventT<void(const uint16_t *,
              unsigned int, unsigned int, unsigned int,
              const std::string &)> newThermalFrame;

  /// \brief Pointer to material switcher
  public: std::unique_ptr<Ogre2ThermalCameraMaterialSwitcher>
      thermalMaterialSwitcher = nullptr;

  /// \brief Add variation to temperature values based on object rgb values
  /// This only affects objects that are not heat sources
  /// TODO(anyone) add API for setting this value?
  public: bool rgbToTemp = true;

  /// \brief bit depth of each pixel
  public: unsigned int bitDepth = 16u;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2ThermalCameraMaterialSwitcher::Ogre2ThermalCameraMaterialSwitcher(
    Ogre2ScenePtr _scene, const std::string & _name) : name(_name)
{
  this->scene = _scene;
  // plain opaque material
  Ogre::ResourcePtr res =
    Ogre::MaterialManager::getSingleton().load("ThermalHeatSource",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  this->heatSourceMaterial = res.staticCast<Ogre::Material>();
  this->heatSourceMaterial->load();

  this->baseHeatSigMaterial = Ogre::MaterialManager::getSingleton().
    getByName("ThermalHeatSignature");

  this->ogreCamera = this->scene->OgreSceneManager()->findCamera(this->name);
}

//////////////////////////////////////////////////
void Ogre2ThermalCameraMaterialSwitcher::SetFormat(PixelFormat _format)
{
  this->format = _format;
  this->bitDepth = 8u * PixelUtil::BytesPerChannel(format);
}

//////////////////////////////////////////////////
void Ogre2ThermalCameraMaterialSwitcher::SetLinearResolution(double _resolution)
{
  this->resolution = _resolution;
}
//////////////////////////////////////////////////
void Ogre2ThermalCameraMaterialSwitcher::cameraPreRenderScene(
    Ogre::Camera * /*_cam*/)
{
  // swap item to use v1 shader material
  // Note: keep an eye out for performance impact on switching materials
  // on the fly. We are not doing this often so should be ok.
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);

    const std::string tempKey = "temperature";
    // get visual
    Ogre::Any userAny = item->getUserObjectBindings().getUserAny();
    if (!userAny.isEmpty() && userAny.getType() == typeid(unsigned int))
    {
      VisualPtr result;
      try
      {
        result = this->scene->VisualById(Ogre::any_cast<unsigned int>(userAny));
      }
      catch(Ogre::Exception &e)
      {
        ignerr << "Ogre Error:" << e.getFullDescription() << "\n";
      }
      Ogre2VisualPtr ogreVisual =
          std::dynamic_pointer_cast<Ogre2Visual>(result);

      // get temperature
      Variant tempAny = ogreVisual->UserData(tempKey);
      if (tempAny.index() != 0 && !std::holds_alternative<std::string>(tempAny))
      {
        float temp = -1.0;
        bool foundTemp = true;
        try
        {
          temp = std::get<float>(tempAny);
        }
        catch(...)
        {
          try
          {
            temp = static_cast<float>(std::get<double>(tempAny));
          }
          catch(...)
          {
            try
            {
              temp = static_cast<float>(std::get<int>(tempAny));
            }
            catch(std::bad_variant_access &e)
            {
              ignerr << "Error casting user data: " << e.what() << "\n";
              temp = -1.0;
              foundTemp = false;
            }
          }
        }

        // if a non-positive temperature was given, clamp it to 0
        if (foundTemp && temp < 0.0)
        {
          temp = 0.0;
          ignwarn << "Unable to set negatve temperature for: "
              << ogreVisual->Name() << ". Value cannot be lower than absolute "
              << "zero. Clamping temperature to 0 degrees Kelvin."
              << std::endl;
        }
        for (unsigned int i = 0; i < item->getNumSubItems(); ++i)
        {
          Ogre::SubItem *subItem = item->getSubItem(i);

          // normalize temperature value
          float color = (temp / this->resolution) / ((1 << bitDepth) - 1.0);

          // set g, b, a to 0. This will be used by shaders to determine
          // if particular fragment is a heat source or not
          // see media/materials/programs/thermal_camera_fs.glsl
          subItem->setCustomParameter(this->customParamIdx,
              Ogre::Vector4(color, 0, 0, 0.0));
          // case when item is using low level materials
          // e.g. shaders
          if (!subItem->getMaterial().isNull())
          {
            this->thermalMaterialMap[subItem] = subItem->getMaterial();
          }
          // regular Pbs Hlms datablock
          else
          {
            Ogre::HlmsDatablock *datablock = subItem->getDatablock();
            this->datablockMap[subItem] = datablock;
          }
          subItem->setMaterial(this->heatSourceMaterial);
        }
      }
      // get heat signature and the corresponding min/max temperature values
      else if (auto heatSignature = std::get_if<std::string>(&tempAny))
      {
        // if this is the first time rendering the heat signature,
        // we need to make sure that the texture is loaded and applied to
        // the heat signature material before loading the material
        if (this->heatSignatureMaterials.find(item->getId()) ==
            this->heatSignatureMaterials.end())
        {
          // make sure the texture is in ogre's resource path
          const auto &texture = *heatSignature;
          auto engine = Ogre2RenderEngine::Instance();
          engine->AddResourcePath(texture);

          // create a material for this item, now that the texture has been
          // searched for. We must clone the base heat signature material since
          // different items may use different textures. We also append the
          // item's ID to the end of the new material name to ensure new
          // material uniqueness in case two items use the same heat signature
          // texture, but have different temperature ranges
          std::string baseName = common::basename(texture);
          auto heatSignatureMaterial = this->baseHeatSigMaterial->clone(
              this->name + "_" + baseName + "_" +
              Ogre::StringConverter::toString(item->getId()));
          auto textureUnitStatePtr = heatSignatureMaterial->
            getTechnique(0)->getPass(0)->getTextureUnitState(0);
          Ogre::String textureName = baseName;
          textureUnitStatePtr->setTextureName(textureName);

          // set temperature range for the heat signature
          auto minTempVariant = ogreVisual->UserData("minTemp");
          auto maxTempVariant = ogreVisual->UserData("maxTemp");
          auto minTemperature = std::get_if<float>(&minTempVariant);
          auto maxTemperature = std::get_if<float>(&maxTempVariant);
          if (minTemperature && maxTemperature)
          {
            // make sure the temperature range is between [min, max] kelvin
            // for the given pixel format and camera resolution
            float maxTemp = ((1 << bitDepth) - 1.0) * this->resolution;
            Ogre::GpuProgramParametersSharedPtr params =
              heatSignatureMaterial->getTechnique(0)->getPass(0)->
              getFragmentProgramParameters();
            params->setNamedConstant("minTemp",
                std::max(static_cast<float>(*minTemperature), 0.0f));
            params->setNamedConstant("maxTemp",
                std::min(static_cast<float>(*maxTemperature), maxTemp));
            params->setNamedConstant("bitDepth",
                static_cast<int>(this->bitDepth));
            params->setNamedConstant("resolution",
                static_cast<float>(this->resolution));
          }
          heatSignatureMaterial->load();
          this->heatSignatureMaterials[item->getId()] = heatSignatureMaterial;
        }

        for (unsigned int i = 0; i < item->getNumSubItems(); ++i)
        {
          Ogre::SubItem *subItem = item->getSubItem(i);

          // case when item is using low level materials
          // e.g. shaders
          if (!subItem->getMaterial().isNull())
          {
            this->thermalMaterialMap[subItem] = subItem->getMaterial();
          }
          // regular Pbs Hlms datablock
          else
          {
            Ogre::HlmsDatablock *datablock = subItem->getDatablock();
            this->datablockMap[subItem] = datablock;
          }

          subItem->setMaterial(this->heatSignatureMaterials[item->getId()]);
        }
      }
      // background objects
      else
      {
        Ogre::Aabb aabb = item->getWorldAabbUpdated();
        Ogre::AxisAlignedBox box = Ogre::AxisAlignedBox(aabb.getMinimum(),
            aabb.getMaximum());

        // we will be converting rgb values to temperature values in shaders
        // but we want to make sure the object rgb values are not affected by
        // lighting, so disable lighting
        // Also check if objects are within camera view
        if (ogreVisual->GeometryCount() > 0u &&
            this->ogreCamera->isVisible(box))
        {
          auto geom = ogreVisual->GeometryByIndex(0);
          if (geom)
          {
            MaterialPtr mat = geom->Material();
            Ogre2MaterialPtr ogreMat =
                std::dynamic_pointer_cast<Ogre2Material>(mat);
            Ogre::HlmsUnlitDatablock *unlit = ogreMat->UnlitDatablock();
            for (unsigned int i = 0; i < item->getNumSubItems(); ++i)
            {
              Ogre::SubItem *subItem = item->getSubItem(i);
              if (!subItem->getMaterial().isNull())
              {
                this->thermalMaterialMap[subItem] = subItem->getMaterial();
              }
              // regular Pbs Hlms datablock
              else
              {
                Ogre::HlmsDatablock *datablock = subItem->getDatablock();
                this->datablockMap[subItem] = datablock;
              }
              subItem->setDatablock(unlit);
            }
          }
        }
      }
    }
    itor.moveNext();
  }
}

//////////////////////////////////////////////////
void Ogre2ThermalCameraMaterialSwitcher::cameraPostRenderScene(
    Ogre::Camera * /*_cam*/)
{
  // restore item to use pbs hlms material
  for (auto it : this->datablockMap)
  {
    Ogre::SubItem *subItem = it.first;
    subItem->setDatablock(it.second);
  }

  // restore item to use low level material
  for (auto it : this->thermalMaterialMap)
  {
    Ogre::SubItem *subItem = it.first;
    subItem->setMaterial(it.second);
  }

  this->datablockMap.clear();
  this->thermalMaterialMap.clear();
}

//////////////////////////////////////////////////
Ogre2ThermalCamera::Ogre2ThermalCamera()
  : dataPtr(new Ogre2ThermalCameraPrivate())
{
  this->dataPtr->ogreCompositorWorkspace = nullptr;
}

//////////////////////////////////////////////////
Ogre2ThermalCamera::~Ogre2ThermalCamera()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void Ogre2ThermalCamera::Init()
{
  BaseThermalCamera::Init();

  // create internal camera
  this->CreateCamera();

  // create dummy render texture
  this->CreateRenderTexture();

  this->Reset();
}

//////////////////////////////////////////////////
void Ogre2ThermalCamera::Destroy()
{
  if (this->dataPtr->thermalImage)
  {
    delete [] this->dataPtr->thermalImage;
    this->dataPtr->thermalImage = nullptr;
  }

  if (!this->ogreCamera)
    return;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  auto ogreCompMgr = ogreRoot->getCompositorManager2();

  // remove thermal texture, material, compositor
  if (this->dataPtr->ogreThermalTexture)
  {
    ogreRoot->getRenderSystem()->getTextureGpuManager()->destroyTexture(
      this->dataPtr->ogreThermalTexture);
    this->dataPtr->ogreThermalTexture = nullptr;
  }
  if (this->dataPtr->ogreCompositorWorkspace)
  {
    ogreCompMgr->removeWorkspace(
        this->dataPtr->ogreCompositorWorkspace);
  }

  if (this->dataPtr->thermalMaterial)
  {
    Ogre::MaterialManager::getSingleton().remove(
        this->dataPtr->thermalMaterial->getName());
    this->dataPtr->thermalMaterial.setNull();
  }

  if (!this->dataPtr->ogreCompositorWorkspaceDef.empty())
  {
    ogreCompMgr->removeWorkspaceDefinition(
        this->dataPtr->ogreCompositorWorkspaceDef);
    ogreCompMgr->removeNodeDefinition(
        this->dataPtr->ogreCompositorNodeDef);
  }

  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
  }
  else
  {
    if (ogreSceneManager->findCameraNoThrow(this->name) != nullptr)
    {
      ogreSceneManager->destroyCamera(this->ogreCamera);
      this->ogreCamera = nullptr;
    }
  }

  this->dataPtr->thermalMaterialSwitcher.reset();
}

//////////////////////////////////////////////////
void Ogre2ThermalCamera::CreateCamera()
{
  // create ogre camera object
  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->ogreCamera = ogreSceneManager->createCamera(this->name);
  if (this->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  // by default, ogre2 cameras are attached to root scene node
  this->ogreCamera->detachFromParent();
  this->ogreNode->attachObject(this->ogreCamera);

  // rotate to Gazebo coordinate system
  this->ogreCamera->yaw(Ogre::Degree(-90.0));
  this->ogreCamera->roll(Ogre::Degree(-90.0));
  this->ogreCamera->setFixedYawAxis(false);

  // TODO(anyone): provide api access
  this->ogreCamera->setAutoAspectRatio(true);
  this->ogreCamera->setProjectionType(Ogre::PT_PERSPECTIVE);
  this->ogreCamera->setCustomProjectionMatrix(false);
}

/////////////////////////////////////////////////
void Ogre2ThermalCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->thermalTexture =
      std::dynamic_pointer_cast<Ogre2RenderTexture>(base);
  this->dataPtr->thermalTexture->SetWidth(1);
  this->dataPtr->thermalTexture->SetHeight(1);
}

/////////////////////////////////////////////////////////
void Ogre2ThermalCamera::CreateThermalTexture()
{
  // set aspect ratio and fov
  double vfov;
  vfov = 2.0 * atan(tan(this->HFOV().Radian() / 2.0) / this->AspectRatio());
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));

  // Load thermal material
  // The ThermalCamera material is defined in script (thermal_camera.material).
  // We need to clone it since we are going to modify its uniform variables
  std::string matThermalName = "ThermalCamera";
  Ogre::MaterialPtr matThermal =
      Ogre::MaterialManager::getSingleton().getByName(matThermalName);
  this->dataPtr->thermalMaterial = matThermal->clone(
      this->Name() + "_" + matThermalName);
  this->dataPtr->thermalMaterial->load();
  Ogre::Pass *pass =
      this->dataPtr->thermalMaterial->getTechnique(0)->getPass(0);
  Ogre::GpuProgramParametersSharedPtr psParams =
      pass->getFragmentProgramParameters();

  // Configure camera behaviour.
  double nearPlane = this->NearClipPlane();
  double farPlane = this->FarClipPlane();
  this->ogreCamera->setNearClipDistance(nearPlane);
  this->ogreCamera->setFarClipDistance(farPlane);

  // only support 8 bit and 16 bit formats for now.
  // default to 16 bit
  Ogre::PixelFormatGpu ogrePF;
  if (this->ImageFormat() == PF_L8)
  {
    ogrePF = Ogre::PFG_R8_UNORM;
  }
  else
  {
    this->SetImageFormat(PF_L16);
    ogrePF = Ogre::PFG_R16_UNORM;
  }

  PixelFormat format = this->ImageFormat();
  this->dataPtr->bitDepth = 8u * PixelUtil::BytesPerChannel(format);

  // Set the uniform variables (thermal_camera_fs.glsl).
  // The projectParams is used to linearize thermal buffer data
  // The other params are used to clamp the range output
  // Use the 'real' clip distance here so thermal can be
  // linearized correctly
  Ogre::Vector2 projectionAB =
    this->ogreCamera->getProjectionParamsAB();
  double projectionA = projectionAB.x;
  double projectionB = projectionAB.y;
  projectionB /= farPlane;
  psParams->setNamedConstant("projectionParams",
      Ogre::Vector2(projectionA, projectionB));
  psParams->setNamedConstant("near",
      static_cast<float>(this->NearClipPlane()));
  psParams->setNamedConstant("far",
      static_cast<float>(this->FarClipPlane()));
  psParams->setNamedConstant("max",
      static_cast<float>(this->maxTemp));
  psParams->setNamedConstant("min",
      static_cast<float>(this->minTemp));
  psParams->setNamedConstant("resolution",
      static_cast<float>(this->resolution));
  psParams->setNamedConstant("ambient",
      static_cast<float>(this->ambient));
  psParams->setNamedConstant("range",
      static_cast<float>(this->ambientRange));
  psParams->setNamedConstant("heatSourceTempRange",
      static_cast<float>(this->heatSourceTempRange));
  psParams->setNamedConstant("rgbToTemp",
      static_cast<int>(this->dataPtr->rgbToTemp));
  psParams->setNamedConstant("bitDepth",
      static_cast<int>(this->dataPtr->bitDepth));

  // Create thermal camera compositor
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  // We need to programmatically create the compositor because we need to
  // configure it to use the cloned thermal material created earlier.
  // The compositor workspace definition is equivalent to the following
  // ogre compositor script:
  // compositor_node ThermalCamera
  // {
  //   in 0 rt_input
  //   texture depthTexture target_width target_height PFG_D32_FLOAT
  //   texture colorTexture target_width target_height PFG_RGBA8_UNORM
  //
  //   rtv colorTexture
  //   {
  //     depth depthTexture
  //   }
  //
  //   target colorTexture
  //   {
  //     pass clear
  //     {
  //       colour_value 0.0 0.0 0.0 1.0
  //     }
  //     pass render_scene
  //     {
  //     }
  //   }
  //   target rt_input
  //   {
  //     pass clear
  //     {
  //       colour_value 0.0 0.0 0.0 1.0
  //     }
  //     pass render_quad
  //     {
  //       material ThermalCamera // Use copy instead of original
  //       input 0 depthTexture
  //       input 1 colorTexture
  //       quad_normals camera_far_corners_view_space
  //     }
  //   }
  //   out 0 rt_input
  // }
  std::string wsDefName = "ThermalCameraWorkspace_" + this->Name();
  this->dataPtr->ogreCompositorWorkspaceDef = wsDefName;
  if (!ogreCompMgr->hasWorkspaceDefinition(wsDefName))
  {
    std::string nodeDefName = wsDefName + "/Node";
    this->dataPtr->ogreCompositorNodeDef = nodeDefName;
    Ogre::CompositorNodeDef *nodeDef =
        ogreCompMgr->addNodeDefinition(nodeDefName);
    // Input texture
    nodeDef->addTextureSourceName("rt_input", 0,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    Ogre::TextureDefinitionBase::TextureDefinition *thermalTexDef =
        nodeDef->addTextureDefinition("depthTexture");
    thermalTexDef->textureType = Ogre::TextureTypes::Type2D;
    thermalTexDef->width = 0;
    thermalTexDef->height = 0;
    thermalTexDef->depthOrSlices = 1;
    thermalTexDef->numMipmaps = 0;
    thermalTexDef->widthFactor = 1;
    thermalTexDef->heightFactor = 1;
    thermalTexDef->format = Ogre::PFG_D32_FLOAT;
    thermalTexDef->textureFlags &= ~Ogre::TextureFlags::Uav;
    // set to default pool so that when the colorTexture pass is rendered, its
    // depth data get populated to depthTexture
    thermalTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    thermalTexDef->depthBufferFormat = Ogre::PFG_UNKNOWN;

    Ogre::TextureDefinitionBase::TextureDefinition *colorTexDef =
        nodeDef->addTextureDefinition("colorTexture");
    colorTexDef->textureType = Ogre::TextureTypes::Type2D;
    colorTexDef->width = 0;
    colorTexDef->height = 0;
    colorTexDef->depthOrSlices = 1;
    colorTexDef->numMipmaps = 0;
    colorTexDef->widthFactor = 1;
    colorTexDef->heightFactor = 1;
    colorTexDef->format = Ogre::PFG_RGBA8_UNORM;
    colorTexDef->textureFlags &= ~Ogre::TextureFlags::Uav;
    colorTexDef->depthBufferId = Ogre::DepthBuffer::POOL_DEFAULT;
    colorTexDef->depthBufferFormat = Ogre::PFG_D32_FLOAT;
    colorTexDef->preferDepthTexture = true;

    Ogre::RenderTargetViewDef *rtv =
      nodeDef->addRenderTextureView("colorTexture");
    rtv->setForTextureDefinition("colorTexture", colorTexDef);
    rtv->depthAttachment.textureName = "depthTexture";

    nodeDef->setNumTargetPass(2);
    Ogre::CompositorTargetDef *colorTargetDef =
        nodeDef->addTargetPass("colorTexture");
    colorTargetDef->setNumPasses(1);
    {
      // scene pass
      Ogre::CompositorPassSceneDef *passScene =
          static_cast<Ogre::CompositorPassSceneDef *>(
          colorTargetDef->addPass(Ogre::PASS_SCENE));
      passScene->setAllLoadActions(Ogre::LoadAction::Clear);
      passScene->setAllClearColours(Ogre::ColourValue(0, 0, 0));
      // thermal camera should not see particles
      passScene->setVisibilityMask(
        IGN_VISIBILITY_ALL & ~Ogre2ParticleEmitter::kParticleVisibilityFlags);
    }

    // rt_input target - converts to thermal
    Ogre::CompositorTargetDef *inputTargetDef =
        nodeDef->addTargetPass("rt_input");
    inputTargetDef->setNumPasses(1);
    {
      // quad pass
      Ogre::CompositorPassQuadDef *passQuad =
          static_cast<Ogre::CompositorPassQuadDef *>(
          inputTargetDef->addPass(Ogre::PASS_QUAD));
      passQuad->setAllLoadActions(Ogre::LoadAction::Clear);
      passQuad->setAllClearColours(Ogre::ColourValue(this->ambient, 0, 1.0));

      passQuad->mMaterialName = this->dataPtr->thermalMaterial->getName();
      passQuad->addQuadTextureSource(0, "depthTexture");
      passQuad->addQuadTextureSource(1, "colorTexture");
      passQuad->mFrustumCorners =
          Ogre::CompositorPassQuadDef::VIEW_SPACE_CORNERS;
    }
    nodeDef->mapOutputChannel(0, "rt_input");
    Ogre::CompositorWorkspaceDef *workDef =
        ogreCompMgr->addWorkspaceDefinition(wsDefName);
    workDef->connectExternal(0, nodeDef->getName(), 0);
  }
  Ogre::CompositorWorkspaceDef *wsDef =
      ogreCompMgr->getWorkspaceDefinition(wsDefName);

  if (!wsDef)
  {
    ignerr << "Unable to add workspace definition [" << wsDefName << "] "
           << " for " << this->Name();
  }

  Ogre::TextureGpuManager *textureMgr =
    ogreRoot->getRenderSystem()->getTextureGpuManager();
  // create render texture - these textures pack the thermal data
  this->dataPtr->ogreThermalTexture =
    textureMgr->createOrRetrieveTexture(this->Name() + "_thermal",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::RenderToTexture,
      Ogre::TextureTypes::Type2D);

  this->dataPtr->ogreThermalTexture->setResolution(
    this->ImageWidth(), this->ImageHeight());
  this->dataPtr->ogreThermalTexture->setNumMipmaps(1u);
  this->dataPtr->ogreThermalTexture->setPixelFormat(
    ogrePF);

  this->dataPtr->ogreThermalTexture->scheduleTransitionTo(
    Ogre::GpuResidency::Resident);

  // create compositor worksspace
  this->dataPtr->ogreCompositorWorkspace =
      ogreCompMgr->addWorkspace(
        this->scene->OgreSceneManager(),
        this->dataPtr->ogreThermalTexture,
        this->ogreCamera,
        wsDefName,
        false);

  // add thermal material switcher to render target listener
  // so we can switch to use heat material when the camera is being udpated
  Ogre::CompositorNode *node =
      this->dataPtr->ogreCompositorWorkspace->getNodeSequence()[0];
  auto channels = node->getLocalTextures();
  for (auto c : channels)
  {
    if (c->getPixelFormat() == Ogre::PFG_RGBA8_UNORM)
    {
      this->dataPtr->thermalMaterialSwitcher.reset(
          new Ogre2ThermalCameraMaterialSwitcher(this->scene, this->Name()));
      this->dataPtr->thermalMaterialSwitcher->SetFormat(this->ImageFormat());
      this->dataPtr->thermalMaterialSwitcher->SetLinearResolution(
          this->resolution);
      this->ogreCamera->addListener(
        this->dataPtr->thermalMaterialSwitcher.get());
      break;
    }
  }
}

//////////////////////////////////////////////////
void Ogre2ThermalCamera::Render()
{
  // GL_DEPTH_CLAMP is disabled in later version of ogre2.2
  // however our shaders rely on clamped values so enable it for this sensor
  auto engine = Ogre2RenderEngine::Instance();
  std::string renderSystemName =
      engine->OgreRoot()->getRenderSystem()->getFriendlyName();
  bool useGL = renderSystemName.find("OpenGL") != std::string::npos;
#ifndef _WIN32
  if (useGL)
    glEnable(GL_DEPTH_CLAMP);
#endif

  // update the compositors
  this->scene->StartRendering(this->ogreCamera);

  this->dataPtr->ogreCompositorWorkspace->_validateFinalTarget();
  this->dataPtr->ogreCompositorWorkspace->_beginUpdate(false);
  this->dataPtr->ogreCompositorWorkspace->_update();
  this->dataPtr->ogreCompositorWorkspace->_endUpdate(false);

  Ogre::vector<Ogre::TextureGpu*>::type swappedTargets;
  swappedTargets.reserve(2u);
  this->dataPtr->ogreCompositorWorkspace->_swapFinalTarget(swappedTargets);

  this->scene->FlushGpuCommandsAndStartNewFrame(1u, false);

#ifndef _WIN32
  if (useGL)
    glDisable(GL_DEPTH_CLAMP);
#endif
}

//////////////////////////////////////////////////
void Ogre2ThermalCamera::PreRender()
{
  if (!this->dataPtr->ogreThermalTexture)
    this->CreateThermalTexture();
}

//////////////////////////////////////////////////
void Ogre2ThermalCamera::PostRender()
{
  if (this->dataPtr->newThermalFrame.ConnectionCount() <= 0u)
    return;

  unsigned int width = this->ImageWidth();
  unsigned int height = this->ImageHeight();
  PixelFormat format = this->ImageFormat();

  int len = width * height;
  unsigned int channelCount = PixelUtil::ChannelCount(format);
  unsigned int bytesPerChannel = PixelUtil::BytesPerChannel(format);

  Ogre::Image2 image;
  image.convertFromTexture(this->dataPtr->ogreThermalTexture, 0u, 0u);

  if (!this->dataPtr->thermalImage)
  {
    this->dataPtr->thermalImage = new uint16_t[len];
  }

  Ogre::TextureBox box = image.getData(0u);
  if (format == PF_L8)
  {
    uint8_t *thermalBuffer = static_cast<uint8_t*>(box.data);
    for (unsigned int i = 0u; i < height; ++i)
    {
      // the texture box step size could be larger than our image buffer step
      // size
      unsigned int rawDataRowIdx = i * box.bytesPerRow / bytesPerChannel;
      for (unsigned int j = 0u; j < width; ++j)
      {
        unsigned int idx = (i * width) + j;
        this->dataPtr->thermalImage[idx] = thermalBuffer[rawDataRowIdx + j];
      }
    }
  }
  else
  {
    // fill thermal data
    // copy data row by row. The texture box may not be a contiguous region of
    // a texture
    uint16_t * thermalBuffer = static_cast<uint16_t *>(box.data);
    for (unsigned int i = 0; i < height; ++i)
    {
      unsigned int rawDataRowIdx = i * box.bytesPerRow / bytesPerChannel;
      unsigned int rowIdx = i * width * channelCount;
      memcpy(&this->dataPtr->thermalImage[rowIdx],
          &thermalBuffer[rawDataRowIdx],
          width * channelCount * bytesPerChannel);
    }
  }

  this->dataPtr->newThermalFrame(
      this->dataPtr->thermalImage, width, height, 1,
      PixelUtil::Name(format));

  // Uncomment to debug thermal output
  // std::cout << "wxh: " << width << " x " << height << std::endl;
  // for (unsigned int i = 0; i < height; ++i)
  // {
  //   for (unsigned int j = 0; j < width; ++j)
  //   {
  //     std::cout << "[" << this->dataPtr->thermalImage[i*width + j] << "]";
  //   }
  //   std::cout << std::endl;
  // }
}

//////////////////////////////////////////////////
common::ConnectionPtr Ogre2ThermalCamera::ConnectNewThermalFrame(
    std::function<void(const uint16_t *, unsigned int, unsigned int,
      unsigned int, const std::string &)>  _subscriber)
{
  return this->dataPtr->newThermalFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
RenderTargetPtr Ogre2ThermalCamera::RenderTarget() const
{
  return this->dataPtr->thermalTexture;
}
