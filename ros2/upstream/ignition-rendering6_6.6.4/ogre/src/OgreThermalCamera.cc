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

#include <limits>

#include <gz/math/Helpers.hh>
#include "gz/rendering/ShaderParams.hh"
#include "gz/rendering/ogre/OgreThermalCamera.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreVisual.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
/// \brief Helper class for switching the ogre item's material to heat source
/// material when a thermal camera is being rendered.
class OgreThermalCameraMaterialSwitcher : public Ogre::RenderTargetListener,
      public Ogre::MaterialManager::Listener
{
  /// \brief constructor
  /// \param[in] _scene the scene manager responsible for rendering
  /// \param[in] _near Camera near plane
  /// \param[in] _far Camera far plane
  public: OgreThermalCameraMaterialSwitcher(OgreScenePtr _scene,
      double _near, double _far);

  /// \brief destructor
  public: ~OgreThermalCameraMaterialSwitcher() = default;

  /// \brief Callback when a render target is about to be rendered
  /// \param[in] _evt Ogre render target event containing information about
  /// the source render target.
  private: virtual void preRenderTargetUpdate(
      const Ogre::RenderTargetEvent &_evt) override;

  /// \brief Callback when a render target is finisned being rendered
  /// \param[in] _evt Ogre render target event containing information about
  /// the source render target.
  private: virtual void postRenderTargetUpdate(
      const Ogre::RenderTargetEvent &_evt) override;

  // Documentation inherited.
  private: Ogre::Technique *handleSchemeNotFound(
    uint16_t _schemeIndex, const Ogre::String &_schemeName,
    Ogre::Material *_originalMaterial, uint16_t _lodIndex,
    const Ogre::Renderable *_rend) override;

  /// \brief Scene manager
  private: OgreScenePtr scene;

  /// \brief Pointer to the heat source material
  private: Ogre::MaterialPtr heatSourceMaterial;

  /// \brief Material scheme name
  private: std::string schemeName = "thermal";

  /// \brief Custom parameter index of temperature data in an ogre subitem.
  /// This has to match the custom index specifed in ThermalHeatSource material
  /// script in media/materials/scripts/thermal_camera.material
  private: const unsigned int customParamIdx = 10u;
};
}
}
}

/// \internal
/// \brief Private data for the OgreThermalCamera class
class gz::rendering::OgreThermalCameraPrivate
{
  /// \brief The thermal material
  public: Ogre::MaterialPtr thermalMaterial;

  /// \brief thermal camera texture
  public: Ogre::Texture *ogreThermalTexture = nullptr;

  /// \brief heat source texture
  public: Ogre::Texture *ogreHeatSourceTexture = nullptr;

  /// \brief Dummy texture
  public: OgreRenderTexturePtr thermalTexture;

  /// \brief Lens distortion compositor
  public: Ogre::CompositorInstance *thermalInstance = nullptr;

  /// \brief The thermal buffer
  public: uint16_t *thermalBuffer = nullptr;

  /// \brief Outgoing thermal data, used by newThermalFrame event.
  public: uint16_t *thermalImage = nullptr;

  /// \brief maximum value used for data outside sensor range
  public: uint16_t dataMaxVal = std::numeric_limits<uint16_t>::max();

  /// \brief minimum value used for data outside sensor range
  public: uint16_t dataMinVal = 0u;

  /// \brief Event used to signal thermal image data
  public: gz::common::EventT<void(const uint16_t *,
              unsigned int, unsigned int, unsigned int,
              const std::string &)> newThermalFrame;

  /// \brief Pointer to material switcher
  public: std::unique_ptr<OgreThermalCameraMaterialSwitcher>
      thermalMaterialSwitcher;
};

using namespace gz;
using namespace rendering;


//////////////////////////////////////////////////
OgreThermalCameraMaterialSwitcher::OgreThermalCameraMaterialSwitcher(
    OgreScenePtr _scene, double _near, double _far)
{
  this->scene = _scene;

  // heat source material
  std::string matName = "ThermalHeatSource";
  Ogre::ResourcePtr res =
    Ogre::MaterialManager::getSingleton().load(matName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  // OGRE 1.9 changes the shared pointer definition
  #if OGRE_VERSION_LT_1_11_0
  this->heatSourceMaterial = res.staticCast<Ogre::Material>();
  #else
  this->heatSourceMaterial =
    std::static_pointer_cast<Ogre::Material>(res);
  #endif
  this->heatSourceMaterial->load();
  static int thermalMatNameCount = 0;
  this->heatSourceMaterial = this->heatSourceMaterial->clone(
      matName + "_" + std::to_string(thermalMatNameCount++));

  Ogre::Pass *pass = this->heatSourceMaterial->getTechnique(0)->getPass(0);
  auto params = pass->getFragmentProgramParameters();
  params->setNamedConstant("near", static_cast<float>(_near));
  params->setNamedConstant("far", static_cast<float>(_far));
}

//////////////////////////////////////////////////
void OgreThermalCameraMaterialSwitcher::preRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  Ogre::MaterialManager::getSingleton().addListener(this);
}

//////////////////////////////////////////////////
void OgreThermalCameraMaterialSwitcher::postRenderTargetUpdate(
    const Ogre::RenderTargetEvent & /*_evt*/)
{
  Ogre::MaterialManager::getSingleton().removeListener(this);
}

//////////////////////////////////////////////////
/// \brief Ogre callback that assigns material to new renderables
Ogre::Technique *OgreThermalCameraMaterialSwitcher::handleSchemeNotFound(
    uint16_t /*_schemeIndex*/, const Ogre::String &_schemeName,
    Ogre::Material * /*_originalMaterial*/, uint16_t /*_lodIndex*/,
    const Ogre::Renderable *_rend)
{
  if (_schemeName != this->schemeName)
    return nullptr;

  if (!_rend || typeid(*_rend) != typeid(Ogre::SubEntity))
    return nullptr;

  const Ogre::SubEntity *subEntity =
    static_cast<const Ogre::SubEntity *>(_rend);

  OgreVisualPtr ogreVisual;
  Ogre::Any userAny =
      subEntity->getParent()->getUserObjectBindings().getUserAny();
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
    ogreVisual = std::dynamic_pointer_cast<OgreVisual>(result);
  }

  if (!ogreVisual)
    return nullptr;

  // get temperature
  std::string tempKey = "temperature";
  Variant tempAny = ogreVisual->UserData(tempKey);
  if (tempAny.index() != 0)
  {
    float temp = -1;
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
        }
      }
    }

    // only accept positive temperature (in kelvin)
    if (temp >= 0.0)
    {
      // normalize temperature value
      float color = temp * 100.0 / static_cast<float>(
          std::numeric_limits<uint16_t>::max());
      const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(
          this->customParamIdx,
          Ogre::Vector4(color, 0.0, 0.0, 1.0));

      return this->heatSourceMaterial->getSupportedTechnique(0);
    }
  }

  const_cast<Ogre::SubEntity *>(subEntity)->setCustomParameter(
      this->customParamIdx,
      Ogre::Vector4(0.0, 0.0, 0.0, 1.0));

  return this->heatSourceMaterial->getSupportedTechnique(0);
}

//////////////////////////////////////////////////
OgreThermalCamera::OgreThermalCamera()
  : dataPtr(new OgreThermalCameraPrivate())
{
}

//////////////////////////////////////////////////
OgreThermalCamera::~OgreThermalCamera()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void OgreThermalCamera::Destroy()
{
  if (this->dataPtr->thermalBuffer)
  {
    delete [] this->dataPtr->thermalBuffer;
    this->dataPtr->thermalBuffer = nullptr;
  }

  if (this->dataPtr->thermalImage)
  {
    delete [] this->dataPtr->thermalImage;
    this->dataPtr->thermalImage = nullptr;
  }

  if (!this->ogreCamera || !this->scene->IsInitialized())
    return;

  if (this->dataPtr->thermalInstance)
  {
    // Do not leave a reference to this->dataPtr->thermalMaterial
    Ogre::MaterialPtr nullMaterial;
    this->dataPtr->thermalInstance->getTechnique()
      ->getOutputTargetPass()
      ->getPass(0)
      ->setMaterial(nullMaterial);
  }

  // remove thermal textures
  if (this->dataPtr->ogreThermalTexture)
  {
    Ogre::TextureManager::getSingleton().remove(
        this->dataPtr->ogreThermalTexture->getName());
    this->dataPtr->ogreThermalTexture = nullptr;
  }

  if (this->dataPtr->ogreHeatSourceTexture)
  {
    Ogre::TextureManager::getSingleton().remove(
        this->dataPtr->ogreHeatSourceTexture->getName());
    this->dataPtr->ogreHeatSourceTexture = nullptr;
  }

  if (!this->dataPtr->thermalMaterial.isNull())
  {
    Ogre::MaterialManager::getSingleton().remove(
      this->dataPtr->thermalMaterial->getHandle());
    this->dataPtr->thermalMaterial.setNull();
  }

  this->dataPtr->thermalMaterialSwitcher.reset();

  this->DestroyRenderTexture();

  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
  }
  else
  {
    if (ogreSceneManager->hasCamera(this->name))
    {
      ogreSceneManager->destroyCamera(this->name);
      this->ogreCamera = nullptr;
    }
  }
}

//////////////////////////////////////////////////
void OgreThermalCamera::Init()
{
  BaseThermalCamera::Init();
  this->CreateCamera();
  this->CreateRenderTexture();
  this->Reset();
}

//////////////////////////////////////////////////
void OgreThermalCamera::CreateCamera()
{
  // create ogre camera object
  Ogre::SceneManager *ogreSceneManager;
  ogreSceneManager = this->scene->OgreSceneManager();
  if (ogreSceneManager == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->ogreCamera = ogreSceneManager->createCamera(
      this->name);
  if (this->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  this->ogreNode->attachObject(this->ogreCamera);

  // rotate to Gazebo coordinate system
  this->ogreCamera->yaw(Ogre::Degree(-90.0));
  this->ogreCamera->roll(Ogre::Degree(-90.0));
  this->ogreCamera->setFixedYawAxis(false);

  // TODO(anyone): provide api access
  this->ogreCamera->setAutoAspectRatio(true);
  this->ogreCamera->setRenderingDistance(0);
  this->ogreCamera->setPolygonMode(Ogre::PM_SOLID);
  this->ogreCamera->setProjectionType(Ogre::PT_PERSPECTIVE);
  this->ogreCamera->setCustomProjectionMatrix(false);
}

/////////////////////////////////////////////////
void OgreThermalCamera::CreateThermalTexture()
{
  if (this->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  if (!this->dataPtr->ogreThermalTexture)
  {
    this->dataPtr->ogreThermalTexture =
      Ogre::TextureManager::getSingleton().createManual(
      this->Name() + "_thermal", "General", Ogre::TEX_TYPE_2D,
      this->ImageWidth(), this->ImageHeight(), 0,
      Ogre::PF_L16, Ogre::TU_RENDERTARGET,
      0, false, 0).get();
    Ogre::RenderTarget *rt =
        this->dataPtr->ogreThermalTexture->getBuffer()->getRenderTarget();
    rt->setAutoUpdated(false);
    Ogre::Viewport *vp = rt->addViewport(this->ogreCamera);
    vp->setClearEveryFrame(true);
    vp->setShadowsEnabled(false);
    vp->setOverlaysEnabled(false);
  }

  double ratio = static_cast<double>(this->ImageWidth()) /
                 static_cast<double>(this->ImageHeight());

  double vfov = 2.0 * atan(tan(this->HFOV().Radian() / 2.0) / ratio);
  this->ogreCamera->setAspectRatio(ratio);
  this->ogreCamera->setFOVy(Ogre::Radian(vfov));

  // near and far plane are passed to heat source frag shaders through
  // material switcher. They are used to normalize depth values which are then
  // used for adding variations to temperature of the heat source
  // Here we set a small clip distance so camera can see close objects
  // without being clipped.
  double nearPlane = this->NearClipPlane();
  double farPlane = this->FarClipPlane();
  this->ogreCamera->setNearClipDistance(1e-4f);
  this->ogreCamera->setFarClipDistance(farPlane);

  // create thermal material
  std::string matName = "ThermalCamera";
  Ogre::ResourcePtr res =
    Ogre::MaterialManager::getSingleton().load(matName,
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

  // OGRE 1.9 changes the shared pointer definition
  #if OGRE_VERSION_LT_1_11_0
  this->dataPtr->thermalMaterial = res.staticCast<Ogre::Material>();
  #else
  this->dataPtr->thermalMaterial =
    std::static_pointer_cast<Ogre::Material>(res);
  #endif

  this->dataPtr->thermalMaterial = this->dataPtr->thermalMaterial->clone(
      matName + "_" + this->ogreCamera->getName());
  this->dataPtr->thermalMaterial->load();

  Ogre::Pass *pass =
      this->dataPtr->thermalMaterial->getTechnique(0)->getPass(0);
  auto params = pass->getFragmentProgramParameters();
  params->setNamedConstant("max", this->maxTemp);
  params->setNamedConstant("min", this->minTemp);
  params->setNamedConstant("resolution", this->resolution);
  params->setNamedConstant("ambient", this->ambient);
  params->setNamedConstant("range", this->ambientRange);
  params->setNamedConstant("heatSourceTempRange", this->heatSourceTempRange);

  this->dataPtr->thermalInstance =
      Ogre::CompositorManager::getSingleton().addCompositor(
      this->ogreCamera->getViewport(), "ThermalCamera");
  this->dataPtr->thermalInstance->getTechnique()->getOutputTargetPass()->
      getPass(0)->setMaterial(this->dataPtr->thermalMaterial);
  this->dataPtr->thermalInstance->setEnabled(true);

  if (!this->dataPtr->ogreHeatSourceTexture)
  {
    this->dataPtr->ogreHeatSourceTexture =
      Ogre::TextureManager::getSingleton().createManual(
      this->Name() + "_heatSource", "General", Ogre::TEX_TYPE_2D,
      this->ImageWidth(), this->ImageHeight(), 0,
      Ogre::PF_R8G8B8, Ogre::TU_RENDERTARGET,
      0, false, 0).get();

    Ogre::RenderTarget *rt =
        this->dataPtr->ogreHeatSourceTexture->getBuffer()->getRenderTarget();
    Ogre::Viewport *vp = rt->addViewport(this->ogreCamera);
    vp->setClearEveryFrame(true);
    vp->setShadowsEnabled(false);
    vp->setOverlaysEnabled(false);
    rt->setAutoUpdated(false);

    vp->setMaterialScheme("thermal");
    this->dataPtr->thermalMaterialSwitcher.reset(
        new OgreThermalCameraMaterialSwitcher(
        this->scene, nearPlane, farPlane));
    rt->addListener(this->dataPtr->thermalMaterialSwitcher.get());

    Ogre::TextureUnitState *tex = pass->getTextureUnitState(0u);
    tex->setTextureName(this->dataPtr->ogreHeatSourceTexture->getName());
    tex->setTextureFiltering(Ogre::TFO_NONE);
  }
}

/////////////////////////////////////////////////
void OgreThermalCamera::CreateRenderTexture()
{
  this->DestroyRenderTexture();
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->thermalTexture =
      std::dynamic_pointer_cast<OgreRenderTexture>(base);
  this->dataPtr->thermalTexture->SetWidth(1);
  this->dataPtr->thermalTexture->SetHeight(1);
}

//////////////////////////////////////////////////
void OgreThermalCamera::DestroyRenderTexture()
{
  if (this->dataPtr->thermalTexture)
  {
    dynamic_cast<OgreRenderTexture *>(this->dataPtr->thermalTexture.get())
      ->Destroy();
    this->dataPtr->thermalTexture.reset();
  }
}

//////////////////////////////////////////////////
void OgreThermalCamera::PreRender()
{
  BaseCamera::PreRender();
  if (!this->dataPtr->ogreThermalTexture)
    this->CreateThermalTexture();
}

//////////////////////////////////////////////////
void OgreThermalCamera::Render()
{
  // render heat source
  Ogre::RenderTarget *heatRt =
      this->dataPtr->ogreHeatSourceTexture->getBuffer()->getRenderTarget();
  heatRt->update();

  Ogre::RenderTarget *rt =
      this->dataPtr->ogreThermalTexture->getBuffer()->getRenderTarget();
  rt->setAutoUpdated(false);
  rt->update(false);
}

//////////////////////////////////////////////////
void OgreThermalCamera::PostRender()
{
  if (this->dataPtr->newThermalFrame.ConnectionCount() <= 0u)
    return;

  unsigned int width = this->ImageWidth();
  unsigned int height = this->ImageHeight();
  unsigned int len = width * height;

  PixelFormat format = PF_L16;
  unsigned int channelCount = PixelUtil::ChannelCount(format);
  unsigned int bytesPerChannel = PixelUtil::BytesPerChannel(format);

  if (!this->dataPtr->thermalImage)
    this->dataPtr->thermalImage = new uint16_t[len * channelCount];
  if (!this->dataPtr->thermalBuffer)
    this->dataPtr->thermalBuffer = new uint16_t[len * channelCount];

  // get thermal data
  Ogre::RenderTarget *rt =
      this->dataPtr->ogreThermalTexture->getBuffer()->getRenderTarget();
  Ogre::PixelBox ogrePixelBox(width, height, 1,
      OgreConversions::Convert(format), this->dataPtr->thermalBuffer);
  rt->copyContentsToMemory(ogrePixelBox);

  // fill thermal data
  memcpy(this->dataPtr->thermalImage, this->dataPtr->thermalBuffer,
      height*width*channelCount*bytesPerChannel);

  this->dataPtr->newThermalFrame(
      this->dataPtr->thermalBuffer, width, height, 1, "L16");

  // Uncomment to debug thermal output
  // igndbg << "wxh: " << width << " x " << height << std::endl;
  // for (unsigned int i = 0; i < height; ++i)
  // {
  //   for (unsigned int j = 0; j < width; ++j)
  //   {
  //     igndbg << "[" << this->dataPtr->thermalImage[i*width + j] << "]";
  //   }
  //   igndbg << std::endl;
  // }
}

//////////////////////////////////////////////////
common::ConnectionPtr OgreThermalCamera::ConnectNewThermalFrame(
    std::function<void(const uint16_t *, unsigned int, unsigned int,
      unsigned int, const std::string &)>  _subscriber)
{
  return this->dataPtr->newThermalFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
RenderTargetPtr OgreThermalCamera::RenderTarget() const
{
  return this->dataPtr->thermalTexture;
}
