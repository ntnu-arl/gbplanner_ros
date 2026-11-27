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

#if (_WIN32)
  /* Needed for std::min */
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <windows.h>
#endif
#include <gz/math/Helpers.hh>
#include "gz/rendering/ogre/OgreDepthCamera.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"

/// \internal
/// \brief Private data for the OgreDepthCamera class
class gz::rendering::OgreDepthCameraPrivate
{
  /// \brief The depth buffer
  public: float *depthBuffer = nullptr;

  /// \brief The depth material
  public: MaterialPtr depthMaterial = nullptr;

  /// \brief Point cloud xyz data buffer
  public: float *pcdBuffer = nullptr;

  /// \brief Point cloud view port
  public: Ogre::Viewport *pcdViewport = nullptr;

  /// \brief Point cloud material
  public: MaterialPtr pcdMaterial = nullptr;

  /// \brief Point cloud texture
  public: OgreRenderTexturePtr pcdTexture;

  /// \brief Point cloud texture
  public: OgreRenderTexturePtr colorTexture;

  /// \brief Point cloud color data buffer
  public: unsigned char *colorBuffer = nullptr;

  /// \brief True to output point cloud xyz and rgb data
  public: bool outputPoints = false;

  /// \brief maximum value used for data outside sensor range
  public: float dataMaxVal = gz::math::INF_D;

  /// \brief minimum value used for data outside sensor range
  public: float dataMinVal = -gz::math::INF_D;

  /// \brief Event used to signal rgb point cloud data
  public: gz::common::EventT<void(const float *,
              unsigned int, unsigned int, unsigned int,
              const std::string &)> newRgbPointCloud;

  /// \brief Event used to signal depth data
  public: gz::common::EventT<void(const float *,
              unsigned int, unsigned int, unsigned int,
              const std::string &)> newDepthFrame;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreDepthCamera::OgreDepthCamera()
  : dataPtr(new OgreDepthCameraPrivate())
{
}

//////////////////////////////////////////////////
OgreDepthCamera::~OgreDepthCamera()
{
  this->Destroy();
}

//////////////////////////////////////////////////
void OgreDepthCamera::Destroy()
{
  if (this->dataPtr->depthBuffer)
  {
    delete [] this->dataPtr->depthBuffer;
    this->dataPtr->depthBuffer = nullptr;
  }

  if (this->dataPtr->pcdBuffer)
  {
    delete [] this->dataPtr->pcdBuffer;
    this->dataPtr->pcdBuffer = nullptr;
  }

  if (this->dataPtr->colorBuffer)
  {
    delete [] this->dataPtr->colorBuffer;
    this->dataPtr->colorBuffer = nullptr;
  }

  if (!this->ogreCamera || !this->scene->IsInitialized())
    return;

  this->DestroyPointCloudTexture();
  this->DestroyDepthTexture();

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

  // call base node destroy to remove parent
  OgreNode::Destroy();
}

//////////////////////////////////////////////////
void OgreDepthCamera::Init()
{
  BaseDepthCamera::Init();
  this->CreateCamera();
  this->CreateDepthTexture();
  this->Reset();
}

//////////////////////////////////////////////////
void OgreDepthCamera::CreateCamera()
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
void OgreDepthCamera::CreatePointCloudTexture()
{
  if (this->dataPtr->pcdTexture || this->dataPtr->colorTexture)
    return;

  // color
  RenderTexturePtr colorTextureBase = this->scene->CreateRenderTexture();
  this->dataPtr->colorTexture = std::dynamic_pointer_cast<OgreRenderTexture>(
      colorTextureBase);
  this->dataPtr->colorTexture->SetCamera(this->ogreCamera);
  this->dataPtr->colorTexture->SetFormat(PF_R8G8B8);
  this->dataPtr->colorTexture->SetWidth(this->ImageWidth());
  this->dataPtr->colorTexture->SetHeight(this->ImageHeight());
  this->dataPtr->colorTexture->SetBackgroundColor(
      this->scene->BackgroundColor());
  this->dataPtr->colorTexture->PreRender();

  // point cloud xyz
  RenderTexturePtr pcdTextureBase = this->scene->CreateRenderTexture();
  this->dataPtr->pcdTexture = std::dynamic_pointer_cast<OgreRenderTexture>(
      pcdTextureBase);
  this->dataPtr->pcdTexture->SetCamera(this->ogreCamera);
  this->dataPtr->pcdTexture->SetFormat(PF_FLOAT32_RGBA);
  this->dataPtr->pcdTexture->SetWidth(this->ImageWidth());
  this->dataPtr->pcdTexture->SetHeight(this->ImageHeight());
  this->dataPtr->pcdTexture->SetAntiAliasing(0);
  this->dataPtr->pcdTexture->SetBackgroundColor(
      this->scene->BackgroundColor());

  this->dataPtr->pcdMaterial = this->scene->CreateMaterial();

  const char *env = std::getenv("IGN_RENDERING_RESOURCE_PATH");
  std::string resourcePath = (env) ? std::string(env) :
      IGN_RENDERING_RESOURCE_PATH;

  // path to look for vertex and fragment shader parameters
  std::string pcdVSPath = common::joinPaths(
      resourcePath, "ogre", "media", "materials", "programs",
      "depth_points_vs.glsl");
  std::string pcdFSPath = common::joinPaths(
      resourcePath, "ogre", "media", "materials", "programs",
      "depth_points_fs.glsl");
  this->dataPtr->pcdMaterial->SetVertexShader(pcdVSPath);
  this->dataPtr->pcdMaterial->SetFragmentShader(pcdFSPath);

  this->dataPtr->pcdTexture->SetMaterial(this->dataPtr->pcdMaterial);
  this->dataPtr->pcdTexture->PreRender();
}

//////////////////////////////////////////////////
void OgreDepthCamera::DestroyPointCloudTexture()
{
  if (this->dataPtr->pcdTexture)
  {
    dynamic_cast<OgreRenderTexture *>(this->dataPtr->pcdTexture.get())
      ->Destroy();
    this->dataPtr->pcdTexture.reset();
  }

  if (this->dataPtr->colorTexture)
  {
    dynamic_cast<OgreRenderTexture *>(this->dataPtr->colorTexture.get())
      ->Destroy();
    this->dataPtr->colorTexture.reset();
  }
}

/////////////////////////////////////////////////
void OgreDepthCamera::CreateDepthTexture()
{
  if (this->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
  }

  if (this->depthTexture == nullptr)
  {
    // dummy render texture
    RenderTexturePtr depthTextureBase =
        this->scene->CreateRenderTexture();
    this->depthTexture = std::dynamic_pointer_cast<OgreRenderTexture>(
        depthTextureBase);
    this->depthTexture->SetFormat(PF_FLOAT32_R);
    this->depthTexture->SetCamera(this->ogreCamera);
    this->depthTexture->SetBackgroundColor(this->scene->BackgroundColor());
    this->depthTexture->SetWidth(1);
    this->depthTexture->SetHeight(1);
  }

  double ratio = static_cast<double>(this->ImageWidth()) /
                 static_cast<double>(this->ImageHeight());

  double vfov = 2.0 * atan(tan(this->HFOV().Radian() / 2.0) / ratio);
  this->ogreCamera->setAspectRatio(ratio);
  this->ogreCamera->setFOVy(Ogre::Radian(this->LimitFOV(vfov)));
}

//////////////////////////////////////////////////
void OgreDepthCamera::DestroyDepthTexture()
{
  if (this->depthTexture)
  {
    dynamic_cast<OgreRenderTexture *>(this->depthTexture.get())->Destroy();
    this->depthTexture.reset();
  }
}

//////////////////////////////////////////////////
void OgreDepthCamera::PreRender()
{
  if (!this->depthTexture)
    this->CreateDepthTexture();
  if (!this->dataPtr->pcdTexture || !this->dataPtr->colorTexture)
    this->CreatePointCloudTexture();
}

//////////////////////////////////////////////////
void OgreDepthCamera::Render()
{
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::ShadowTechnique shadowTech = sceneMgr->getShadowTechnique();

  // point cloud xyz and depth
  sceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_NONE);
  sceneMgr->_suppressRenderStateChanges(true);

  this->dataPtr->pcdTexture->SetAutoUpdated(false);
  OgreMaterialPtr ogreMat =
      std::dynamic_pointer_cast<OgreMaterial>(this->dataPtr->pcdMaterial);
  this->UpdateRenderTarget(this->dataPtr->pcdTexture,
      ogreMat->Material().get(), ogreMat->Material()->getName());
  this->dataPtr->pcdTexture->RenderTarget()->update(false);

  sceneMgr->_suppressRenderStateChanges(false);
  sceneMgr->setShadowTechnique(shadowTech);

  // skip color pass if we do not need to output point clouds
  this->dataPtr->outputPoints =
      (this->dataPtr->newRgbPointCloud.ConnectionCount() > 0);
  if (!this->dataPtr->outputPoints)
    return;

  // color
  this->dataPtr->colorTexture->SetAutoUpdated(false);
  this->dataPtr->colorTexture->Render();
}

//////////////////////////////////////////////////
void OgreDepthCamera::UpdateRenderTarget(OgreRenderTexturePtr _target,
    Ogre::Material *_material, const std::string &_matName)

{
  Ogre::RenderTarget *target = _target->RenderTarget();
  std::string matName = _matName;

  Ogre::RenderSystem *renderSys;
  Ogre::Viewport *vp = target->getViewport(0);
  Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
  Ogre::Pass *pass;

  renderSys = this->scene->OgreSceneManager()->getDestinationRenderSystem();
  // Get pointer to the material pass
  pass = _material->getBestTechnique()->getPass(0);

  // Render the depth texture
  // OgreSceneManager::_render function automatically sets farClip to 0.
  // Which normally equates to infinite distance. We don't want this. So
  // we have to set the distance every time.
  this->ogreCamera->setFarClipDistance(this->FarClipPlane());
  this->ogreCamera->setNearClipDistance(1e-4f);

  Ogre::AutoParamDataSource autoParamDataSource;

  // return farClip in case no renderable object is inside frustrum
  vp->setBackgroundColour(Ogre::ColourValue(this->FarClipPlane(),
      this->FarClipPlane(), this->FarClipPlane()));

  Ogre::CompositorManager::getSingleton().setCompositorEnabled(
      vp, matName, true);

  // Need this line to render the ground plane. No idea why it's necessary.
  renderSys->_setViewport(vp);
  sceneMgr->_setPass(pass, true, false);
  autoParamDataSource.setCurrentPass(pass);
  autoParamDataSource.setCurrentViewport(vp);
  autoParamDataSource.setCurrentRenderTarget(target);
  autoParamDataSource.setCurrentSceneManager(sceneMgr);
  autoParamDataSource.setCurrentCamera(this->ogreCamera, true);

  renderSys->setLightingEnabled(false);
  renderSys->_setFog(Ogre::FOG_NONE);

  // These two lines don't seem to do anything useful
  renderSys->_setProjectionMatrix(
      this->ogreCamera->getProjectionMatrixRS());
  renderSys->_setViewMatrix(this->ogreCamera->getViewMatrix(true));

  pass->_updateAutoParams(&autoParamDataSource, 1);

  // NOTE: We MUST bind parameters AFTER updating the autos
  if (pass->hasVertexProgram())
  {
    renderSys->bindGpuProgram(
    pass->getVertexProgram()->_getBindingDelegate());

    renderSys->bindGpuProgramParameters(Ogre::GPT_VERTEX_PROGRAM,
      pass->getVertexProgramParameters(), 1);
  }

  if (pass->hasFragmentProgram())
  {
    renderSys->bindGpuProgram(
    pass->getFragmentProgram()->_getBindingDelegate());

    renderSys->bindGpuProgramParameters(Ogre::GPT_FRAGMENT_PROGRAM,
    pass->getFragmentProgramParameters(), 1);
  }
}

//////////////////////////////////////////////////
void OgreDepthCamera::PostRender()
{
  unsigned int width = this->ImageWidth();
  unsigned int height = this->ImageHeight();
  unsigned int len = width * height;
  double farPlane = this->FarClipPlane();
  double nearPlane = this->NearClipPlane();

  // get depth data
  if (!this->dataPtr->depthBuffer)
    this->dataPtr->depthBuffer = new float[len];
  PixelFormat format = this->dataPtr->pcdTexture->Format();
  unsigned int channelCount = PixelUtil::ChannelCount(format);
  if (!this->dataPtr->pcdBuffer)
    this->dataPtr->pcdBuffer = new float[len * channelCount];
  this->dataPtr->pcdTexture->Buffer(this->dataPtr->pcdBuffer);

  // color data
  unsigned int colorChannelCount = 3;
  int bgColorR = static_cast<int>(this->scene->BackgroundColor().R() * 255);
  int bgColorG = static_cast<int>(this->scene->BackgroundColor().G() * 255);
  int bgColorB = static_cast<int>(this->scene->BackgroundColor().B() * 255);
  int bgColorA = static_cast<int>(this->scene->BackgroundColor().A() * 255);
  if (this->dataPtr->outputPoints)
  {
    PixelFormat colorFormat = this->dataPtr->colorTexture->Format();
    colorChannelCount = PixelUtil::ChannelCount(colorFormat);

    if (!this->dataPtr->colorBuffer)
      this->dataPtr->colorBuffer = new unsigned char[len * colorChannelCount];

    Ogre::PixelBox ogrePixelBox(width, height, 1,
        OgreConversions::Convert(colorFormat), this->dataPtr->colorBuffer);
    this->dataPtr->colorTexture->RenderTarget()->copyContentsToMemory(
        ogrePixelBox);
  }

  // fill depthBuffer and clamp values
  // \todo(anyone) figure out how to do this in shaders?
  for (unsigned int i = 0; i < height; ++i)
  {
    unsigned int step = i*width;
    unsigned int pcdStep = step * channelCount;
    for (unsigned int j = 0; j < width; ++j)
    {
      float *x = &this->dataPtr->pcdBuffer[pcdStep + j*channelCount];
      float *y = &this->dataPtr->pcdBuffer[pcdStep + j*channelCount + 1];
      float *z = &this->dataPtr->pcdBuffer[pcdStep + j*channelCount + 2];

      float depth = *x;
      bool clamp = false;
      // shaders return far for pixels with no depth data
      // manually clamp to max
      if ((*x >= farPlane) && (*y >= farPlane)
          && (*z >= farPlane))
      {
        clamp = true;
        depth = this->dataPtr->dataMaxVal;
        if (this->dataPtr->outputPoints)
        {
          *x = this->dataPtr->dataMaxVal;
          *y = this->dataPtr->dataMaxVal;
          *z = this->dataPtr->dataMaxVal;
        }
      }
      // Manually clamp values to min
      else if ((*x <= nearPlane) && (*y <= nearPlane)
          && (*z <= nearPlane))
      {
        clamp = true;
        depth = this->dataPtr->dataMinVal;
        if (this->dataPtr->outputPoints)
        {
          *x = this->dataPtr->dataMinVal;
          *y = this->dataPtr->dataMinVal;
          *z = this->dataPtr->dataMinVal;
        }
      }
      this->dataPtr->depthBuffer[step + j] = depth;

      // color
      if (this->dataPtr->outputPoints)
      {
        unsigned int colorStep = step * colorChannelCount;
        int r = 0;
        int g = 0;
        int b = 0;
        int a = 255;
        float *color = &this->dataPtr->pcdBuffer[pcdStep + j*channelCount + 3];
        if (clamp)
        {
          r = bgColorR;
          g = bgColorG;
          b = bgColorB;
          a = bgColorA;
        }
        else
        {
          r = this->dataPtr->colorBuffer[colorStep + j*colorChannelCount];
          g = this->dataPtr->colorBuffer[colorStep + j*colorChannelCount + 1];
          b = this->dataPtr->colorBuffer[colorStep + j*colorChannelCount + 2];
        }
        uint32_t rgba = (static_cast<uint8_t>(r) << 24) +
                        (static_cast<uint8_t>(g) << 16) +
                        (static_cast<uint8_t>(b) << 8) +
                        (static_cast<uint8_t>(a) << 0);
        // cppcheck-suppress invalidPointerCast
        float *c = reinterpret_cast<float *>(&rgba);
        *color = *c;
      }
    }
  }

  this->dataPtr->newDepthFrame(
      this->dataPtr->depthBuffer, width, height, 1, "FLOAT32");

  // point cloud
  if (this->dataPtr->outputPoints)
  {
    this->dataPtr->newRgbPointCloud(
        this->dataPtr->pcdBuffer, width, height, channelCount,
        "PF_FLOAT32_RGBA");

    // Uncomment to debug xyz output
    // igndbg << "wxh: " << width << " x " << height << std::endl;
    // for (unsigned int i = 0; i < height; ++i)
    // {
    //   for (unsigned int j = 0; j < width; ++j)
    //   {
    //     igndbg << "[" << this->dataPtr->pcdBuffer[i*width*4+j*4] << "]"
    //       << "[" << this->dataPtr->pcdBuffer[i*width*4+j*4+1] << "]"
    //       << "[" << this->dataPtr->pcdBuffer[i*width*4+j*4+2] << "],";
    //   }
    //   igndbg << std::endl;
    // }

    // Uncommnet to debug color output
    // for (unsigned int i = 0; i < height; ++i)
    // {
    //   for (unsigned int j = 0; j < width; ++j)
    //   {
    //     float c = this->dataPtr->pcdBuffer[i*width*4 + j*4 + 3];
    //     uint32_t *rgba = reinterpret_cast<uint32_t *>(&c);
    //     unsigned int r = *rgba >> 24 & 0xFF;
    //     unsigned int g = *rgba >> 16 & 0xFF;
    //     unsigned int b = *rgba >> 8 & 0xFF;
    //     igndbg << "[" << r << "]"
    //            << "[" << g << "]"
    //            << "[" << b << "],";
    //   }
    //   igndbg << std::endl;
    // }
  }
}

//////////////////////////////////////////////////
const float *OgreDepthCamera::DepthData() const
{
  return this->dataPtr->depthBuffer;
}

//////////////////////////////////////////////////
common::ConnectionPtr OgreDepthCamera::ConnectNewDepthFrame(
    std::function<void(const float *, unsigned int, unsigned int,
      unsigned int, const std::string &)>  _subscriber)
{
  return this->dataPtr->newDepthFrame.Connect(_subscriber);
}

//////////////////////////////////////////////////
common::ConnectionPtr OgreDepthCamera::ConnectNewRgbPointCloud(
    std::function<void(const float *, unsigned int, unsigned int,
      unsigned int, const std::string &)>  _subscriber)
{
  return this->dataPtr->newRgbPointCloud.Connect(_subscriber);
}

//////////////////////////////////////////////////
RenderTargetPtr OgreDepthCamera::RenderTarget() const
{
  return this->depthTexture;
}

//////////////////////////////////////////////////
double OgreDepthCamera::LimitFOV(const double _fov)
{
  return std::min(std::max(0.001, _fov), IGN_PI * 0.999);
}

//////////////////////////////////////////////////
void OgreDepthCamera::SetNearClipPlane(const double _near)
{
  BaseDepthCamera::SetNearClipPlane(_near);
}

//////////////////////////////////////////////////
void OgreDepthCamera::SetFarClipPlane(const double _far)
{
  BaseDepthCamera::SetFarClipPlane(_far);
  this->ogreCamera->setFarClipDistance(_far);
}

//////////////////////////////////////////////////
double OgreDepthCamera::NearClipPlane() const
{
  return BaseDepthCamera::NearClipPlane();
}

//////////////////////////////////////////////////
double OgreDepthCamera::FarClipPlane() const
{
  if (this->ogreCamera)
    return this->ogreCamera->getFarClipDistance();
  else
    return 0;
}
