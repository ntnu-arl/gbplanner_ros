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

#include <gz/common/Console.hh>

#include "gz/rendering/ogre/OgreArrowVisual.hh"
#include "gz/rendering/ogre/OgreAxisVisual.hh"
#include "gz/rendering/ogre/OgreCamera.hh"
#include "gz/rendering/ogre/OgreCapsule.hh"
#include "gz/rendering/ogre/OgreCOMVisual.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreDepthCamera.hh"
#include "gz/rendering/ogre/OgreGeometry.hh"
#include "gz/rendering/ogre/OgreGizmoVisual.hh"
#include "gz/rendering/ogre/OgreGpuRays.hh"
#include "gz/rendering/ogre/OgreGrid.hh"
#include "gz/rendering/ogre/OgreHeightmap.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreInertiaVisual.hh"
#include "gz/rendering/ogre/OgreJointVisual.hh"
#include "gz/rendering/ogre/OgreLidarVisual.hh"
#include "gz/rendering/ogre/OgreLightVisual.hh"
#include "gz/rendering/ogre/OgreMarker.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreMeshFactory.hh"
#include "gz/rendering/ogre/OgreParticleEmitter.hh"
#include "gz/rendering/ogre/OgreRTShaderSystem.hh"
#include "gz/rendering/ogre/OgreRayQuery.hh"
#include "gz/rendering/ogre/OgreRenderEngine.hh"
#include "gz/rendering/ogre/OgreRenderTarget.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreStorage.hh"
#include "gz/rendering/ogre/OgreText.hh"
#include "gz/rendering/ogre/OgreThermalCamera.hh"
#include "gz/rendering/ogre/OgreVisual.hh"
#include "gz/rendering/ogre/OgreWireBox.hh"

namespace ignition
{
  namespace rendering
  {

    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Subclassing the Ogre Rectangle2D class to create a gradient
    /// colored rectangle. The class is setting colors at the four vertices
    /// (corners) of the rectangle and Ogre/OpenGL then interpolates the colors
    /// between the vertices.
    /// \ref https://forums.ogre3d.org/viewtopic.php?f=2&t=60677
    class ColoredRectangle2D : public Ogre::Rectangle2D
    {
      // Documentation inherited
      public: explicit ColoredRectangle2D(
          bool _includeTextureCoordinates = false)
        : Ogre::Rectangle2D(_includeTextureCoordinates)
      {
        Ogre::VertexDeclaration* decl = mRenderOp.vertexData->vertexDeclaration;

        decl->addElement(this->kColorBinding, 0,
            Ogre::VET_COLOUR, Ogre::VES_DIFFUSE);
        Ogre::VertexBufferBinding* bind =
            mRenderOp.vertexData->vertexBufferBinding;

        Ogre::HardwareVertexBufferSharedPtr vbuf =
          Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
            decl->getVertexSize(this->kColorBinding),
            mRenderOp.vertexData->vertexCount,
            Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY);

        // Bind buffer
        bind->setBinding(this->kColorBinding, vbuf);
      }

      /// \brief Class destructor.
      public: ~ColoredRectangle2D()
      {
      }

      /// \brief Set the degraded colors of the rectangle
      /// \param[in] _topLeft Top left color
      /// \param[in] _bottomLeft Bottom left color
      /// \param[in] _topRight Top right color
      /// \param[in] _bottomRight Bottom right color
      public: void SetColors(const Ogre::ColourValue &_topLeft,
                             const Ogre::ColourValue &_bottomLeft,
                             const Ogre::ColourValue &_topRight,
                             const Ogre::ColourValue &_bottomRight)
      {
        Ogre::HardwareVertexBufferSharedPtr vbuf =
            mRenderOp.vertexData->vertexBufferBinding->getBuffer(
                this->kColorBinding);
        unsigned int* pUint32 =
          static_cast<unsigned int*>(vbuf->lock(
          Ogre::HardwareBuffer::HBL_DISCARD));

        const Ogre::VertexElementType srcType =
          Ogre::VertexElement::getBestColourVertexElementType();

        *pUint32++ = Ogre::VertexElement::convertColourValue(_topLeft, srcType);
        *pUint32++ =
            Ogre::VertexElement::convertColourValue(_bottomLeft, srcType);
        *pUint32++ =
            Ogre::VertexElement::convertColourValue(_topRight, srcType);
        *pUint32++ =
            Ogre::VertexElement::convertColourValue(_bottomRight, srcType);

        vbuf->unlock();
      }

      /// \brief Index associated with the vertex buffer.
      private: const int kColorBinding = 3;
    };
    }
  }
}

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreScene::OgreScene(unsigned int _id, const std::string &_name) :
  BaseScene(_id, _name),
  rootVisual(nullptr),
  meshFactory(nullptr),
  ogreRoot(nullptr),
  ogreSceneManager(nullptr)
{
  this->backgroundColor = math::Color::Black;
  this->gradientBackgroundColor = {math::Color::Black, math::Color::Black,
      math::Color::Black, math::Color::Black};
}

//////////////////////////////////////////////////
OgreScene::~OgreScene()
{
}

//////////////////////////////////////////////////
void OgreScene::Fini()
{
}

//////////////////////////////////////////////////
RenderEngine *OgreScene::Engine() const
{
  return OgreRenderEngine::Instance();
}

//////////////////////////////////////////////////
VisualPtr OgreScene::RootVisual() const
{
  return this->rootVisual;
}

//////////////////////////////////////////////////
math::Color OgreScene::AmbientLight() const
{
  Ogre::ColourValue ogreColor = this->ogreSceneManager->getAmbientLight();
  return OgreConversions::Convert(ogreColor);
}

//////////////////////////////////////////////////
void OgreScene::SetAmbientLight(const math::Color &_color)
{
  Ogre::ColourValue ogreColor = OgreConversions::Convert(_color);
  this->ogreSceneManager->setAmbientLight(ogreColor);
}

//////////////////////////////////////////////////
void OgreScene::SetBackgroundColor(const math::Color &_color)
{
  this->backgroundColor = _color;

  // TODO(anyone): clean up code
  unsigned int count = this->SensorCount();

  for (unsigned int i = 0; i < count; ++i)
  {
    SensorPtr sensor = this->SensorByIndex(i);
    OgreCameraPtr camera = std::dynamic_pointer_cast<OgreCamera>(sensor);
    if (camera) camera->SetBackgroundColor(_color);
  }
}

//////////////////////////////////////////////////
void OgreScene::SetGradientBackgroundColor(
    const std::array<math::Color, 4> &_colors)
{
  ColoredRectangle2D* rect = nullptr;
  Ogre::SceneNode *backgroundNodePtr = nullptr;

  // Check if we have created the scene node to render the gradient background
  if (!this->ogreSceneManager->hasSceneNode("Background"))
  {
    // Create background material
    Ogre::MaterialPtr material =
      Ogre::MaterialManager::getSingleton().create("Background", "General");
    material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
    material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    material->getTechnique(0)->getPass(0)->setLightingEnabled(false);

    // Create background rectangle covering the whole screen
    rect = new ColoredRectangle2D();
    rect->setCorners(-1.0, 1.0, 1.0, -1.0);
#if OGRE_VERSION_LT_1_11_0
    rect->setMaterial("Background");
#else
    rect->setMaterial(material);
#endif
    // Render the background before everything else
    rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

    // Use infinite AAB to always stay visible
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    rect->setBoundingBox(aabInf);

    // Attach background to the scene
    backgroundNodePtr = this->ogreSceneManager->getRootSceneNode()->
        createChildSceneNode("Background");
    backgroundNodePtr->attachObject(rect);
  }

  backgroundNodePtr = this->ogreSceneManager->getSceneNode("Background");

  auto coloredRectangle2D = backgroundNodePtr->getAttachedObject(0);
  if (!coloredRectangle2D)
  {
    ignerr << "Unable to find the background attached object" << std::endl;
    return;
  }

  rect = dynamic_cast<ColoredRectangle2D *>(coloredRectangle2D);
  if (!rect)
  {
    ignerr << "Unable to cast from Ogre::MovableObject to ColouredRectangle2D"
           << std::endl;
    return;
  }

  // Convert the math::Color to Ogre::ColourValue.
  std::array<Ogre::ColourValue, 4> ogreColors;
  for (auto i = 0u; i < 4; ++i)
    ogreColors[i].setAsRGBA(_colors[i].AsRGBA());

  rect->SetColors(ogreColors[0], ogreColors[1], ogreColors[2], ogreColors[3]);
  rect->setVisible(true);

  this->gradientBackgroundColor = _colors;
  this->isGradientBackgroundColor = true;
}

//////////////////////////////////////////////////
void OgreScene::RemoveGradientBackgroundColor()
{
  // If the gradient background color is set, we should make it invisible,
  // otherwise the background color will not be visible.
  if (this->ogreSceneManager->hasSceneNode("Background"))
  {
    auto backgroundNodePtr = this->ogreSceneManager->getSceneNode("Background");
    auto coloredRectangle2D = backgroundNodePtr->getAttachedObject(0);
    if (coloredRectangle2D && coloredRectangle2D->isVisible())
      coloredRectangle2D->setVisible(false);
  }

  this->isGradientBackgroundColor = false;
}

//////////////////////////////////////////////////
void OgreScene::PreRender()
{
  BaseScene::PreRender();
  OgreRTShaderSystem::Instance()->Update();
}

//////////////////////////////////////////////////
void OgreScene::Clear()
{
  BaseScene::Clear();
}

//////////////////////////////////////////////////
void OgreScene::Destroy()
{
  BaseScene::Destroy();

  // ogre scene manager is destroyed when ogre root is deleted
  // removing here seems to cause the system to freeze on deleting rthsader
  // system when unloading the engine
  this->ogreSceneManager = nullptr;
}

//////////////////////////////////////////////////
Ogre::SceneManager *OgreScene::OgreSceneManager() const
{
  return this->ogreSceneManager;
}

//////////////////////////////////////////////////
bool OgreScene::LoadImpl()
{
  return true;
}

//////////////////////////////////////////////////
bool OgreScene::InitImpl()
{
  this->CreateContext();
  this->CreateRootVisual();
  this->CreateStores();
  this->CreateMeshFactory();

  OgreRTShaderSystem::Instance()->AddScene(this->SharedThis());
  OgreRTShaderSystem::Instance()->ApplyShadows(this->SharedThis());

  return true;
}

//////////////////////////////////////////////////
LightStorePtr OgreScene::Lights() const
{
  return this->lights;
}

//////////////////////////////////////////////////
SensorStorePtr OgreScene::Sensors() const
{
  return this->sensors;
}

//////////////////////////////////////////////////
VisualStorePtr OgreScene::Visuals() const
{
  return this->visuals;
}

//////////////////////////////////////////////////
void OgreScene::ClearMaterialsCache(const std::string &_name)
{
  this->meshFactory->ClearMaterialsCache(_name);
}

//////////////////////////////////////////////////
MaterialMapPtr OgreScene::Materials() const
{
  return this->materials;
}

//////////////////////////////////////////////////
DirectionalLightPtr OgreScene::CreateDirectionalLightImpl(unsigned int _id,
    const std::string &_name)
{
  OgreDirectionalLightPtr light(new OgreDirectionalLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
PointLightPtr OgreScene::CreatePointLightImpl(unsigned int _id,
    const std::string &_name)
{
  OgrePointLightPtr light(new OgrePointLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
COMVisualPtr OgreScene::CreateCOMVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreCOMVisualPtr visual(new OgreCOMVisual);
    bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

InertiaVisualPtr OgreScene::CreateInertiaVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreInertiaVisualPtr visual(new OgreInertiaVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
JointVisualPtr OgreScene::CreateJointVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreJointVisualPtr visual(new OgreJointVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
LightVisualPtr OgreScene::CreateLightVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreLightVisualPtr visual(new OgreLightVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
SpotLightPtr OgreScene::CreateSpotLightImpl(unsigned int _id,
    const std::string &_name)
{
  OgreSpotLightPtr light(new OgreSpotLight);
  bool result = this->InitObject(light, _id, _name);
  return (result) ? light : nullptr;
}

//////////////////////////////////////////////////
CameraPtr OgreScene::CreateCameraImpl(unsigned int _id,
    const std::string &_name)
{
  OgreCameraPtr camera(new OgreCamera);
  bool result = this->InitObject(camera, _id, _name);
  camera->SetBackgroundColor(this->backgroundColor);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
DepthCameraPtr OgreScene::CreateDepthCameraImpl(const unsigned int _id,
    const std::string &_name)
{
  OgreDepthCameraPtr camera(new OgreDepthCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

//////////////////////////////////////////////////
ThermalCameraPtr OgreScene::CreateThermalCameraImpl(const unsigned int _id,
    const std::string &_name)
{
  OgreThermalCameraPtr camera(new OgreThermalCamera);
  bool result = this->InitObject(camera, _id, _name);
  return (result) ? camera : nullptr;
}

///////////////////////////////////////////////////
GpuRaysPtr OgreScene::CreateGpuRaysImpl(const unsigned int _id,
    const std::string &_name)
{
  OgreGpuRaysPtr gpuRays(new OgreGpuRays);
  bool result = this->InitObject(gpuRays, _id, _name);
  return (result) ? gpuRays : nullptr;
}

////////////////////////////////////////////////
VisualPtr OgreScene::CreateVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreVisualPtr visual(new OgreVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
ArrowVisualPtr OgreScene::CreateArrowVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreArrowVisualPtr visual(new OgreArrowVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
AxisVisualPtr OgreScene::CreateAxisVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreAxisVisualPtr visual(new OgreAxisVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
GizmoVisualPtr OgreScene::CreateGizmoVisualImpl(unsigned int _id,
    const std::string &_name)
{
  OgreGizmoVisualPtr visual(new OgreGizmoVisual);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
GeometryPtr OgreScene::CreateBoxImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_box");
}

//////////////////////////////////////////////////
GeometryPtr OgreScene::CreateConeImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_cone");
}

//////////////////////////////////////////////////
GeometryPtr OgreScene::CreateCylinderImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_cylinder");
}

//////////////////////////////////////////////////
GeometryPtr OgreScene::CreatePlaneImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_plane");
}

//////////////////////////////////////////////////
GeometryPtr OgreScene::CreateSphereImpl(unsigned int _id,
    const std::string &_name)
{
  return this->CreateMeshImpl(_id, _name, "unit_sphere");
}

//////////////////////////////////////////////////
MeshPtr OgreScene::CreateMeshImpl(unsigned int _id, const std::string &_name,
    const std::string &_meshName)
{
  MeshDescriptor descriptor(_meshName);
  return this->CreateMeshImpl(_id, _name, descriptor);
}

//////////////////////////////////////////////////
MeshPtr OgreScene::CreateMeshImpl(unsigned int _id, const std::string &_name,
    const MeshDescriptor &_desc)
{
  OgreMeshPtr mesh = this->meshFactory->Create(_desc);
  if (nullptr == mesh)
    return nullptr;

  mesh->SetDescriptor(_desc);
  bool result = this->InitObject(mesh, _id, _name);
  return (result) ? mesh : nullptr;
}

//////////////////////////////////////////////////
CapsulePtr OgreScene::CreateCapsuleImpl(
  unsigned int _id, const std::string &_name)
{
  OgreCapsulePtr capsule(new OgreCapsule);
  bool result = this->InitObject(capsule, _id, _name);
  return (result) ? capsule : nullptr;
}

//////////////////////////////////////////////////
HeightmapPtr OgreScene::CreateHeightmapImpl(unsigned int _id,
    const std::string &_name, const HeightmapDescriptor &_desc)
{
  OgreHeightmapPtr heightmap;
  heightmap.reset(new OgreHeightmap(_desc));
  bool result = this->InitObject(heightmap, _id, _name);
  return (result) ? heightmap : nullptr;
}

//////////////////////////////////////////////////
GridPtr OgreScene::CreateGridImpl(unsigned int _id, const std::string &_name)
{
  OgreGridPtr grid(new OgreGrid);
  bool result = this->InitObject(grid, _id, _name);
  return (result) ? grid: nullptr;
}

//////////////////////////////////////////////////
WireBoxPtr OgreScene::CreateWireBoxImpl(unsigned int _id,
                                      const std::string &_name)
{
  OgreWireBoxPtr wireBox(new OgreWireBox);
  bool result = this->InitObject(wireBox, _id, _name);
  return (result) ? wireBox: nullptr;
}

//////////////////////////////////////////////////
MarkerPtr OgreScene::CreateMarkerImpl(unsigned int _id,
                                      const std::string &_name)
{
  OgreMarkerPtr marker(new OgreMarker);
  bool result = this->InitObject(marker, _id, _name);
  return (result) ? marker: nullptr;
}

//////////////////////////////////////////////////
LidarVisualPtr OgreScene::CreateLidarVisualImpl(unsigned int _id,
                                      const std::string &_name)
{
  OgreLidarVisualPtr lidar(new OgreLidarVisual);
  bool result = this->InitObject(lidar, _id, _name);
  return (result) ? lidar: nullptr;
}

//////////////////////////////////////////////////
TextPtr OgreScene::CreateTextImpl(unsigned int _id, const std::string &_name)
{
  OgreTextPtr text(new OgreText);
  bool result = this->InitObject(text, _id, _name);
  return (result) ? text: nullptr;
}

//////////////////////////////////////////////////
MaterialPtr OgreScene::CreateMaterialImpl(unsigned int _id,
    const std::string &_name)
{
  OgreMaterialPtr material(new OgreMaterial);
  bool result = this->InitObject(material, _id, _name);
  return (result) ? material : nullptr;
}

//////////////////////////////////////////////////
RenderTexturePtr OgreScene::CreateRenderTextureImpl(unsigned int _id,
    const std::string &_name)
{
  OgreRenderTexturePtr renderTexture(new OgreRenderTexture);
  bool result = this->InitObject(renderTexture, _id, _name);
  return (result) ? renderTexture : nullptr;
}

//////////////////////////////////////////////////
RenderWindowPtr OgreScene::CreateRenderWindowImpl(unsigned int _id,
    const std::string &_name)
{
  OgreRenderWindowPtr renderWindow(new OgreRenderWindow);
  bool result = this->InitObject(renderWindow, _id, _name);
  return (result) ? renderWindow: nullptr;
}

//////////////////////////////////////////////////
RayQueryPtr OgreScene::CreateRayQueryImpl(unsigned int _id,
    const std::string &_name)
{
  OgreRayQueryPtr rayQuery(new OgreRayQuery);
  bool result = this->InitObject(rayQuery, _id, _name);
  return (result) ? rayQuery : nullptr;
}

//////////////////////////////////////////////////
ParticleEmitterPtr OgreScene::CreateParticleEmitterImpl(unsigned int _id,
    const std::string &_name)
{
  OgreParticleEmitterPtr visual(new OgreParticleEmitter);
  bool result = this->InitObject(visual, _id, _name);
  return (result) ? visual : nullptr;
}

//////////////////////////////////////////////////
bool OgreScene::InitObject(OgreObjectPtr _object, unsigned int _id,
    const std::string &_name)
{
  // assign needed varibles
  _object->id = _id;
  _object->name = _name;
  _object->scene = this->SharedThis();

  // initialize object
  _object->Load();
  _object->Init();

  return true;
}

//////////////////////////////////////////////////
void OgreScene::CreateContext()
{
  Ogre::Root *root = OgreRenderEngine::Instance()->OgreRoot();
  this->ogreSceneManager = root->createSceneManager(Ogre::ST_GENERIC);

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
  this->ogreSceneManager->addRenderQueueListener(
      OgreRenderEngine::Instance()->OverlaySystem());
#endif
}

//////////////////////////////////////////////////
void OgreScene::CreateRootVisual()
{
  // create unregistered visual
  this->rootVisual = OgreVisualPtr(new OgreVisual);
  unsigned int rootId = this->CreateObjectId();
  std::string rootName = this->CreateObjectName(rootId, "_ROOT_");

  // check if root visual created successfully
  if (!this->InitObject(this->rootVisual, rootId, rootName))
  {
    ignerr << "Unable to create root visual" << std::endl;
    this->rootVisual = nullptr;
  }

  // add visual node to actual ogre root
  Ogre::SceneNode *ogreRootNode = this->rootVisual->Node();
  this->ogreSceneManager->getRootSceneNode()->addChild(ogreRootNode);
}

//////////////////////////////////////////////////
void OgreScene::CreateMeshFactory()
{
  OgreScenePtr sharedThis = this->SharedThis();
  this->meshFactory = OgreMeshFactoryPtr(new OgreMeshFactory(sharedThis));
}

//////////////////////////////////////////////////
void OgreScene::CreateStores()
{
  this->lights = OgreLightStorePtr(new OgreLightStore);
  this->sensors = OgreSensorStorePtr(new OgreSensorStore);
  this->visuals = OgreVisualStorePtr(new OgreVisualStore);
  this->materials = OgreMaterialMapPtr(new OgreMaterialMap);
}

//////////////////////////////////////////////////
OgreScenePtr OgreScene::SharedThis()
{
  ScenePtr sharedBase = this->shared_from_this();
  return std::dynamic_pointer_cast<OgreScene>(sharedBase);
}
