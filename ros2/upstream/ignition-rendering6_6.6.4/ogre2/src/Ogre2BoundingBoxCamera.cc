/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include <limits>

#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable:5033)
#endif
#include <OgreBitwise.h>
#ifdef _MSC_VER
#pragma warning(pop)
#endif

#include <ignition/common/Console.hh>

#include <ignition/math/Color.hh>
#include <ignition/math/Vector4.hh>
#include <ignition/math/eigen3/Util.hh>
#include <ignition/math/OrientedBox.hh>

#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Utils.hh"
#include "ignition/rendering/ogre2/Ogre2BoundingBoxCamera.hh"
#include "ignition/rendering/ogre2/Ogre2Camera.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTarget.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"

#include "Ogre2BoundingBoxMaterialSwitcher.hh"

using namespace ignition;
using namespace rendering;

class ignition::rendering::Ogre2BoundingBoxCameraPrivate
{
  /// \brief Merge a vector of 2D boxes. Used in multi-links model.
  /// \param[in] _boxes Vector of 2D boxes
  /// \return Merged bounding box
  public: BoundingBox MergeBoxes2D(
    const std::vector<std::shared_ptr<BoundingBox>> &_boxes);

  /// \brief Get the 3d vertices (in world coord.) of the item's that
  /// belongs to the same parent (only used in multi-links models)
  /// \param[in] _ogreIds vector of ogre ids that belongs to the same model
  /// \param[out] _vertices vector of 3d vertices of the item
  public: void MeshVertices(const std::vector<uint32_t> &_ogreIds,
              std::vector<math::Vector3d> &_vertices);

  /// \brief Add a line to the viewport. If the line's endpoints are not inside
  /// the viewport, the added line will be a clipped line that fits in the
  /// viewport. If the line to be added doesn't intersect the viewport at all,
  /// the line won't be saved to _lines
  /// \param[in] _bounds The bounds of the viewport. Order of the vector should
  /// be: xmin, ymin, xmax, ymax
  /// \param[in] _p0 The line's first endpoint
  /// \param[in] _p1 The line's second endpoint
  /// \param[out] _lines The list of lines in the viewport. The line
  /// computed from _p0 and _p1 will be added to this list
  public: void AddToViewportLines(const math::Vector4d &_bounds,
              const math::Vector2d &_p0, const math::Vector2d &_p1,
              std::vector<math::Vector2d> &_lines) const;

  /// \brief Clip a line to be within the bounds of a viewport. Using:
  /// https://en.wikipedia.org/wiki/Cohen%E2%80%93Sutherland_algorithm
  /// \param[in] _bounds The bounds of the viewport. Order of the vector should
  /// be: xmin, ymin, xmax, ymax
  /// \param[in] _p0 The line's first endpoint
  /// \param[in] _p1 The line's second endpoint
  /// \return The new endpoints for the clipped line in the viewport. The first
  /// point in the pair is the clipped point for _p0, and the second
  /// point in the pair is the clipped point for _p1
  private: std::pair<math::Vector2d, math::Vector2d> ClipToViewPort(
              const math::Vector4d &_bounds, const math::Vector2d &_p0,
              const math::Vector2d &_p1) const;

  /// \brief Determine where a point is relative to the viewport
  /// \param[in] _bounds The bounds of the viewport. Order of the vector should
  /// be: xmin, ymin, xmax, ymax
  /// \param[in] _x The x coordinate of the point
  /// \param[in] _y The y coordinate of the point
  /// \return The location, which is a bitwise combination of the following:
  ///   INSIDE = 0 (0000)
  ///   LEFT   = 1 (0001)
  ///   RIGHT  = 2 (0010)
  ///   BOTTOM = 4 (0100)
  ///   TOP    = 8 (1000)
  private: int LocationRelativeToViewPort(const math::Vector4d &_bounds,
               double _x, double _y) const;

  /// \brief Material Switcher to switch item's material with ogre Ids
  /// For bounding boxes visibility checking & finding boundaires
  public: std::unique_ptr<Ogre2BoundingBoxMaterialSwitcher> materialSwitcher;

  /// \brief Workspace to interface with render texture
  public: Ogre::CompositorWorkspace *ogreCompositorWorkspace {nullptr};

  /// \brief Workspace Definition
  public: std::string workspaceDefinition;

  /// \brief Pointer to the ogre camera
  public: Ogre::Camera *ogreCamera{nullptr};

  /// \brief Texture to create the render texture from.
  public: Ogre::TextureGpu *ogreRenderTexture {nullptr};

  /// \brief Buffer to store render texture data & to be sent to listeners
  public: uint8_t *buffer = nullptr;

  /// \brief Dummy render texture to set image dims
  public: Ogre2RenderTexturePtr dummyTexture {nullptr};

  /// \brief New BoundingBox Frame Event to notify listeners with new data
  public: common::EventT<void(const std::vector<BoundingBox> &)>
        newBoundingBoxes;

  /// \brief Image / Render Texture Format
  public: Ogre::PixelFormatGpu format = Ogre::PFG_RGBA8_UNORM;

  /// \brief map ogreId id to bounding box
  /// Key: ogreId, value: bounding box contains max & min boundaries
  public: std::map<uint32_t, std::shared_ptr<BoundingBox>> boundingboxes;

  /// \brief Keep track of the visible bounding boxes (used in filtering)
  /// Key: ogreId, value: label id
  public: std::map<uint32_t, uint32_t> visibleBoxesLabel;

  /// \brief Map parent name of the visual to the boxes in it to merge them.
  /// Used in multi-link models, as each parent contains many boxes in it.
  /// Key: parent name, value: vector of boxes that belongs to that model.
  public: std::map<std::string, std::vector<std::shared_ptr<BoundingBox>>>
      parentNameToBoxes;

  /// \brief Keep track of the visible bounding boxes (used in filtering)
  /// Key: parent name, value: vector of ogre ids of it's childern
  public: std::map<std::string, std::vector<uint32_t>> parentNameToOgreIds;

  /// \brief The ogre item's 3d vertices from the vao(used in multi-link models)
  /// Key: ogre id, value: vector of it's 3d vertices(pointcloud or mesh points)
  public: std::map<uint32_t, std::vector<math::Vector3d>> itemVertices;

  /// \brief Map ogre id to Ogre::Item (used in multi-link models)
  /// Key: ogre id, value: ogre item pointer
  public: std::map<uint32_t, Ogre::Item *> ogreIdToItem;

  /// \brief Output bounding boxes to notify listeners
  public: std::vector<BoundingBox> outputBoxes;

  /// \brief Bounding Box type
  public: BoundingBoxType type {BoundingBoxType::BBT_VISIBLEBOX2D};

  /// \brief Alias variable that's used in the ClipToViewPort and
  /// LocationRelativeToViewPort methods.
  /// Binary representation of 0000
  private: const int kInside = 0;

  /// \brief Alias variable that's used in the ClipToViewPort and
  /// LocationRelativeToViewPort methods.
  /// Binary representation of 0001
  private: const int kLeft = 1;

  /// \brief Alias variable that's used in the ClipToViewPort and
  /// LocationRelativeToViewPort methods.
  /// Binary representation of 0010
  private: const int kRight = 2;

  /// \brief Alias variable that's used in the ClipToViewPort and
  /// LocationRelativeToViewPort methods.
  /// Binary representation of 0100
  private: const int kBottom = 4;

  /// \brief Alias variable that's used in the ClipToViewPort and
  /// LocationRelativeToViewPort methods.
  /// Binary representation of 1000
  private: const int kTop = 8;
};

/////////////////////////////////////////////////
void Ogre2BoundingBoxCameraPrivate::AddToViewportLines(
    const math::Vector4d &_bounds, const math::Vector2d &_p0,
    const math::Vector2d &_p1, std::vector<math::Vector2d> &_lines) const
{
  auto endpoints = this->ClipToViewPort(_bounds, _p0, _p1);
  if (endpoints.first.IsFinite() && endpoints.second.IsFinite())
  {
    _lines.push_back(endpoints.first);
    _lines.push_back(endpoints.second);
  }
}

/////////////////////////////////////////////////
std::pair<math::Vector2d, math::Vector2d>
Ogre2BoundingBoxCameraPrivate::ClipToViewPort(const math::Vector4d &_bounds,
    const math::Vector2d &_p0, const math::Vector2d &_p1) const
{
  const auto xmin = _bounds[0];
  const auto ymin = _bounds[1];
  const auto xmax = _bounds[2];
  const auto ymax = _bounds[3];

  auto x0 = _p0.X();
  auto y0 = _p0.Y();
  auto x1 = _p1.X();
  auto y1 = _p1.Y();

  auto location0 = this->LocationRelativeToViewPort(_bounds, x0, y0);
  auto location1 = this->LocationRelativeToViewPort(_bounds, x1, y1);
  bool accept = false;

  while (true)
  {
    if (!(location0 | location1))
    {
      // bitwise OR is 0, which means both endpoints are in the bounds
      accept = true;
      break;
    }
    else if (location0 & location1)
    {
      // bitwise AND is not 0, which means that both points share a zone
      // outside (left, right, top, bottom) of the window. This means that the
      // line formed by these points does not appear in the window formed by the
      // bounds. Reject this line
      break;
    }
    else
    {
      // calculate the line segment to clip from an outside point to an
      // intersection with clip edge
      double x = 0.0;
      double y = 0.0;

      // at least one endpoint is outside the clip rectangle; pick it
      int outerLocation = location1 > location0 ? location1 : location0;

      // Find the intersection point. Use formulas:
      //    slope = (y1 - y0) / (x1 - x0)
      //    x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
      //    y = y0 + slope * (xm - x0), where xm is xmin or xmax
      // Divide by zero wont happen because in each case, outerLocation bit
      // being tested guarantees the denominator is non-zero
      if (outerLocation & this->kTop)
      {
        // point is above clip window
        x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
        y = ymax;
      }
      else if (outerLocation & this->kBottom)
      {
        // point is below clip window
        x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
        y = ymin;
      }
      else if (outerLocation & this->kRight)
      {
        // point is to the right of clip window
        y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
        x = xmax;
      }
      else if (outerLocation & this->kLeft)
      {
        // point is to the left of clip window
        y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
        x = xmin;
      }
      else
      {
        ignerr << "Internal error: no point was found outside of the clip "
               << "window\n";
        break;
      }

      // update the outside point to the intersection point
      if (outerLocation == location0)
      {
        x0 = x;
        y0 = y;
        location0 = this->LocationRelativeToViewPort(_bounds, x0, y0);
      }
      else
      {
        x1 = x;
        y1 = y;
        location1 = this->LocationRelativeToViewPort(_bounds, x1, y1);
      }
    }
  }

  if (accept)
    return {math::Vector2d(x0, y0), math::Vector2d(x1, y1)};

  return {math::Vector2d::NaN, math::Vector2d::NaN};
}

/////////////////////////////////////////////////
int Ogre2BoundingBoxCameraPrivate::LocationRelativeToViewPort(
    const math::Vector4d &_bounds, double _x, double _y) const
{
  // Relative location is a bitwise combination of:
  //   inside = 0 (0000)
  //   left   = 1 (0001)
  //   right  = 2 (0010)
  //   bottom = 4 (0100)
  //   top    = 8 (1000)

  const auto xmin = _bounds[0];
  const auto ymin = _bounds[1];
  const auto xmax = _bounds[2];
  const auto ymax = _bounds[3];

  // initialize the point as being inside the bounds
  int relativeLocation = this->kInside;

  // to the left
  if (_x < xmin)
    relativeLocation |= this->kLeft;
  // to the right
  else if (_x > xmax)
    relativeLocation |= this->kRight;

  // below
  if (_y < ymin)
    relativeLocation |= this->kBottom;
  // above
  else if (_y > ymax)
    relativeLocation |= this->kTop;

  return relativeLocation;
}

/////////////////////////////////////////////////
Ogre2BoundingBoxCamera::Ogre2BoundingBoxCamera() :
  dataPtr(std::make_unique<Ogre2BoundingBoxCameraPrivate>())
{
}

/////////////////////////////////////////////////
Ogre2BoundingBoxCamera::~Ogre2BoundingBoxCamera()
{
  this->Destroy();
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::Init()
{
  BaseCamera::Init();
  this->CreateCamera();
  this->CreateRenderTexture();

  this->dataPtr->materialSwitcher = std::make_unique<
    Ogre2BoundingBoxMaterialSwitcher>(this->scene);
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::CreateCamera()
{
  auto ogreScene = this->scene->OgreSceneManager();
  if (ogreScene == nullptr)
  {
    ignerr << "Scene manager cannot be obtained" << std::endl;
    return;
  }

  this->dataPtr->ogreCamera = ogreScene->createCamera(this->Name());
  if (this->dataPtr->ogreCamera == nullptr)
  {
    ignerr << "Ogre camera cannot be created" << std::endl;
    return;
  }

  this->dataPtr->ogreCamera->detachFromParent();
  this->ogreNode->attachObject(this->dataPtr->ogreCamera);

  // rotate to ignition gazebo coord.
  this->dataPtr->ogreCamera->yaw(Ogre::Degree(-90));
  this->dataPtr->ogreCamera->roll(Ogre::Degree(-90));
  this->dataPtr->ogreCamera->setFixedYawAxis(false);

  this->dataPtr->ogreCamera->setAutoAspectRatio(true);
  this->dataPtr->ogreCamera->setRenderingDistance(100);
  this->dataPtr->ogreCamera->setProjectionType(
      Ogre::ProjectionType::PT_PERSPECTIVE);
  this->dataPtr->ogreCamera->setCustomProjectionMatrix(false);
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::Destroy()
{
  if (this->dataPtr->buffer)
  {
    delete [] this->dataPtr->buffer;
    this->dataPtr->buffer = nullptr;
  }

  if (!this->dataPtr->ogreCamera)
    return;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr =
      ogreRoot->getCompositorManager2();

  // remove render texture, material, compositor
  if (this->dataPtr->ogreRenderTexture)
  {
    ogreRoot->getRenderSystem()->getTextureGpuManager()->destroyTexture(
          this->dataPtr->ogreRenderTexture);
    this->dataPtr->ogreRenderTexture = nullptr;
  }

  if (this->dataPtr->ogreCompositorWorkspace)
  {
    ogreCompMgr->removeWorkspace(
        this->dataPtr->ogreCompositorWorkspace);
  }

  if (!this->dataPtr->workspaceDefinition.empty())
  {
    ogreCompMgr->removeWorkspaceDefinition(
        this->dataPtr->workspaceDefinition);
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
      ogreSceneManager->destroyCamera(this->dataPtr->ogreCamera);
      this->dataPtr->ogreCamera = nullptr;
    }
  }

  if (this->dataPtr->materialSwitcher)
    this->dataPtr->materialSwitcher.reset();
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::PreRender()
{
  if (!this->dataPtr->ogreRenderTexture)
    this->CreateBoundingBoxTexture();

  this->dataPtr->outputBoxes.clear();
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::CreateBoundingBoxTexture()
{
  // Camera Parameters
  this->dataPtr->ogreCamera->setNearClipDistance(this->NearClipPlane());
  this->dataPtr->ogreCamera->setFarClipDistance(this->FarClipPlane());
  this->dataPtr->ogreCamera->setAspectRatio(this->AspectRatio());
  double vfov = 2.0 * atan(tan(this->HFOV().Radian() / 2.0) /
    this->AspectRatio());
  this->dataPtr->ogreCamera->setFOVy(Ogre::Radian(vfov));

  // render texture
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::TextureGpuManager *textureMgr =
      ogreRoot->getRenderSystem()->getTextureGpuManager();
  this->dataPtr->ogreRenderTexture =
      textureMgr->createOrRetrieveTexture(this->Name() + "_boundingbox_cam",
      Ogre::GpuPageOutStrategy::SaveToSystemRam,
      Ogre::TextureFlags::RenderToTexture,
      Ogre::TextureTypes::Type2D);

  if (!this->dataPtr->ogreRenderTexture)
  {
    ignerr << "Null render texture" << std::endl;
    return;
  }

  this->dataPtr->ogreRenderTexture->setResolution(
      this->ImageWidth(), this->ImageHeight());
  this->dataPtr->ogreRenderTexture->setNumMipmaps(1u);
  this->dataPtr->ogreRenderTexture->setPixelFormat(this->dataPtr->format);

  this->dataPtr->ogreRenderTexture->scheduleTransitionTo(
      Ogre::GpuResidency::Resident);

  // Switch material to OGRE Ids map to use it to get the visible bboxes
  // or to check visiblity in full bboxes
  this->dataPtr->ogreCamera->addListener(this->dataPtr->materialSwitcher.get());

  // workspace
  auto ogreCompositorManager = ogreRoot->getCompositorManager2();
  if (ogreCompositorManager == nullptr)
  {
    ignerr << "Null Ogre compositor manager." << std::endl;
    return;
  }

  this->dataPtr->workspaceDefinition = "BoundingBoxCameraWorkspace_" +
    this->Name();

  float background = static_cast<float>(
      this->dataPtr->materialSwitcher->backgroundLabel);
  auto backgroundColor = Ogre::ColourValue(background, background, background);

  // basic workspace consist of clear pass with the givin color &
  // a render scene pass to the givin render texture
  ogreCompositorManager->createBasicWorkspaceDef(
    this->dataPtr->workspaceDefinition,
    backgroundColor
    );

  // connect the compositor with the render texture to render the final output
  this->dataPtr->ogreCompositorWorkspace =
    ogreCompositorManager->addWorkspace(
      this->scene->OgreSceneManager(),
      this->dataPtr->ogreRenderTexture,
      this->dataPtr->ogreCamera,
      this->dataPtr->workspaceDefinition,
      false
    );
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::Render()
{
  if (!this->scene)
  {
    ignerr << "Null scene." << std::endl;
    return;
  }
  if (!this->dataPtr->ogreCompositorWorkspace)
  {
    ignerr << "Null Ogre compositor workspace." << std::endl;
    return;
  }

  // update the compositors
  this->scene->StartRendering(nullptr);

  this->dataPtr->ogreCompositorWorkspace->_validateFinalTarget();
  this->dataPtr->ogreCompositorWorkspace->_beginUpdate(false);
  this->dataPtr->ogreCompositorWorkspace->_update();
  this->dataPtr->ogreCompositorWorkspace->_endUpdate(false);

  Ogre::vector<Ogre::TextureGpu *>::type swappedTargets;
  swappedTargets.reserve(2u);
  this->dataPtr->ogreCompositorWorkspace->_swapFinalTarget(swappedTargets);

  this->scene->FlushGpuCommandsAndStartNewFrame(1u, false);
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::PostRender()
{
  // return if no one is listening to the new frame
  if (this->dataPtr->newBoundingBoxes.ConnectionCount() == 0)
    return;

  if (!this->dataPtr->ogreRenderTexture)
  {
    ignerr << "Null render texture" << std::endl;
    return;
  }

  unsigned int width = this->ImageWidth();
  unsigned int height = this->ImageHeight();

  PixelFormat format = PF_R8G8B8;
  unsigned int channelCount = PixelUtil::ChannelCount(format);
  unsigned int bytesPerChannel = PixelUtil::BytesPerChannel(format);
  // raw gpu texture format is RGBA8
  unsigned int rawChannelCount = 4u;

  Ogre::Image2 image;
  image.convertFromTexture(this->dataPtr->ogreRenderTexture, 0u, 0u);
  Ogre::TextureBox box = image.getData(0);
  uint8_t *imgBufferTmp = static_cast<uint8_t *>(box.data);
  if (!this->dataPtr->buffer)
  {
    auto bufferSize = PixelUtil::MemorySize(format, width, height);
    this->dataPtr->buffer = new uint8_t[bufferSize];
  }

  for (unsigned int row = 0; row < height; ++row)
  {
    // the texture box step size could be larger than our image buffer step
    // size
    unsigned int rawDataRowIdx = row * box.bytesPerRow / bytesPerChannel;
    for (unsigned int column = 0; column < width; ++column)
    {
      unsigned int idx = (row * width * channelCount) +
          column * channelCount;
      unsigned int rawIdx = rawDataRowIdx +
          column * rawChannelCount;

      this->dataPtr->buffer[idx] = imgBufferTmp[rawIdx];
      this->dataPtr->buffer[idx + 1] = imgBufferTmp[rawIdx + 1];
      this->dataPtr->buffer[idx + 2] = imgBufferTmp[rawIdx + 2];
    }
  }

  if (this->dataPtr->type == BoundingBoxType::BBT_VISIBLEBOX2D)
    this->VisibleBoundingBoxes();
  else if (this->dataPtr->type == BoundingBoxType::BBT_FULLBOX2D)
    this->FullBoundingBoxes();
  else if (this->dataPtr->type == BoundingBoxType::BBT_BOX3D)
    this->BoundingBoxes3D();

  this->dataPtr->boundingboxes.clear();
  this->dataPtr->visibleBoxesLabel.clear();
  this->dataPtr->parentNameToBoxes.clear();
  this->dataPtr->parentNameToOgreIds.clear();
  this->dataPtr->itemVertices.clear();
  this->dataPtr->ogreIdToItem.clear();
  this->dataPtr->materialSwitcher->ogreIdName.clear();

  this->dataPtr->newBoundingBoxes(this->dataPtr->outputBoxes);
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::MarkVisibleBoxes()
{
  if (!this->dataPtr->buffer)
  {
    ignerr << "Null buffer" << std::endl;
    return;
  }

  uint32_t width = this->ImageWidth();
  uint32_t height = this->ImageHeight();
  uint32_t channelCount = 3;

  // Filter bounding boxes by looping over all pixels in ogre ids map
  for (uint32_t y = 0; y < height; ++y)
  {
    for (uint32_t x = 0; x < width; ++x)
    {
      auto index = (y * width + x) * channelCount;

      uint32_t label = this->dataPtr->buffer[index + 2];

      if (label == this->dataPtr->materialSwitcher->backgroundLabel)
        continue;

      // get the ogre id encoded in 16 bit value
      uint32_t ogreId1 = this->dataPtr->buffer[index + 1];
      uint32_t ogreId2 = this->dataPtr->buffer[index + 0];
      uint32_t ogreId = ogreId1 * 256 + ogreId2;

      // mark the ogreId as visible not to filter its bbox
      if (!this->dataPtr->visibleBoxesLabel.count(ogreId))
        this->dataPtr->visibleBoxesLabel[ogreId] = label;
    }
  }
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCameraPrivate::MeshVertices(
    const std::vector<uint32_t> &_ogreIds,
    std::vector<math::Vector3d> &_vertices)
{
  auto viewMatrix = this->ogreCamera->getViewMatrix();

  for (auto ogreId : _ogreIds)
  {
    Ogre::Item *item = this->ogreIdToItem[ogreId];
    Ogre::MeshPtr mesh = item->getMesh();
    Ogre::Node *node = item->getParentNode();

    Ogre::Vector3 position = node->_getDerivedPosition();
    Ogre::Quaternion oreintation = node->_getDerivedOrientation();
    Ogre::Vector3 scale = node->_getDerivedScale();

    auto subMeshes = mesh->getSubMeshes();

    for (const auto &subMesh : subMeshes)
    {
      Ogre::VertexArrayObjectArray vaos = subMesh->mVao[0];

      if (!vaos.empty())
      {
        // Get the first LOD level
        Ogre::VertexArrayObject *vao = vaos[0];

        // request async read from buffer
        Ogre::VertexArrayObject::ReadRequestsArray requests;
        requests.push_back(Ogre::VertexArrayObject::ReadRequests(
          Ogre::VES_POSITION));
        vao->readRequests(requests);
        vao->mapAsyncTickets(requests);

        unsigned int subMeshVerticiesNum =
          requests[0].vertexBuffer->getNumElements();
        for (size_t i = 0; i < subMeshVerticiesNum; ++i)
        {
          Ogre::Vector3 vec;
          if (requests[0].type == Ogre::VET_HALF4)
          {
            const Ogre::uint16* vertex = reinterpret_cast<const Ogre::uint16*>
              (requests[0].data);
            vec.x = Ogre::Bitwise::halfToFloat(vertex[0]);
            vec.y = Ogre::Bitwise::halfToFloat(vertex[1]);
            vec.z = Ogre::Bitwise::halfToFloat(vertex[2]);
          }
          else if (requests[0].type == Ogre::VET_FLOAT3)
          {
            const float* vertex =
              reinterpret_cast<const float*>(requests[0].data);
            vec.x = *vertex++;
            vec.y = *vertex++;
            vec.z = *vertex++;
          }
          else
            ignerr << "Vertex Buffer type error" << std::endl;

          // Convert to world coordinates
          vec = (oreintation * (vec * scale)) + position;

          // Convert to camera view coordiantes
          Ogre::Vector4 vec4(vec.x, vec.y, vec.z, 1);
          vec4 = viewMatrix * vec4;

          vec.x = vec4.x;
          vec.y = vec4.y;
          vec.z = vec4.z;

          // Add the vertex to the vertices of all items that
          // belongs to the same parent
          _vertices.push_back(Ogre2Conversions::Convert(vec));

          // get the next element
          requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
        }
        vao->unmapAsyncTickets(requests);
      }
    }
  }
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::MergeMultiLinksModels3D()
{
  // Combine the boxes with the same parent name together to merge them
  for (const auto &box : this->dataPtr->boundingboxes)
  {
    auto ogreId = box.first;
    auto parentName = this->dataPtr->materialSwitcher->ogreIdName[ogreId];
    this->dataPtr->parentNameToOgreIds[parentName].push_back(ogreId);
  }

  // Merge the boxes that is related to the same parent
  for (const auto &nameToOgreIds : this->dataPtr->parentNameToOgreIds)
  {
    auto ogreIds = nameToOgreIds.second;

    // If not a multi-link model, add the 3d box from the OGRE API
    if (ogreIds.size() == 1)
    {
      auto box = this->dataPtr->boundingboxes[ogreIds[0]];
      this->dataPtr->outputBoxes.push_back(*box);
    }
    else
    {
      std::vector<math::Vector3d> vertices;

      // Get all the 3D vertices of the sub-items(total mesh)
      this->dataPtr->MeshVertices(ogreIds, vertices);

      // Get the oriented bounding box from the mesh using PCA
      math::OrientedBoxd mergedBox = math::eigen3::verticesToOrientedBox(
        vertices);

      // convert to rendering::BoundingBox format
      BoundingBox box;
      auto pose = mergedBox.Pose();
      box.SetCenter(pose.Pos());
      box.SetOrientation(pose.Rot());
      box.SetSize(mergedBox.Size());
      box.SetLabel(this->dataPtr->visibleBoxesLabel[ogreIds[0]]);

      this->dataPtr->outputBoxes.push_back(box);
    }
  }

  // reverse the order of the boxes (useful in testing)
  std::reverse(this->dataPtr->outputBoxes.begin(),
    this->dataPtr->outputBoxes.end());
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::MergeMultiLinksModels2D()
{
  // Combine the boxes with the same parent name together to merge them
  for (const auto &box : this->dataPtr->boundingboxes)
  {
    auto ogreId = box.first;
    auto parentName = this->dataPtr->materialSwitcher->ogreIdName[ogreId];
    this->dataPtr->parentNameToBoxes[parentName].push_back(box.second);
  }

  // Merge the boxes that is related to the same parent
  for (const auto &nameToBoxes : this->dataPtr->parentNameToBoxes)
  {
    auto mergedBox = this->dataPtr->MergeBoxes2D(nameToBoxes.second);

    // Store boxes in the output vector
    this->dataPtr->outputBoxes.push_back(mergedBox);
  }

  // reverse the order of the boxes (usful in testing)
  std::reverse(this->dataPtr->outputBoxes.begin(),
    this->dataPtr->outputBoxes.end());
}

/////////////////////////////////////////////////
BoundingBox Ogre2BoundingBoxCameraPrivate::MergeBoxes2D(
        const std::vector<std::shared_ptr<BoundingBox>> &_boxes)
{
  if (_boxes.size() == 1)
    return *_boxes[0];

  BoundingBox mergedBox;
  double minX = std::numeric_limits<double>::max();
  double maxX = 0.0;
  double minY = std::numeric_limits<double>::max();
  double maxY = 0.0;

  for (const auto &box : _boxes)
  {
    double boxMinX = box->Center().X() - box->Size().X() * 0.5;
    double boxMaxX = box->Center().X() + box->Size().X() * 0.5;
    double boxMinY = box->Center().Y() - box->Size().Y() * 0.5;
    double boxMaxY = box->Center().Y() + box->Size().Y() * 0.5;

    minX = std::min(minX, boxMinX);
    maxX = std::max(maxX, boxMaxX);
    minY = std::min(minY, boxMinY);
    maxY = std::max(maxY, boxMaxY);
  }

  auto width = maxX - minX;
  auto height = maxY - minY;
  mergedBox.SetSize({width, height, 0});
  mergedBox.SetCenter({minX + width * 0.5, minY + height * 0.5, 0});
  mergedBox.SetLabel(_boxes[0]->Label());

  return mergedBox;
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::BoundingBoxes3D()
{
  auto viewMatrix = this->dataPtr->ogreCamera->getViewMatrix();

  // used to filter the hidden boxes
  this->MarkVisibleBoxes();

  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);
    Ogre::MeshPtr mesh = item->getMesh();

    uint32_t ogreId = item->getId();

    // Skip the items which is hidden from the ogreId map
    if (!this->dataPtr->visibleBoxesLabel.count(ogreId))
    {
      itor.moveNext();
      continue;
    }

    // get attached node
    Ogre::Node *node = item->getParentNode();

    Ogre::Quaternion orientation = node->_getDerivedOrientation();
    Ogre::Vector3 scale = node->_getDerivedScale();
    Ogre::Vector3 size = item->getLocalAabb().getSize();
    size *= scale;

    Ogre::Aabb aabb = item->getWorldAabb();

    // filter the boxes outside the camera frustum
    Ogre::AxisAlignedBox worldAabb;
    worldAabb.setExtents(aabb.getMinimum(), aabb.getMaximum());
    if (!this->dataPtr->ogreCamera->isVisible(worldAabb))
    {
      itor.moveNext();
      continue;
    }

    // Keep track of mesh, useful in multi-links models
    this->dataPtr->ogreIdToItem[ogreId] = item;

    auto box = std::make_shared<BoundingBox>();

    // Position in world coord
    Ogre::Vector3 position = worldAabb.getCenter();

    // Position in camera coord
    Ogre::Vector3 viewPosition = viewMatrix * position;

    // Convert to ignition::math
    box->SetCenter(Ogre2Conversions::Convert(viewPosition));
    box->SetSize(Ogre2Conversions::Convert(size));

    // Compute the rotation of the box from its world rotation & view matrix
    auto worldCameraRotation = Ogre2Conversions::Convert(
      viewMatrix.extractQuaternion());
    auto bodyWorldRotation = Ogre2Conversions::Convert(orientation);

    // Body to camera rotation = body_world * world_camera
    auto bodyCameraRotation = worldCameraRotation * bodyWorldRotation;
    box->SetOrientation(bodyCameraRotation);

    this->dataPtr->boundingboxes[ogreId] = box;
    itor.moveNext();
  }

  // Set boxes labels
  for (const auto &box : this->dataPtr->boundingboxes)
  {
    uint32_t ogreId = box.first;
    uint32_t label = this->dataPtr->visibleBoxesLabel[ogreId];

    box.second->SetLabel(label);
  }

  // Combine boxes of multi-links model if exists
  this->MergeMultiLinksModels3D();
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::VisibleBoundingBoxes()
{
  if (!this->dataPtr->buffer)
  {
    ignerr << "Null buffer" << std::endl;
    return;
  }

  uint32_t width = this->ImageWidth();
  uint32_t height = this->ImageHeight();
  uint32_t channelCount = 3;

  struct BoxBoundary
  {
    uint32_t minX;
    uint32_t minY;
    uint32_t maxX;
    uint32_t maxY;
  };

  std::unordered_map<uint32_t, std::shared_ptr<BoxBoundary>> boxesBoundary;

  // find item's boundaries from panoptic BoundingBox
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      auto index = (y * width + x) * channelCount;

      uint32_t label = this->dataPtr->buffer[index + 2];
      uint32_t ogreId1 = this->dataPtr->buffer[index + 1];
      uint32_t ogreId2 = this->dataPtr->buffer[index + 0];

      if (label != this->dataPtr->materialSwitcher->backgroundLabel)
      {
        // get the OGRE id of 16 bit value
        uint32_t ogreId = ogreId1 * 256 + ogreId2;

        std::shared_ptr<BoundingBox> box;
        std::shared_ptr<BoxBoundary> boundary;

        // create new boxes when its first pixel appears
        if (!this->dataPtr->boundingboxes.count(ogreId))
        {
          box = std::make_shared<BoundingBox>();
          box->SetLabel(label);

          boundary = std::make_shared<BoxBoundary>();
          boundary->minX = width;
          boundary->minY = height;
          boundary->maxX = 0;
          boundary->maxY = 0;

          boxesBoundary[ogreId] = boundary;
          this->dataPtr->boundingboxes[ogreId] = box;
        }
        else
        {
          boundary = boxesBoundary[ogreId];
        }

        boundary->minX = std::min<uint32_t>(boundary->minX, x);
        boundary->minY = std::min<uint32_t>(boundary->minY, y);
        boundary->maxX = std::max<uint32_t>(boundary->maxX, x);
        boundary->maxY = std::max<uint32_t>(boundary->maxY, y);
      }
    }
  }

  for (const auto &box : this->dataPtr->boundingboxes)
  {
    // Get the box's boundary
    auto ogreId = box.first;
    auto boundary = boxesBoundary[ogreId];
    auto boxWidth = boundary->maxX - boundary->minX;
    auto boxHeight = boundary->maxY - boundary->minY;

    box.second->SetCenter({boundary->minX + boxWidth * 0.5,
        boundary->minY + boxHeight * 0.5, 0});
    box.second->SetSize(
        {static_cast<double>(boxWidth), static_cast<double>(boxHeight), 0.0});
  }

  // Combine boxes of multi-links model if exists
  this->MergeMultiLinksModels2D();
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::FullBoundingBoxes()
{
  // used to filter the hidden boxes
  this->MarkVisibleBoxes();

  Ogre::Matrix4 viewMatrix = this->dataPtr->ogreCamera->getViewMatrix();
  Ogre::Matrix4 projMatrix = this->dataPtr->ogreCamera->getProjectionMatrix();

  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ItemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::Item *item = static_cast<Ogre::Item *>(object);
    Ogre::MeshPtr mesh = item->getMesh();

    uint32_t ogreId = item->getId();

    // Skip the items which is hidden in the ogreId map
    if (!this->dataPtr->visibleBoxesLabel.count(ogreId))
    {
      itor.moveNext();
      continue;
    }

    // get attached node
    Ogre::Node *node = item->getParentNode();

    // get the derived position
    Ogre::Vector3 position = node->_getDerivedPosition();
    Ogre::Quaternion oreintation = node->_getDerivedOrientation();
    Ogre::Vector3 scale = node->_getDerivedScale();

    Ogre::Aabb aabb = item->getWorldAabb();
    Ogre::AxisAlignedBox worldAabb;
    worldAabb.setExtents(aabb.getMinimum(), aabb.getMaximum());

    // filter the boxes outside the camera frustum
    if (!this->dataPtr->ogreCamera->isVisible(worldAabb))
    {
      itor.moveNext();
      continue;
    }

    Ogre::Vector3 minVertex;
    Ogre::Vector3 maxVertex;

    this->MeshMinimalBox(
      mesh,
      viewMatrix,
      projMatrix,
      minVertex,
      maxVertex,
      position,
      oreintation,
      scale
    );

    if ((abs(minVertex.x) > 1 && abs(maxVertex.x) > 1) ||
        (abs(minVertex.y) > 1 && abs(maxVertex.y) > 1))
    {
      itor.moveNext();
      continue;
    }

    this->ConvertToScreenCoord(minVertex, maxVertex);

    auto box = std::make_shared<BoundingBox>();
    auto boxWidth = maxVertex.x - minVertex.x;
    auto boxHeight = minVertex.y - maxVertex.y;
    box->SetCenter(
        {minVertex.x + boxWidth / 2, maxVertex.y + boxHeight / 2, 0});
    box->SetSize({boxWidth, boxHeight, 0});

    this->dataPtr->boundingboxes[ogreId] = box;

    itor.moveNext();
  }

  // Set boxes label
  for (auto box : this->dataPtr->boundingboxes)
  {
    uint32_t ogreId = box.first;
    uint32_t label = this->dataPtr->visibleBoxesLabel[ogreId];

    box.second->SetLabel(label);
  }

  // Combine boxes of multi-links model if exists
  this->MergeMultiLinksModels2D();
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::MeshMinimalBox(
  const Ogre::MeshPtr _mesh,
  const Ogre::Matrix4 &_viewMatrix,
  const Ogre::Matrix4 &_projMatrix,
  Ogre::Vector3 &_minVertex,
  Ogre::Vector3 &_maxVertex,
  const Ogre::Vector3 &_position,
  const Ogre::Quaternion &_orientation,
  const Ogre::Vector3 &_scale
  )
{
  _minVertex.x = std::numeric_limits<float>::max();
  _minVertex.y = std::numeric_limits<float>::max();
  _minVertex.z = std::numeric_limits<float>::max();
  _maxVertex.x = -std::numeric_limits<float>::max();
  _maxVertex.y = -std::numeric_limits<float>::max();
  _maxVertex.z = -std::numeric_limits<float>::max();

  auto subMeshes = _mesh->getSubMeshes();

  for (auto subMesh : subMeshes)
  {
    Ogre::VertexArrayObjectArray vaos = subMesh->mVao[0];

    if (!vaos.empty())
    {
      // Get the first LOD level
      Ogre::VertexArrayObject *vao = vaos[0];

      // request async read from buffer
      Ogre::VertexArrayObject::ReadRequestsArray requests;
      requests.push_back(Ogre::VertexArrayObject::ReadRequests(
        Ogre::VES_POSITION));
      vao->readRequests(requests);
      vao->mapAsyncTickets(requests);

      unsigned int subMeshVerticiesNum =
        requests[0].vertexBuffer->getNumElements();
      for (size_t i = 0; i < subMeshVerticiesNum; ++i)
      {
        Ogre::Vector3 vec;
        if (requests[0].type == Ogre::VET_HALF4)
        {
          const Ogre::uint16* vertex = reinterpret_cast<const Ogre::uint16*>
            (requests[0].data);
          vec.x = Ogre::Bitwise::halfToFloat(vertex[0]);
          vec.y = Ogre::Bitwise::halfToFloat(vertex[1]);
          vec.z = Ogre::Bitwise::halfToFloat(vertex[2]);
        }
        else if (requests[0].type == Ogre::VET_FLOAT3)
        {
          const float* vertex =
            reinterpret_cast<const float*>(requests[0].data);
          vec.x = *vertex++;
          vec.y = *vertex++;
          vec.z = *vertex++;
        }
        else
          ignerr << "Vertex Buffer type error" << std::endl;

        vec = (_orientation * (vec * _scale)) + _position;

        Ogre::Vector4 vec4(vec.x, vec.y, vec.z, 1);
        vec4 =  _projMatrix * _viewMatrix * vec4;

        // homogenous
        vec.x = vec4.x / vec4.w;
        vec.y = vec4.y / vec4.w;
        vec.z = vec4.z;

        _minVertex.x = std::min(_minVertex.x, vec.x);
        _minVertex.y = std::min(_minVertex.y, vec.y);
        _minVertex.z = std::min(_minVertex.z, vec.z);

        _maxVertex.x = std::max(_maxVertex.x, vec.x);
        _maxVertex.y = std::max(_maxVertex.y, vec.y);
        _maxVertex.z = std::max(_maxVertex.z, vec.z);

        // get the next element
        requests[0].data += requests[0].vertexBuffer->getBytesPerElement();
      }
      vao->unmapAsyncTickets(requests);
    }
  }
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::DrawLine(unsigned char *_data,
  const math::Vector2i &_point1, const math::Vector2i &_point2,
  const math::Color &_color) const
{
  int x0, y0, x1, y1;

  // Check if the line is close to a vertical or horizontal line
  if (std::abs(_point2.Y() - _point1.Y())
    < std::abs(_point2.X() - _point1.X()))
  {
    if (_point1.X() < _point2.X())
    {
      x0 = _point1.X();
      y0 = _point1.Y();
      x1 = _point2.X();
      y1 = _point2.Y();
    }
    else
    {
      x0 = _point2.X();
      y0 = _point2.Y();
      x1 = _point1.X();
      y1 = _point1.Y();
    }
    auto dx = x1 - x0;
    auto dy = y1 - y0;
    auto yi = 1;
    if (dy < 0)
    {
      yi = -1;
      dy = -dy;
    }
    auto D = 2 * dy - dx;
    auto y = y0;

    for (int x = x0; x < x1; x++)
    {
      // Plot the point
      auto index = (y * this->ImageWidth() + x) * 3;
      _data[index] = static_cast<unsigned char>(255 * _color.R());
      _data[index + 1] = static_cast<unsigned char>(255 * _color.G());
      _data[index + 2] = static_cast<unsigned char>(255 * _color.B());

      if (D > 0)
      {
        y += yi;
        D += (2 * (dy - dx));
      }
      else
      {
        D += (2 * dy);
      }
    }
  }
  else
  {
    if (_point1.Y() < _point2.Y())
    {
      x0 = _point1.X();
      y0 = _point1.Y();
      x1 = _point2.X();
      y1 = _point2.Y();
    }
    else
    {
      x0 = _point2.X();
      y0 = _point2.Y();
      x1 = _point1.X();
      y1 = _point1.Y();
    }
    auto dx = x1 - x0;
    auto dy = y1 - y0;
    auto xi = 1;
    if (dx < 0)
    {
      xi = -1;
      dx = -dx;
    }
    auto D = 2 * dx - dy;
    auto x = x0;

    for (int y = y0; y < y1; y++)
    {
      // Plot the point
      auto index = (y * this->ImageWidth() + x) * 3;
      _data[index] = static_cast<unsigned char>(255 * _color.R());
      _data[index + 1] = static_cast<unsigned char>(255 * _color.G());
      _data[index + 2] = static_cast<unsigned char>(255 * _color.B());

      if (D > 0)
      {
        x += xi;
        D += (2 * (dx - dy));
      }
      else
      {
        D += (2 * dx);
      }
    }
  }
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::DrawBoundingBox(unsigned char *_data,
    const math::Color &_color, const BoundingBox &_box) const
{
  // 3D box
  if (this->Type() == BoundingBoxType::BBT_BOX3D)
  {
    // Get the 3D vertices of the box in 3D camera coord.
    auto vertices = _box.Vertices3D();

    // Project the 3D vertices in 3D camera coord to 2D vertices in clip coord
    auto projMatrix = this->dataPtr->ogreCamera->getProjectionMatrix();
    std::vector<math::Vector2d> vertices2d;
    for (auto &vertex : vertices)
    {
      // Skip boxes which has any vertex behind the camera (has positive z)
      if (vertex.Z() > 0)
        return;

      // Convert to homogeneous coord.
      auto homoVertex =
        Ogre::Vector4(vertex.X(), vertex.Y(), vertex.Z(), 1);

      auto projVertex = projMatrix * homoVertex;
      projVertex.x /= projVertex.w;
      projVertex.y /= projVertex.w;

      vertices2d.push_back({projVertex.x, projVertex.y});
    }

    // clip the values outside the frustum range [-1, 1]
    /*

        1 -------- 0
       /|         /|
      2 -------- 3 .
      | |        | |
      . 5 -------- 4
      |/         |/
      6 -------- 7

    */

    std::vector<math::Vector2d> clippedVertices;
    const math::Vector4d box(-1.0, -1.0, 1.0, 1.0);
    // top
    this->dataPtr->AddToViewportLines(box, vertices2d[0], vertices2d[1],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[1], vertices2d[2],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[2], vertices2d[3],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[3], vertices2d[0],
        clippedVertices);
    // bottom
    this->dataPtr->AddToViewportLines(box, vertices2d[4], vertices2d[5],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[5], vertices2d[6],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[6], vertices2d[7],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[7], vertices2d[4],
        clippedVertices);
    // pillars
    this->dataPtr->AddToViewportLines(box, vertices2d[0], vertices2d[4],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[1], vertices2d[5],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[2], vertices2d[6],
        clippedVertices);
    this->dataPtr->AddToViewportLines(box, vertices2d[3], vertices2d[7],
        clippedVertices);

    // Convert To screen coord.
    uint32_t width = this->ImageWidth();
    uint32_t height = this->ImageHeight();

    std::vector<math::Vector2i> projVertices;
    for (auto &vertex : clippedVertices)
    {
      // convert from [-1, 1] range to [0, 1] range & to the screen range
      vertex.X() = uint32_t((vertex.X() + 1.0) / 2 * width );
      vertex.Y() = uint32_t((1.0 - vertex.Y()) / 2 * height);

      vertex.X() = std::max<double>(0.0, vertex.X());
      vertex.Y() = std::max<double>(0.0, vertex.Y());
      vertex.X() = std::min<double>(vertex.X(), width - 1.0);
      vertex.Y() = std::min<double>(vertex.Y(), height - 1.0);

      projVertices.push_back(math::Vector2i(static_cast<int>(vertex.X()),
          static_cast<int>(vertex.Y())));
    }

    for (unsigned int endPt = 0; endPt < projVertices.size(); endPt += 2)
    {
      this->DrawLine(_data, projVertices[endPt], projVertices[endPt + 1],
          _color);
    }

    return;
  }

  // 2D box
  math::Vector2i minVertex(
    static_cast<int>(_box.Center().X() - _box.Size().X() / 2),
    static_cast<int>(_box.Center().Y() - _box.Size().Y() / 2));

  math::Vector2i maxVertex(
    static_cast<int>(_box.Center().X() + _box.Size().X() / 2),
    static_cast<int>(_box.Center().Y() + _box.Size().Y() / 2));

  uint32_t width = this->ImageWidth();

  std::vector<uint32_t> xValues =
    {uint32_t(minVertex.X()),
        std::min(this->ImageWidth() - 1, uint32_t(maxVertex.X()))};
  std::vector<uint32_t> yValues =
    {uint32_t(minVertex.Y()),
        std::min(this->ImageHeight() - 1, uint32_t(maxVertex.Y()))};

  for (auto i = minVertex.Y(); i < maxVertex.Y(); i++)
  {
    for (auto j : xValues)
    {
      auto index = (i * width + j) * 3;
      _data[index] = static_cast<unsigned char>(255 * _color.R());
      _data[index + 1] = static_cast<unsigned char>(255 * _color.G());
      _data[index + 2] = static_cast<unsigned char>(255 * _color.B());
    }
  }
  for (auto i : yValues)
  {
    for (auto j = minVertex.X(); j < maxVertex.X(); j++)
    {
      auto index = (i * width + j) * 3;
      _data[index] = static_cast<unsigned char>(255 * _color.R());
      _data[index + 1] = static_cast<unsigned char>(255 * _color.G());
      _data[index + 2] = static_cast<unsigned char>(255 * _color.B());
    }
  }
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::ConvertToScreenCoord(
  Ogre::Vector3 &_minVertex, Ogre::Vector3 &_maxVertex) const
{
  uint32_t width = this->ImageWidth();
  uint32_t height = this->ImageHeight();

  // clip the values outside the frustum range
  _minVertex.x = std::clamp<float>(_minVertex.x, -1.0, 1.0);
  _minVertex.y = std::clamp<float>(_minVertex.y, -1.0, 1.0);
  _maxVertex.x = std::clamp<float>(_maxVertex.x, -1.0, 1.0);
  _maxVertex.y = std::clamp<float>(_maxVertex.y, -1.0, 1.0);

  // convert from [-1, 1] range to [0, 1] range & multiply by screen dims
  _minVertex.x = (_minVertex.x + 1.0) / 2 * width;
  _minVertex.y = (1.0 - _minVertex.y) / 2 * height;
  _maxVertex.x = (_maxVertex.x + 1.0) / 2 * width;
  _maxVertex.y = (1.0 - _maxVertex.y) / 2 * height;

  // clip outside screen boundries
  _minVertex.x = std::max<float>(0.0, _minVertex.x);
  _minVertex.y = std::max<float>(0.0, _minVertex.y);
  _maxVertex.x = std::min<float>(_maxVertex.x, width - 1.0);
  _maxVertex.y = std::min<float>(_maxVertex.y, height - 1.0);
}

/////////////////////////////////////////////////
const std::vector<BoundingBox> &Ogre2BoundingBoxCamera::BoundingBoxData() const
{
  return this->dataPtr->outputBoxes;
}

/////////////////////////////////////////////////
common::ConnectionPtr
  Ogre2BoundingBoxCamera::ConnectNewBoundingBoxes(
  std::function<void(const std::vector<BoundingBox> &)>  _subscriber)
{
  return this->dataPtr->newBoundingBoxes.Connect(_subscriber);
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::CreateRenderTexture()
{
  RenderTexturePtr base = this->scene->CreateRenderTexture();
  this->dataPtr->dummyTexture =
    std::dynamic_pointer_cast<Ogre2RenderTexture>(base);
  this->dataPtr->dummyTexture->SetWidth(1);
  this->dataPtr->dummyTexture->SetHeight(1);
}

/////////////////////////////////////////////////
RenderTargetPtr Ogre2BoundingBoxCamera::RenderTarget() const
{
  return this->dataPtr->dummyTexture;
}

/////////////////////////////////////////////////
void Ogre2BoundingBoxCamera::SetBoundingBoxType(BoundingBoxType _type)
{
  this->dataPtr->type = _type;
}

/////////////////////////////////////////////////
BoundingBoxType Ogre2BoundingBoxCamera::Type() const
{
  return this->dataPtr->type;
}
