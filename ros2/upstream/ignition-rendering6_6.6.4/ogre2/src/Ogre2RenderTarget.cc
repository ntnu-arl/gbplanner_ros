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

#include <gz/common/Console.hh>

#include "gz/rendering/Material.hh"

#include "gz/rendering/ogre2/Ogre2Includes.hh"
#include "gz/rendering/ogre2/Ogre2RenderEngine.hh"
#include "gz/rendering/ogre2/Ogre2RenderPass.hh"
#include "gz/rendering/ogre2/Ogre2Conversions.hh"
#include "gz/rendering/ogre2/Ogre2Material.hh"
#include "gz/rendering/ogre2/Ogre2RenderTarget.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
//
/// \brief Listener for changing ogre compositor pass properties
class Ogre2RenderTargetCompositorListener :
    public Ogre::CompositorWorkspaceListener
{
  /// \brief Constructor
  /// \param[in] _target ogre render target object
  public: explicit Ogre2RenderTargetCompositorListener(
      Ogre2RenderTarget *_target)
  {
    this->ogreRenderTarget = _target;
  }

  /// \brief Destructor
  public: virtual ~Ogre2RenderTargetCompositorListener() = default;

  // Documentation inherited.
  public: virtual void passPreExecute(Ogre::CompositorPass *_pass)
  {
    if (_pass->getType() == Ogre::PASS_SCENE)
    {
      Ogre::CompositorPassScene *scenePass =
          static_cast<Ogre::CompositorPassScene *>(_pass);
      IGN_ASSERT(scenePass != nullptr, "Unable to get scene pass");
      Ogre::Viewport *vp = scenePass->getCamera()->getLastViewport();
      if (vp == nullptr) return;
      // make sure we do not alter the reserved visibility flags
      uint32_t f = this->ogreRenderTarget->VisibilityMask() &
                   Ogre::VisibilityFlags::RESERVED_VISIBILITY_FLAGS;
      // apply the new visibility mask
      uint32_t flags = f & vp->getVisibilityMask();
      vp->_setVisibilityMask(flags, vp->getLightVisibilityMask());
    }
  }

  /// \brief Pointer to render target that added this listener
  private: Ogre2RenderTarget *ogreRenderTarget = nullptr;
};
}
}
}

/// \brief Private data class for Ogre2RenderTarget
class gz::rendering::Ogre2RenderTargetPrivate
{
  /// \brief Listener for chaning compositor pass properties
  public: Ogre2RenderTargetCompositorListener *rtListener = nullptr;

  /// \brief Name of sky box material
  public: const std::string kSkyboxMaterialName = "SkyBox";

  /// \brief Name of base rendering compositor node
  public: const std::string kBaseNodeName = "PbsMaterialsRenderingNode";

  /// \brief Name of final rendering compositor node
  public: const std::string kFinalNodeName = "FinalComposition";

  /// \brief Name of shadow compositor node
  public: const std::string kShadowNodeName = "PbsMaterialsShadowNode";

  /// \brief Pointer to the internal ogre render texture objects
  /// There's two because we ping pong postprocessing effects
  /// and the final result is always in ogreTexture[1]
  /// RenderWindows may have a 3rd texture which is the
  /// actual window
  ///
  public: Ogre::TextureGpu *ogreTexture[2] = {nullptr, nullptr};
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
// Ogre2RenderTarget
//////////////////////////////////////////////////
Ogre2RenderTarget::Ogre2RenderTarget()
  : dataPtr(new Ogre2RenderTargetPrivate)
{
  this->ogreBackgroundColor = Ogre::ColourValue::Black;
  this->ogreCompositorWorkspaceDefName = "PbsMaterialsWorkspace";
}

//////////////////////////////////////////////////
Ogre2RenderTarget::~Ogre2RenderTarget()
{
  IGN_ASSERT(this->dataPtr->rtListener == nullptr &&
               this->dataPtr->ogreTexture[0] == nullptr &&
               this->dataPtr->ogreTexture[1] == nullptr &&
               this->ogreCompositorWorkspace == nullptr,
             "Ogre2RenderTarget::Destroy not called!");
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::BuildCompositor()
{
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  this->UpdateBackgroundMaterial();

  bool validBackground = this->backgroundMaterial &&
      !this->backgroundMaterial->EnvironmentMap().empty();

  // The function build a similar compositor as the one defined in
  // ogre2/media/2.0/scripts/Compositors/PbsMaterials.compositor
  // but supports an extra quad pass to render the skybox cubemap if
  // sky is enabled.
  // todo(anyone) Note the definition programmatically created here
  // replaces the one defined in the script so it maybe safe to remove the
  // PbsMaterials.compositor file
  std::string wsDefName = "PbsMaterialWorkspace_" + this->Name();
  this->ogreCompositorWorkspaceDefName = wsDefName;
  if (!ogreCompMgr->hasWorkspaceDefinition(wsDefName))
  {
    // PbsMaterialsRenderingNode
    std::string nodeDefName = wsDefName + "/" + this->dataPtr->kBaseNodeName;
    Ogre::CompositorNodeDef *nodeDef =
        ogreCompMgr->addNodeDefinition(nodeDefName);

    nodeDef->addTextureSourceName(
          "rt0", 0u, Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    nodeDef->addTextureSourceName(
          "rt1", 1u, Ogre::TextureDefinitionBase::TEXTURE_INPUT);

    {
      // Add a manually-defined RTV (based on an automatically generated one)
      // so that we can perform an explicit MSAA resolve.
      const Ogre::RenderTargetViewDef *rt0Def =
          nodeDef->getRenderTargetViewDef( "rt0" );
      Ogre::RenderTargetViewDef *rtvDef =
          nodeDef->addRenderTextureView( "rtv" );

      *rtvDef = *rt0Def;

      const uint8_t fsaa = TargetFSAA();
      if (fsaa > 1u)
      {
        Ogre::TextureDefinitionBase::TextureDefinition *msaaDef =
            nodeDef->addTextureDefinition("rt_fsaa");

        msaaDef->fsaa = std::to_string(fsaa);

        rtvDef->colourAttachments[0].textureName = "rt_fsaa";
        rtvDef->colourAttachments[0].resolveTextureName = "rt0";
      }
    }

    nodeDef->setNumTargetPass(2);
    Ogre::CompositorTargetDef *rt0TargetDef =
        nodeDef->addTargetPass("rtv");

    if (validBackground)
      rt0TargetDef->setNumPasses(3);
    else
      rt0TargetDef->setNumPasses(2);
    {
      // scene pass - opaque
      {
        Ogre::CompositorPassSceneDef *passScene =
            static_cast<Ogre::CompositorPassSceneDef *>(
            rt0TargetDef->addPass(Ogre::PASS_SCENE));
        passScene->mShadowNode = this->dataPtr->kShadowNodeName;
        passScene->mIncludeOverlays = false;
        passScene->mFirstRQ = 0u;
        passScene->mLastRQ = 2u;
        if (validBackground)
        {
          passScene->setAllLoadActions(Ogre::LoadAction::DontCare);
          passScene->mLoadActionDepth = Ogre::LoadAction::Clear;
          passScene->mLoadActionStencil = Ogre::LoadAction::Clear;
        }
        else
        {
          passScene->setAllLoadActions(Ogre::LoadAction::Clear);
          passScene->setAllClearColours(this->ogreBackgroundColor);
        }
      }

      // render background, e.g. sky, after opaque stuff
      if (validBackground)
      {
        // quad pass
        Ogre::CompositorPassQuadDef *passQuad =
            static_cast<Ogre::CompositorPassQuadDef *>(
            rt0TargetDef->addPass(Ogre::PASS_QUAD));
        passQuad->mMaterialName = this->dataPtr->kSkyboxMaterialName + "_"
            + this->Name();
        passQuad->mFrustumCorners =
            Ogre::CompositorPassQuadDef::CAMERA_DIRECTION;
      }

      // scene pass - transparent stuff
      {
        Ogre::CompositorPassSceneDef *passScene =
            static_cast<Ogre::CompositorPassSceneDef *>(
            rt0TargetDef->addPass(Ogre::PASS_SCENE));
        passScene->mIncludeOverlays = true;
        passScene->mShadowNode = this->dataPtr->kShadowNodeName;
        passScene->mFirstRQ = 2u;
      }
    }

    nodeDef->mapOutputChannel(0, "rt0");
    nodeDef->mapOutputChannel(1, "rt1");

    // Final Composition
    std::string finalNodeDefName = wsDefName + "/" +
        this->dataPtr->kFinalNodeName;
    Ogre::CompositorNodeDef *finalNodeDef =
        ogreCompMgr->addNodeDefinition(finalNodeDefName);
    finalNodeDef->addTextureSourceName("rtN", 0,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);
    finalNodeDef->addTextureSourceName("rt_output", 1,
        Ogre::TextureDefinitionBase::TEXTURE_INPUT);

    finalNodeDef->setNumTargetPass(2);
    Ogre::CompositorTargetDef *outTargetDef =
        finalNodeDef->addTargetPass("rt_output");
    outTargetDef->setNumPasses(2);
    {
      // quad pass
      Ogre::CompositorPassQuadDef *passQuad =
          static_cast<Ogre::CompositorPassQuadDef *>(
          outTargetDef->addPass(Ogre::PASS_QUAD));
      passQuad->mMaterialName = "Ogre/Copy/4xFP32";
      passQuad->addQuadTextureSource(0, "rtN");

      // scene pass
      Ogre::CompositorPassSceneDef *passScene =
          static_cast<Ogre::CompositorPassSceneDef *>(
          outTargetDef->addPass(Ogre::PASS_SCENE));
      passScene->mUpdateLodLists = false;
      passScene->mIncludeOverlays = true;
      passScene->mFirstRQ = 254;
      passScene->mLastRQ = 255;

    }
    Ogre::CompositorWorkspaceDef *workDef =
        ogreCompMgr->addWorkspaceDefinition(wsDefName);

    workDef->connectExternal(0, nodeDefName, 0);
    workDef->connectExternal(1, nodeDefName, 1);

    if (!this->IsRenderWindow())
    {
      workDef->connect(nodeDefName, finalNodeDefName);
    }
    else
    {
      // connect the last render pass to the final compositor node
      // but only input, since output goes to the render window
      workDef->connect(nodeDefName, 0, finalNodeDefName, 0);
      workDef->connectExternal(2,  finalNodeDefName, 1);
    }
  }

  Ogre::CompositorChannelVec externalTargets(2u);
  for (size_t i = 0u; i < 2u; ++i)
  {
    // Connect them in reverse order
    const size_t srcIdx = 2u - i - 1u;
    externalTargets[i] = this->dataPtr->ogreTexture[srcIdx];
  }

  this->ogreCompositorWorkspace =
      ogreCompMgr->addWorkspace(
        this->scene->OgreSceneManager(),
        externalTargets,
        this->ogreCamera,
        this->ogreCompositorWorkspaceDefName,
        false);

  this->dataPtr->rtListener = new Ogre2RenderTargetCompositorListener(this);
  this->ogreCompositorWorkspace->addListener(this->dataPtr->rtListener);
  this->ogreCompositorWorkspace->addListener(engine->TerraWorkspaceListener());
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::DestroyCompositor()
{
  if (!this->ogreCompositorWorkspace)
    return;

  // Restore the original order so that this->ogreTexture[1] is the one with
  // FSAA (which we need for BuildCompositor to connect correctly)
  const Ogre::CompositorChannelVec &externalTargets =
      this->ogreCompositorWorkspace->getExternalRenderTargets();
  for (size_t i = 0u; i < 2u; ++i)
  {
    const size_t srcIdx = (2u - i - 1u);
    this->dataPtr->ogreTexture[srcIdx] = externalTargets[i];
  }

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();
  this->ogreCompositorWorkspace->addListener(nullptr);
  ogreCompMgr->removeWorkspace(this->ogreCompositorWorkspace);
  ogreCompMgr->removeWorkspaceDefinition(this->ogreCompositorWorkspaceDefName);
  ogreCompMgr->removeNodeDefinition(this->ogreCompositorWorkspaceDefName +
      "/" + this->dataPtr->kBaseNodeName);
  ogreCompMgr->removeNodeDefinition(this->ogreCompositorWorkspaceDefName +
      "/" + this->dataPtr->kFinalNodeName);

  this->ogreCompositorWorkspace = nullptr;
  delete this->dataPtr->rtListener;
  this->dataPtr->rtListener = nullptr;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::RebuildCompositor()
{
  this->DestroyCompositor();
  this->BuildCompositor();
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::Copy(Image &_image) const
{
  // TODO(anyone) handle Bayer conversions

  if (_image.Width() != this->width || _image.Height() != this->height)
  {
    ignerr << "Invalid image dimensions" << std::endl;
    return;
  }

  Ogre::PixelFormatGpu dstOgrePf = Ogre2Conversions::Convert(_image.Format());
  Ogre::TextureGpu *texture = this->RenderTarget();

  if (Ogre::PixelFormatGpuUtils::isSRgb(dstOgrePf) !=
      Ogre::PixelFormatGpuUtils::isSRgb(texture->getPixelFormat()))
  {
    // Formats are identical except for sRGB-ness.
    // Force a raw copy by making them match (no conversion!).
    // We can't change the TextureGpu format now, so we change dstOgrePf
    if (Ogre::PixelFormatGpuUtils::isSRgb(texture->getPixelFormat()))
      dstOgrePf = Ogre::PixelFormatGpuUtils::getEquivalentSRGB(dstOgrePf);
    else
      dstOgrePf = Ogre::PixelFormatGpuUtils::getEquivalentLinear(dstOgrePf);
  }

  Ogre::TextureBox dstBox(
    texture->getWidth(), texture->getHeight(),
    texture->getDepth(), texture->getNumSlices(),
    static_cast<uint32_t>(
      Ogre::PixelFormatGpuUtils::getBytesPerPixel(dstOgrePf)),
    static_cast<uint32_t>(Ogre::PixelFormatGpuUtils::getSizeBytes(
      texture->getWidth(), 1u, 1u, 1u, dstOgrePf, 1u)),
    static_cast<uint32_t>(Ogre::PixelFormatGpuUtils::getSizeBytes(
      texture->getWidth(), texture->getHeight(), 1u, 1u,
      dstOgrePf, 1u)));
  dstBox.data = _image.Data();

  Ogre::Image2::copyContentsToMemory(texture, texture->getEmptyBox(0u), dstBox,
                                     dstOgrePf);
}

//////////////////////////////////////////////////
Ogre::Camera *Ogre2RenderTarget::Camera() const
{
  return this->ogreCamera;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetCamera(Ogre::Camera *_camera)
{
  this->ogreCamera = _camera;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
math::Color Ogre2RenderTarget::BackgroundColor() const
{
  return Ogre2Conversions::Convert(this->ogreBackgroundColor);
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetBackgroundColor(math::Color _color)
{
  this->ogreBackgroundColor = Ogre2Conversions::Convert(_color);
  this->colorDirty = true;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetBackgroundMaterial(MaterialPtr _material)
{
  this->backgroundMaterial = _material;
  this->backgroundMaterialDirty = true;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
MaterialPtr Ogre2RenderTarget::BackgroundMaterial() const
{
  return this->backgroundMaterial;
}

//////////////////////////////////////////////////
unsigned int Ogre2RenderTarget::AntiAliasing() const
{
  return this->antiAliasing;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetAntiAliasing(unsigned int _aa)
{
  this->antiAliasing = _aa;
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::PreRender()
{
  BaseRenderTarget::PreRender();
  this->UpdateBackgroundColor();

  if (this->material)
  {
    this->material->PreRender();
  }
  this->UpdateRenderPassChain();
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::PostRender()
{
  // do nothing by default
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::Render()
{
  this->scene->StartRendering(this->ogreCamera);

  this->ogreCompositorWorkspace->_validateFinalTarget();
  this->ogreCompositorWorkspace->_beginUpdate(false);
  this->ogreCompositorWorkspace->_update();
  this->ogreCompositorWorkspace->_endUpdate(false);

  Ogre::vector<Ogre::TextureGpu*>::type swappedTargets;
  swappedTargets.reserve(2u);
  this->ogreCompositorWorkspace->_swapFinalTarget(swappedTargets);

  this->scene->FlushGpuCommandsAndStartNewFrame(1u, false);
}

//////////////////////////////////////////////////
bool Ogre2RenderTarget::IsRenderWindow() const
{
  return false;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::DestroyTargetImpl()
{
  if (nullptr == this->dataPtr->ogreTexture[0])
    return;

  this->DestroyCompositor();

  Ogre::Root *root = Ogre2RenderEngine::Instance()->OgreRoot();

  Ogre::TextureGpuManager *textureManager =
    root->getRenderSystem()->getTextureGpuManager();
  for (size_t i = 0u; i < 2u; ++i)
  {
    textureManager->destroyTexture(this->dataPtr->ogreTexture[i]);
    this->dataPtr->ogreTexture[i] = nullptr;
  }

  // TODO(anyone) there is memory leak when a render texture is destroyed.
  // The RenderSystem::_cleanupDepthBuffers method used in ogre1 does not
  // seem to work in ogre2
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::BuildTargetImpl()
{
  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::TextureGpuManager *textureMgr =
      ogreRoot->getRenderSystem()->getTextureGpuManager();

  for (size_t i = 0u; i < 2u; ++i)
  {
    this->dataPtr->ogreTexture[i] =
        textureMgr->createTexture(
          this->name + std::to_string(i),
          Ogre::GpuPageOutStrategy::Discard,
          Ogre::TextureFlags::RenderToTexture,
          Ogre::TextureTypes::Type2D);

    this->dataPtr->ogreTexture[i]->setResolution(this->width, this->height);
    this->dataPtr->ogreTexture[i]->setNumMipmaps(1u);
    this->dataPtr->ogreTexture[i]->setPixelFormat(Ogre::PFG_RGBA8_UNORM_SRGB);

    this->dataPtr->ogreTexture[i]->scheduleTransitionTo(
          Ogre::GpuResidency::Resident);
  }
}

//////////////////////////////////////////////////
unsigned int Ogre2RenderTarget::GLIdImpl() const
{
  if (!this->dataPtr->ogreTexture[0])
    return 0;

  unsigned int texId;
  this->dataPtr->ogreTexture[1]->getCustomAttribute("msFinalTextureBuffer",
                                                    &texId);

  return static_cast<unsigned int>(texId);
}

//////////////////////////////////////////////////
uint8_t Ogre2RenderTarget::TargetFSAA() const
{
  // check if target fsaa is supported
  std::vector<unsigned int> fsaaLevels =
      Ogre2RenderEngine::Instance()->FSAALevels();
  unsigned int targetFSAA = this->antiAliasing;
  auto const it = std::find(fsaaLevels.begin(), fsaaLevels.end(), targetFSAA);

  if (it == fsaaLevels.end())
  {
    // output warning but only do it once
    static bool ogre2FSAAWarn = false;
    if (!ogre2FSAAWarn)
    {
      std::ostringstream os;
      os << "[ ";
      for (auto &&level : fsaaLevels)
      {
        os << level << " ";
      }
      os << "]";

      ignwarn << "Anti-aliasing level of '" << this->antiAliasing << "' "
              << "is not supported; valid FSAA levels are: " << os.str()
              << ". Setting to 0" << std::endl;
      ogre2FSAAWarn = true;
    }
    targetFSAA = 0u;
  }

  if (targetFSAA == 0u)
    targetFSAA = 1u;

  return static_cast<uint8_t>(targetFSAA);
}

//////////////////////////////////////////////////
Ogre::TextureGpu *Ogre2RenderTarget::RenderTargetImpl() const
{
  return this->dataPtr->ogreTexture[1];
}

//////////////////////////////////////////////////
uint32_t Ogre2RenderTarget::VisibilityMask() const
{
  return this->visibilityMask;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetVisibilityMask(uint32_t _mask)
{
  this->visibilityMask = _mask;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateBackgroundColor()
{
  if (this->colorDirty)
  {
    // set background color in the first pass that clears
    // the RT (both node and its definition).
    auto nodeSeq = this->ogreCompositorWorkspace->getNodeSequence();
    auto pass = nodeSeq[0]->_getPasses()[0];
    pass->getRenderPassDesc()->setClearColour(this->ogreBackgroundColor);

    auto passDef = pass->getDefinition();
    const_cast<Ogre::CompositorPassDef*>(passDef)->setAllClearColours(
          this->ogreBackgroundColor);

    this->colorDirty = false;
  }
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateBackgroundMaterial()
{
  if (!this->backgroundMaterialDirty)
    return;

  bool validBackground = this->backgroundMaterial &&
      !this->backgroundMaterial->EnvironmentMap().empty();

  if (validBackground)
  {
    Ogre::MaterialManager &matManager = Ogre::MaterialManager::getSingleton();
    std::string skyMatName = this->dataPtr->kSkyboxMaterialName + "_"
        + this->Name();
    auto mat = matManager.getByName(skyMatName);
    if (!mat)
    {
      auto skyboxMat = matManager.getByName(this->dataPtr->kSkyboxMaterialName);
      if (!skyboxMat)
      {
        ignerr << "Unable to find skybox material" << std::endl;
        return;
      }
      mat = skyboxMat->clone(skyMatName);
    }
    Ogre::TextureUnitState *texUnit =
        mat->getTechnique(0u)->getPass(0u)->getTextureUnitState(0u);
    texUnit->setTextureName(this->backgroundMaterial->EnvironmentMap(),
        Ogre::TextureTypes::TypeCube);
    texUnit->setHardwareGammaEnabled(false);
  }

  this->backgroundMaterialDirty = false;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateRenderPassChain()
{
  UpdateRenderPassChain(this->ogreCompositorWorkspace,
      this->ogreCompositorWorkspaceDefName,
      this->ogreCompositorWorkspaceDefName + "/" +
      this->dataPtr->kBaseNodeName,
      this->ogreCompositorWorkspaceDefName + "/" +
      this->dataPtr->kFinalNodeName,
      this->renderPasses,
      this->renderPassDirty,
      &this->dataPtr->ogreTexture,
      this->IsRenderWindow());

  if (this->renderPassDirty)
  {
    // make sure to render the result after updating the render pass chain
    for (auto &pass : this->renderPasses)
      pass->PreRender();
  }

  this->renderPassDirty = false;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::UpdateRenderPassChain(
    Ogre::CompositorWorkspace *_workspace, const std::string &_workspaceDefName,
    const std::string &_baseNode, const std::string &_finalNode,
    const std::vector<RenderPassPtr> &_renderPasses,
    bool _recreateNodes, Ogre::TextureGpu *(*_ogreTextures)[2],
    bool _isRenderWindow)
{
  if (!_workspace || _workspaceDefName.empty() ||
      _baseNode.empty() || _finalNode.empty() || _renderPasses.empty())
    return;

  // check pass enabled state and update connections if necessary.
  // If render pass is dirty then skip the enabled state check since the whole
  // workspace nodes and connections will be recreated
  bool updateConnection = false;
  if (!_recreateNodes)
  {
    auto nodeSeq = _workspace->getNodeSequence();

    // set node instance to render pass and update enabled state
    for (const auto &pass : _renderPasses)
    {
      Ogre2RenderPass *ogre2RenderPass =
          dynamic_cast<Ogre2RenderPass *>(pass.get());
      Ogre::CompositorNode *node =
          _workspace->findNodeNoThrow(
          ogre2RenderPass->OgreCompositorNodeDefinitionName());

      // check if we need to create all nodes or just update the connections.
      // if node does not exist then it means it either has not been added to
      // the chain yet or it was removed because it was disabled.
      // In both cases, we need to recreate the nodes and connections
      if (!node && ogre2RenderPass->IsEnabled())
      {
        _recreateNodes = true;
      }
      else if (node && node->getEnabled() != ogre2RenderPass->IsEnabled())
      {
        node->setEnabled(ogre2RenderPass->IsEnabled());
        updateConnection = true;
      }
    }
  }

  if (!_recreateNodes && !updateConnection)
    return;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  Ogre::CompositorWorkspaceDef *workspaceDef =
    ogreCompMgr->getWorkspaceDefinition(_workspaceDefName);

  auto nodeSeq = _workspace->getNodeSequence();

  // The first node and final node in the workspace are defined in
  // PbsMaterials.compositor
  // the first node is the base scene pass node
  std::string outNodeDefName = _baseNode;
  // the final compositor node
  const std::string finalNodeDefName = _finalNode;
  std::string inNodeDefName;

  // if new nodes need to be added then clear everything,
  // otherwise clear only the node connections
  if (_recreateNodes)
    workspaceDef->clearAll();
  else
    workspaceDef->clearAllInterNodeConnections();

  int numActiveNodes = 0;

  // chain the render passes by connecting all the ogre compositor nodes
  // in between the base scene pass node and the final compositor node
  for (const auto &pass : _renderPasses)
  {
    Ogre2RenderPass *ogre2RenderPass =
        dynamic_cast<Ogre2RenderPass *>(pass.get());
    ogre2RenderPass->CreateRenderPass();
    inNodeDefName = ogre2RenderPass->OgreCompositorNodeDefinitionName();
    // only connect passes that are enabled
    if (!inNodeDefName.empty() && ogre2RenderPass->IsEnabled())
    {
      workspaceDef->connect(outNodeDefName, inNodeDefName);
      outNodeDefName = inNodeDefName;
      ++numActiveNodes;
    }
  }

  workspaceDef->connectExternal(0, _baseNode, 0);
  workspaceDef->connectExternal(1, _baseNode, 1);

  if (!_isRenderWindow)
  {
    // connect the last render pass to the final compositor node
    workspaceDef->connect(outNodeDefName, finalNodeDefName);

    // We must ensure the output is always in ogreTextures[1]
    const bool bMustSwapRts = (numActiveNodes & 0x01) == 0u;

    const Ogre::CompositorChannelVec &externalTargets =
        _workspace->getExternalRenderTargets();
    for (size_t i = 0u; i < 2u; ++i)
    {
      const size_t srcIdx = bMustSwapRts ? (2u - i - 1u) : i;
      (*_ogreTextures)[srcIdx] = externalTargets[i];
    }
  }
  else
  {
    // connect the last render pass to the final compositor node
    // but only input, since output goes to the render window
    workspaceDef->connect(outNodeDefName, 0, finalNodeDefName, 0);
    workspaceDef->connectExternal(2, _finalNode, 1);
  }


  // if new node definitions were added then recreate all the compositor nodes,
  // otherwise update the connections
  if (_recreateNodes)
  {
    // clearAll requires the output to be connected again.
    _workspace->recreateAllNodes();
  }
  else
  {
    _workspace->reconnectAllNodes();
  }
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::RebuildImpl()
{
  this->RebuildTarget();
  this->RebuildMaterial();
  this->RebuildCompositor();
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetMaterial(MaterialPtr _material)
{
  this->material = _material;

  // Have to rebuild the target so there is something to apply
  // the applicator to
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::SetShadowsNodeDefDirty()
{
  this->DestroyCompositor();

  // Have to rebuild the target so there is something to apply the applicator to
  this->targetDirty = true;
}

//////////////////////////////////////////////////
void Ogre2RenderTarget::RebuildMaterial()
{
  if (this->material)
  {
    Ogre2Material *ogreMaterial = dynamic_cast<Ogre2Material *>(
        this->material.get());
    Ogre::MaterialPtr matPtr = ogreMaterial->Material();

    Ogre::SceneManager *sceneMgr = this->scene->OgreSceneManager();
    this->materialApplicator.reset(new Ogre2RenderTargetMaterial(
        sceneMgr, this->ogreCamera, matPtr.get()));
  }
}

//////////////////////////////////////////////////
// Ogre2RenderTexture
//////////////////////////////////////////////////
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
Ogre2RenderTexture::Ogre2RenderTexture()
{
}
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif

//////////////////////////////////////////////////
Ogre2RenderTexture::~Ogre2RenderTexture()
{
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::Destroy()
{
  this->DestroyTarget();
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::RebuildTarget()
{
  this->DestroyTarget();
  this->BuildTarget();
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::DestroyTarget()
{
  Ogre2RenderTarget::DestroyTargetImpl();
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::BuildTarget()
{
  Ogre2RenderTarget::BuildTargetImpl();
}

//////////////////////////////////////////////////
unsigned int Ogre2RenderTexture::GLId() const
{
  return Ogre2RenderTarget::GLIdImpl();
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::PreRender()
{
  Ogre2RenderTarget::PreRender();
}

//////////////////////////////////////////////////
void Ogre2RenderTexture::PostRender()
{
  Ogre2RenderTarget::PostRender();
}

//////////////////////////////////////////////////
Ogre::TextureGpu *Ogre2RenderTexture::RenderTarget() const
{
  return Ogre2RenderTarget::RenderTargetImpl();
}

//////////////////////////////////////////////////
// Ogre2RenderWindow
//////////////////////////////////////////////////
Ogre2RenderWindow::Ogre2RenderWindow()
{
}

//////////////////////////////////////////////////
Ogre2RenderWindow::~Ogre2RenderWindow()
{
}

//////////////////////////////////////////////////
bool Ogre2RenderWindow::IsRenderWindow() const
{
  return true;
}

//////////////////////////////////////////////////
Ogre::TextureGpu *Ogre2RenderWindow::RenderTarget() const
{
  return this->ogreRenderWindow;
}

//////////////////////////////////////////////////
void Ogre2RenderWindow::Destroy()
{
  // TODO(anyone)
}

//////////////////////////////////////////////////
void Ogre2RenderWindow::RebuildTarget()
{
  // TODO(anyone): determine when to rebuild
  // ie. only when ratio or handle changes!
  // e.g. sizeDirty?
  if (!this->ogreRenderWindow)
    this->BuildTarget();

  Ogre::Window *window =
      dynamic_cast<Ogre::Window *>(this->ogreRenderWindow);
  window->requestResolution(this->width, this->height);
  window->getTexture()->setResolution(this->width, this->height);
  window->windowMovedOrResized();
}

//////////////////////////////////////////////////
void Ogre2RenderWindow::BuildTarget()
{
  auto engine = Ogre2RenderEngine::Instance();
  engine->CreateRenderWindow(this->handle,
      this->width,
      this->height,
      this->ratio,
      this->antiAliasing);

  this->ogreRenderWindow = engine->OgreWindow()->getTexture();
}
