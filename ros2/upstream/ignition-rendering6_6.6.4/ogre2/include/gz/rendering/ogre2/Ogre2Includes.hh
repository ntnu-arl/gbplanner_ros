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
#ifndef GZ_RENDERING_OGRE2_OGRE2INCLUDES_HH_
#define GZ_RENDERING_OGRE2_OGRE2INCLUDES_HH_

// This disables warning messages for OGRE
#ifndef _MSC_VER
  #pragma GCC system_header
#else
  #pragma warning(push, 0)
#endif

// This prevents some deprecation #warning messages on OSX 10.9
#pragma clang diagnostic ignored "-W#warnings"

#include <memory>

#include <Ogre.h>
#include <OgreBillboard.h>
#include <OgreCamera.h>
#include <OgreColourValue.h>
#include <OgreDataStream.h>
#include <OgreDepthBuffer.h>
#include <OgreFrameListener.h>
#include <OgreGpuProgramManager.h>
#include <OgreHardwareBufferManager.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreItem.h>
#include <OgreImage2.h>
#include <OgreLogManager.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreMatrix4.h>
#include <OgreMesh2.h>
#include <OgreMeshManager2.h>
#include <OgreMovableObject.h>
#include <OgreNode.h>
#include <OgreOldSkeletonManager.h>
#include <OgreParticleEmitter.h>
#include <OgreParticleSystem.h>
#include <OgrePass.h>
#include <OgrePixelFormatGpuUtils.h>
#include <OgrePlugin.h>
#include <OgreQuaternion.h>
#include <OgreRectangle2D.h>
#include <OgreRenderObjectListener.h>
#include <OgreRenderable.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreSceneQuery.h>
#include <OgreShadowCameraSetupPSSM.h>
#include <OgreSimpleRenderable.h>
#include <OgreStagingTexture.h>
#include <OgreSubMesh2.h>
#include <OgreTechnique.h>
#include <OgreTextureGpu.h>
#include <OgreTextureGpuManager.h>
#include <OgreTextureUnitState.h>
#include <OgreGpuProgramManager.h>
#include <OgreHighLevelGpuProgramManager.h>
#include <OgreShadowCameraSetupPSSM.h>
#include <OgreDepthBuffer.h>
#include <OgreParticleAffector.h>
#include <OgreParticleAffectorFactory.h>
#include <OgreParticleEmitter.h>
#include <OgreParticleSystem.h>
#include <OgreVector3.h>
#include <OgreWindow.h>
#include <OgreWindowEventUtilities.h>

#include <Hlms/Unlit/OgreHlmsUnlit.h>
#include <Hlms/Pbs/OgreHlmsPbs.h>

#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorNode.h>
#include <Compositor/OgreCompositorNodeDef.h>
#include <Compositor/OgreCompositorShadowNode.h>
#include <Compositor/OgreCompositorWorkspace.h>
#include <Compositor/OgreCompositorWorkspaceListener.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuadDef.h>
#include <Compositor/Pass/PassScene/OgreCompositorPassScene.h>
#include <Compositor/Pass/PassScene/OgreCompositorPassSceneDef.h>

#include <Overlay/OgreOverlayPrerequisites.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgreOverlayElement.h>
#include <Overlay/OgreOverlayContainer.h>
#include <Overlay/OgreFontManager.h>
#include <Overlay/OgreOverlaySystem.h>

// TODO(anyone): enable when ogre 2.1 fully supports paging
// and terrain components
// #include <Paging/OgrePageManager.h>
// #include <Paging/OgrePagedWorld.h>
// #include <Terrain/OgreTerrainPaging.h>
// #include <Terrain/OgreTerrainMaterialGeneratorA.h>
// #include <Terrain/OgreTerrain.h>
// #include <Terrain/OgreTerrainGroup.h>

#ifdef _MSC_VER
#pragma warning(pop)
#endif

#endif
