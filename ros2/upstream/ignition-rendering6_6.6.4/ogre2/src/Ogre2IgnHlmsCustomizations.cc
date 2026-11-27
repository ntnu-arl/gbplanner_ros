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

#include "Ogre2IgnHlmsCustomizations.hh"

#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreCamera.h>
#include <OgreHlms.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreViewport.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
void Ogre2IgnHlmsCustomizations::preparePassHash(
    const Ogre::CompositorShadowNode */*_shadowNode*/,
    bool _casterPass, bool /*_dualParaboloid*/,
    Ogre::SceneManager */*_sceneManager*/,
    Ogre::Hlms *_hlms)
{
  this->needsWorldPos = false;
  if (!_casterPass && this->MinDistanceClipEnabled())
  {
    const Ogre::int32 numClipPlanes =
      _hlms->_getProperty("hlms_pso_clip_distances");
    _hlms->_setProperty("ign_spherical_clip_min_distance", 1);
    _hlms->_setProperty("ign_spherical_clip_idx", numClipPlanes);
    _hlms->_setProperty("hlms_pso_clip_distances", numClipPlanes + 1);

    if (_hlms->getType() == Ogre::HLMS_UNLIT)
    {
      if (_hlms->_getProperty("hlms_global_clip_planes") == 0)
      {
        this->needsWorldPos = true;
        _hlms->_setProperty("ign_spherical_clip_needs_worldPos", 1);
      }
    }
  }
}

//////////////////////////////////////////////////
Ogre::uint32 Ogre2IgnHlmsCustomizations::getPassBufferSize(
    const Ogre::CompositorShadowNode */*_shadowNode*/,
    bool _casterPass, bool /*_dualParaboloid*/,
    Ogre::SceneManager */*_sceneManager*/) const
{
  if (_casterPass || !this->MinDistanceClipEnabled())
    return 0u;

  Ogre::uint32 bufferSize = sizeof(float) * 4u;
  if (this->needsWorldPos)
    bufferSize += sizeof(float) * 16u;

  return bufferSize;
}

//////////////////////////////////////////////////
float* Ogre2IgnHlmsCustomizations::preparePassBuffer(
    const Ogre::CompositorShadowNode */*_shadowNode*/,
    bool _casterPass, bool /*_dualParaboloid*/,
    Ogre::SceneManager *_sceneManager,
    float *_passBufferPtr)
{
  if (!_casterPass && this->MinDistanceClipEnabled())
  {
    const Ogre::Camera *camera =
        _sceneManager->getCamerasInProgress().renderingCamera;
    const Ogre::Vector3 &camPos = camera->getDerivedPosition();

    // float4 ignMinClipDistance_ignCameraPos
    *_passBufferPtr++ = this->minDistanceClip;
    *_passBufferPtr++ = camPos.x;
    *_passBufferPtr++ = camPos.y;
    *_passBufferPtr++ = camPos.z;

    if (this->needsWorldPos)
    {
      // TODO(dark_sylinc): Modify Ogre to access HlmsUnlit::mPreparedPass
      // so we get the view matrix that's going to be used instead of
      // recalculating everything again (which can get complex if VR is
      // being used)
      auto engine = Ogre2RenderEngine::Instance();
      auto ogreRoot = engine->OgreRoot();
      Ogre::RenderPassDescriptor *renderPassDesc =
          ogreRoot->getRenderSystem()->getCurrentPassDescriptor();
      Ogre::Matrix4 projectionMatrix =
          camera->getProjectionMatrixWithRSDepth();
      if (renderPassDesc->requiresTextureFlipping())
      {
          projectionMatrix[1][0]  = -projectionMatrix[1][0];
          projectionMatrix[1][1]  = -projectionMatrix[1][1];
          projectionMatrix[1][2]  = -projectionMatrix[1][2];
          projectionMatrix[1][3]  = -projectionMatrix[1][3];
      }

      Ogre::Matrix4 invViewProj = (projectionMatrix *
                                   camera->getViewMatrix(true)).inverse();
      for (size_t i = 0; i < 16u; ++i)
      {
        *_passBufferPtr++ = static_cast<float>(invViewProj[0][i]);
      }
    }
  }
  return _passBufferPtr;
}
