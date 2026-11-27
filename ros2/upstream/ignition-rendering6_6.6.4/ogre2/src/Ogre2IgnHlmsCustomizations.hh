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

#ifndef IGNITION_RENDERING_OGRE2_OGRE2IGNHLMSCUSTOMIZATIONS_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2IGNHLMSCUSTOMIZATIONS_HH_

#include "ignition/rendering/config.hh"
#include "ignition/rendering/ogre2/Export.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreHlmsListener.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Controls custom shader snippets of Hlms (both Pbs and Unlit):
    ///
    ///   - Toggles them on/off
    ///   - Sends relevant data to the GPU buffers for shaders to use
    ///
    /// This listener requires Hlms to have been created with the piece data
    /// files in ogre2/src/media/Hlms/Ignition registered
    ///
    /// \internal
    /// \remark Public variables take effect immediately (i.e. for the
    /// next render)
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2IgnHlmsCustomizations final :
        public Ogre::HlmsListener
    {
    public:
      virtual ~Ogre2IgnHlmsCustomizations() = default;

      /// \brief
      /// \return Returns true if spherical clipping customizations should
      /// be active
      public: bool MinDistanceClipEnabled() const
      {
        return this->minDistanceClip >= 0.0f;
      }

      /// \brief Determines which custom pieces we should activate
      /// \param _casterPass
      /// \param _hlms
      private: virtual void preparePassHash(
            const Ogre::CompositorShadowNode *_shadowNode,
            bool _casterPass, bool _dualParaboloid,
            Ogre::SceneManager *_sceneManager,
            Ogre::Hlms *_hlms) override;

      /// \brief Tells Ogre the buffer data sent to GPU should be a little
      /// bigger to fit our data we need to send
      ///
      /// This data is sent in Ogre2IgnHlmsCustomizations::preparePassBuffer
      /// \param _casterPass
      /// \param _hlms
      private: virtual Ogre::uint32 getPassBufferSize(
            const Ogre::CompositorShadowNode *_shadowNode,
            bool _casterPass, bool _dualParaboloid,
            Ogre::SceneManager *_sceneManager) const override;

      /// \brief Sends our custom data to GPU buffers that our
      /// pieces activated in Ogre2IgnHlmsCustomizations::preparePassHash
      /// will need.
      ///
      /// Bytes written must not exceed what we informed in getPassBufferSize
      /// \param _casterPass
      /// \param _sceneManager
      /// \param _passBufferPtr
      /// \return The pointer where Ogre should continue appending more data
      private: virtual float* preparePassBuffer(
            const Ogre::CompositorShadowNode *_shadowNode,
            bool _casterPass, bool _dualParaboloid,
            Ogre::SceneManager *_sceneManager,
            float *_passBufferPtr) override;

      /// \brief Min distance to clip geometry against in a spherical manner
      /// (i.e. vertices that are too close to camera are clipped)
      /// Usually this means the min lidar distance
      ///
      /// Regular near clip distance clips in a rectangular way, so
      /// it's not enough.
      ///
      /// Set to a negative value to disable (0 does NOT disable it!)
      ///
      /// See https://github.com/ignitionrobotics/ign-rendering/pull/356
      public: float minDistanceClip = -1.0f;

      /// \brief When true, we're currently dealing with HlmsUnlit
      /// where we need to define and calculate `float3 worldPos`
      /// \internal
      private: bool needsWorldPos = false;
    };
    }
  }
}
#endif
