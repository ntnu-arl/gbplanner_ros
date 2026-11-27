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
#ifndef GZ_RENDERING_OGRE_OGRERENDERPASS_HH_
#define GZ_RENDERING_OGRE_OGRERENDERPASS_HH_

#include "gz/rendering/base/BaseRenderPass.hh"
#include "gz/rendering/ogre/Export.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreObject.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /* \class OgreRenderPass OgreRenderPass.hh \
     * ignition/rendering/ogre/OgreRenderPass.hh
     */
    /// \brief Ogre implementation of the RenderPass class
    class IGNITION_RENDERING_OGRE_VISIBLE OgreRenderPass :
      public BaseRenderPass<OgreObject>
    {
      /// \brief Constructor
      protected: OgreRenderPass();

      /// \brief Destructor
      public: virtual ~OgreRenderPass();

      /// \brief Set the ogre camera that the render pass applies to
      /// \param[in] _camera Pointer to the ogre camera.
      public: virtual void SetCamera(Ogre::Camera *_camera);

      // Documentation inherited.
      public: void Destroy() override;

      /// \brief Create the render pass using ogre compositor
      public: virtual void CreateRenderPass();

      /// \brief Pointer to the ogre camera
      protected: Ogre::Camera *ogreCamera = nullptr;
    };
    }
  }
}
#endif
