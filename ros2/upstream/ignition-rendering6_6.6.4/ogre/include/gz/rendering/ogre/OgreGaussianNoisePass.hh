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
#ifndef GZ_RENDERING_OGRE_OGREGAUSSIANNOISEPASS_HH_
#define GZ_RENDERING_OGRE_OGREGAUSSIANNOISEPASS_HH_

#include <memory>

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/base/BaseGaussianNoisePass.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderPass.hh"
#include "gz/rendering/ogre/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class GaussianNoiseCompositorListener;

    /* \class OgreGaussianNoisePass OgreGaussianNoisePass.hh \
     * ignition/rendering/ogre/OgreGaussianNoisePass.hh
     */
    /// \brief Ogre implementation of the GaussianNoisePass class
    class IGNITION_RENDERING_OGRE_VISIBLE OgreGaussianNoisePass :
      public BaseGaussianNoisePass<OgreRenderPass>
    {
      /// \brief Constructor
      public: OgreGaussianNoisePass();

      /// \brief Destructor
      public: virtual ~OgreGaussianNoisePass();

      // Documentation inherited
      public: void PreRender() override;

      // Documentation inherited
      public: void Destroy() override;

      // Documentation inherited
      public: void CreateRenderPass() override;

      /// \brief Gaussian noise compositor.
      public: Ogre::CompositorInstance *gaussianNoiseInstance = nullptr;

      /// \brief Gaussian noise compositor listener
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      public: std::shared_ptr<GaussianNoiseCompositorListener>
          gaussianNoiseCompositorListener;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
