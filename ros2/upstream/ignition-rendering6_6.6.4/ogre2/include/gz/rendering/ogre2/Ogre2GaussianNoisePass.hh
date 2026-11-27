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
#ifndef GZ_RENDERING_OGRE2_OGRE2GAUSSIANNOISEPASS_HH_
#define GZ_RENDERING_OGRE2_OGRE2GAUSSIANNOISEPASS_HH_

#include <memory>

#include "gz/rendering/base/BaseGaussianNoisePass.hh"
#include "gz/rendering/ogre2/Ogre2RenderPass.hh"
#include "gz/rendering/ogre2/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class Ogre2GaussianNoisePassPrivate;

    /* \class Ogre2GaussianNoisePass Ogre2GaussianNoisePass.hh \
     * ignition/rendering/ogre2/Ogre2GaussianNoisePass.hh
     */
    /// \brief Ogre2 Implementation of a Gaussian noise render pass.
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2GaussianNoisePass :
      public BaseGaussianNoisePass<Ogre2RenderPass>
    {
      /// \brief Constructor
      public: Ogre2GaussianNoisePass();

      /// \brief Destructor
      public: virtual ~Ogre2GaussianNoisePass();

      // Documentation inherited
      public: void PreRender() override;

      // Documentation inherited
      public: void CreateRenderPass() override;

      /// \brief Pointer to private data class
      private: std::unique_ptr<Ogre2GaussianNoisePassPrivate> dataPtr;
    };
    }
  }
}
#endif
