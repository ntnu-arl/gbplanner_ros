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
#ifndef GZ_RENDERING_RENDERPASS_HH_
#define GZ_RENDERING_RENDERPASS_HH_

#include "gz/rendering/config.hh"
#include "gz/rendering/Object.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class RenderPass RenderPass.hh ignition/rendering/RenderPass.hh
    /// \brief A render pass can be added to a camera to affect how the scene
    /// is rendered. It can be used to add post-processing effects. Multiple
    /// passes can be chained together.
    class IGNITION_RENDERING_VISIBLE RenderPass
      : public virtual Object
    {
      /// \brief Destructor
      public: virtual ~RenderPass() { }

      /// \brief Set to enable or disable the render pass
      /// \param[in] _enabled True to enable the render pass, false to disable.
      public: virtual void SetEnabled(bool _enabled) = 0;

      /// \brief Get whether or not the render pass is enabled
      /// \return True if the render pass is enabled, false otherwise.
      public: virtual bool IsEnabled() const = 0;
    };
    }
  }
}
#endif
