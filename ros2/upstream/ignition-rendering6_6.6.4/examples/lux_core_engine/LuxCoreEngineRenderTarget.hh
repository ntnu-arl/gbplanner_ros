/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINERENDERTARGET_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINERENDERTARGET_HH_

#include "ignition/rendering/base/BaseRenderTarget.hh"

#include "LuxCoreEngineObject.hh"
#include "LuxCoreEngineRenderTypes.hh"

namespace ignition {
namespace rendering {
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

/// \brief LuxCore implementation of the render target class
class LuxCoreEngineRenderTarget
    : public virtual BaseRenderTarget<LuxCoreEngineObject>
{
  /// \brief Constructor
  protected: LuxCoreEngineRenderTarget();

  /// \brief Destructor
  public: virtual ~LuxCoreEngineRenderTarget();

  // Documentation inherited.
  public: virtual void Copy(Image &_image) const override;

  /// \brief Get host data buffer
  /// \return hostDataBuffer
  public: void *HostDataBuffer();

  /// \brief Allocate memory to host data buffer
  /// \param[in] size of host data buffer
  public: void ResizeHostDataBuffer(unsigned int size);

  /// \brief Get Memory Size
  /// \return memory size
  protected: unsigned int MemorySize() const;

  // Documentation inherited.
  protected: virtual void RebuildImpl() override;

  protected: void *hostDataBuffer{nullptr};

  private: friend class LuxCoreEngineCamera;
};

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition
#endif
