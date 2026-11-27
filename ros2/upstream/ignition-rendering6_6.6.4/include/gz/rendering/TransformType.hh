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

#ifndef GZ_RENDERING_TRANSFORMTYPE_HH_
#define GZ_RENDERING_TRANSFORMTYPE_HH_

#include <gz/math/Vector3.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \enum TransformAxis
    /// \brief Unique identifiers for transformation axes.
    enum IGNITION_RENDERING_VISIBLE TransformAxis
    {
      /// \brief No axis
      TA_NONE = 0,
      /// \brief Translation in x
      TA_TRANSLATION_X = 0x00000001,
      /// \brief Translation in y
      TA_TRANSLATION_Y = 0x00000002,
      /// \brief Translation in z
      TA_TRANSLATION_Z = 0x00000004,
      /// \brief Rotation in x
      TA_ROTATION_X = 0x00000010,
      /// \brief Rotation in y
      TA_ROTATION_Y = 0x00000020,
      /// \brief Rotation in z
      TA_ROTATION_Z = 0x00000040,
      /// \brief Scale in x
      TA_SCALE_X = 0x00000100,
      /// \brief Scale in y
      TA_SCALE_Y = 0x00000200,
      /// \brief Scale in z
      TA_SCALE_Z = 0x00000400
    };

    /// \enum TransformMode
    /// \brief Unique identifiers for transformation modes.
    enum IGNITION_RENDERING_VISIBLE TransformMode
    {
      /// \brief Inactive state
      TM_NONE = 0,
      /// \brief Translation mode
      TM_TRANSLATION = TA_TRANSLATION_X | TA_TRANSLATION_Y | TA_TRANSLATION_Z,
      /// \brief Rotation mode
      TM_ROTATION = TA_ROTATION_X | TA_ROTATION_Y | TA_ROTATION_Z,
      /// \brief Scale mode
      TM_SCALE = TA_SCALE_X | TA_SCALE_Y, TA_SCALEZ,
    };

    /// \enum TransformSpace
    /// \brief Unique identifiers for transformation spaces.
    enum IGNITION_RENDERING_VISIBLE TransformSpace
    {
      /// \brief transformation in local frame
      TS_LOCAL,
      /// \brief transformation in world frame
      TS_WORLD
    };
    }
  }
}
#endif
