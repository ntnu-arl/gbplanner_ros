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
#ifndef GZ_RENDERING_EXAMPLES_ACTOR_ANIMATION_GLUTWINDOW_HH_
#define GZ_RENDERING_EXAMPLES_ACTOR_ANIMATION_GLUTWINDOW_HH_

#include <vector>
#include "gz/rendering/RenderTypes.hh"
#include "gz/common/graphics/Types.hh"

namespace ir = gz::rendering;
namespace ic = gz::common;

/// \brief Run the demo and render the scene from the cameras
/// \param[in] _cameras Cameras in the scene
/// \param[in] _visuals Actor visuals
/// \param[in] _skel Actor skeleton
void run(std::vector<ir::CameraPtr> _cameras,
    const std::vector<ir::VisualPtr> &_visuals,
    ic::SkeletonPtr _skel);

#endif
