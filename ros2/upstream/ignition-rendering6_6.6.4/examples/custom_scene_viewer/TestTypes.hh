/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_EXAMPLES_CUSTOM_SCENE_VIEWER_TESTYPES_HH_
#define GZ_RENDERING_EXAMPLES_CUSTOM_SCENE_VIEWER_TESTYPES_HH_

#include <vector>
#include <gz/rendering.hh>

namespace ignition
{
  namespace rendering
  {
    class SceneBuilder;
    class ManualSceneDemo;

    typedef std::shared_ptr<SceneBuilder> SceneBuilderPtr;
    typedef std::shared_ptr<ManualSceneDemo> ManualSceneDemoPtr;
    typedef std::vector<ScenePtr> SceneList;
    typedef std::vector<CameraPtr> CameraList;
    typedef std::vector<SceneBuilderPtr> SceneBuilderList;
  }
}

#endif
