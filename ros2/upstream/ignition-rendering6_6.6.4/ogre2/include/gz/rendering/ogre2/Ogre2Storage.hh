/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE2_OGRE2STORAGE_HH_
#define GZ_RENDERING_OGRE2_OGRE2STORAGE_HH_

#include <memory>

#include "gz/rendering/base/BaseStorage.hh"

#include "gz/rendering/ogre2/Ogre2Light.hh"
#include "gz/rendering/ogre2/Ogre2Mesh.hh"
#include "gz/rendering/ogre2/Ogre2Geometry.hh"
#include "gz/rendering/ogre2/Ogre2Node.hh"
#include "gz/rendering/ogre2/Ogre2Sensor.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2Visual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

// armhf failed to build with this code. It can not be removed for the rest
// of arches to keep ABI but should be removed in major versions unreleased
// see https://github.com/ignitionrobotics/ign-rendering/pull/457
#ifndef __ARM_PCS_VFP
    template class BaseGeometryStore<Ogre2Geometry>;
    template class BaseLightStore<Ogre2Light>;
    template class BaseNodeStore<Ogre2Node>;
    template class BaseSceneStore<Ogre2Scene>;
    template class BaseSensorStore<Ogre2Sensor>;
    template class BaseSubMeshStore<Ogre2SubMesh>;
    template class BaseVisualStore<Ogre2Visual>;
#endif

    typedef BaseGeometryStore<Ogre2Geometry>    Ogre2GeometryStore;
    typedef BaseLightStore<Ogre2Light>          Ogre2LightStore;
    typedef BaseNodeStore<Ogre2Node>            Ogre2NodeStore;
    typedef BaseSceneStore<Ogre2Scene>          Ogre2SceneStore;
    typedef BaseSensorStore<Ogre2Sensor>        Ogre2SensorStore;
    typedef BaseSubMeshStore<Ogre2SubMesh>      Ogre2SubMeshStore;
    typedef BaseVisualStore<Ogre2Visual>        Ogre2VisualStore;

    typedef std::shared_ptr<Ogre2GeometryStore> Ogre2GeometryStorePtr;
    typedef std::shared_ptr<Ogre2LightStore>    Ogre2LightStorePtr;
    typedef std::shared_ptr<Ogre2NodeStore>     Ogre2NodeStorePtr;
    typedef std::shared_ptr<Ogre2SceneStore>    Ogre2SceneStorePtr;
    typedef std::shared_ptr<Ogre2SensorStore>   Ogre2SensorStorePtr;
    typedef std::shared_ptr<Ogre2SubMeshStore>  Ogre2SubMeshStorePtr;
    typedef std::shared_ptr<Ogre2VisualStore>   Ogre2VisualStorePtr;
    }
  }
}
#endif
