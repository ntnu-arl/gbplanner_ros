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
#ifndef GZ_RENDERING_OGRE_OGRESTORAGE_HH_
#define GZ_RENDERING_OGRE_OGRESTORAGE_HH_

#include <memory>
#include "gz/rendering/base/BaseStorage.hh"

#include "gz/rendering/ogre/OgreGeometry.hh"
#include "gz/rendering/ogre/OgreLight.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreMesh.hh"
#include "gz/rendering/ogre/OgreNode.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreSensor.hh"
#include "gz/rendering/ogre/OgreVisual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

// armhf failed to build with this code. It can not be removed for the rest
// of arches to keep ABI but should be removed in major versions unreleased
// see https://github.com/ignitionrobotics/ign-rendering/pull/457
#ifndef __ARM_PCS_VFP
    template class BaseSceneStore<OgreScene>;
    template class BaseNodeStore<OgreNode>;
    template class BaseLightStore<OgreLight>;
    template class BaseSensorStore<OgreSensor>;
    template class BaseVisualStore<OgreVisual>;
    template class BaseGeometryStore<OgreGeometry>;
    template class BaseSubMeshStore<OgreSubMesh>;
    template class BaseMaterialMap<OgreMaterial>;
#endif

    typedef BaseSceneStore<OgreScene>       OgreSceneStore;
    typedef BaseNodeStore<OgreNode>         OgreNodeStore;
    typedef BaseLightStore<OgreLight>       OgreLightStore;
    typedef BaseSensorStore<OgreSensor>     OgreSensorStore;
    typedef BaseVisualStore<OgreVisual>     OgreVisualStore;
    typedef BaseGeometryStore<OgreGeometry> OgreGeometryStore;
    typedef BaseSubMeshStore<OgreSubMesh>   OgreSubMeshStore;
    typedef BaseMaterialMap<OgreMaterial>   OgreMaterialMap;

    typedef std::shared_ptr<OgreSceneStore>    OgreSceneStorePtr;
    typedef std::shared_ptr<OgreNodeStore>     OgreNodeStorePtr;
    typedef std::shared_ptr<OgreLightStore>    OgreLightStorePtr;
    typedef std::shared_ptr<OgreSensorStore>   OgreSensorStorePtr;
    typedef std::shared_ptr<OgreVisualStore>   OgreVisualStorePtr;
    typedef std::shared_ptr<OgreGeometryStore> OgreGeometryStorePtr;
    typedef std::shared_ptr<OgreSubMeshStore>  OgreSubMeshStorePtr;
    typedef std::shared_ptr<OgreMaterialMap>   OgreMaterialMapPtr;
    }
  }
}
#endif
