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
#ifndef GZ_RENDERING_OPTIX_OPTIXSTORAGE_HH_
#define GZ_RENDERING_OPTIX_OPTIXSTORAGE_HH_

#include "gz/rendering/base/BaseStorage.hh"

#include "gz/rendering/optix/OptixGeometry.hh"
#include "gz/rendering/optix/OptixLight.hh"
#include "gz/rendering/optix/OptixMaterial.hh"
#include "gz/rendering/optix/OptixMesh.hh"
#include "gz/rendering/optix/OptixNode.hh"
#include "gz/rendering/optix/OptixScene.hh"
#include "gz/rendering/optix/OptixSensor.hh"
#include "gz/rendering/optix/OptixVisual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template class BaseSceneStore<OptixScene>;
    template class BaseNodeStore<OptixNode>;
    template class BaseLightStore<OptixLight>;
    template class BaseSensorStore<OptixSensor>;
    template class BaseVisualStore<OptixVisual>;
    template class BaseGeometryStore<OptixGeometry>;
    template class BaseSubMeshStore<OptixSubMesh>;
    template class BaseMaterialMap<OptixMaterial>;

    typedef BaseSceneStore<OptixScene>       OptixSceneStore;
    typedef BaseNodeStore<OptixNode>         OptixNodeStore;
    typedef BaseLightStore<OptixLight>       OptixLightStore;
    typedef BaseSensorStore<OptixSensor>     OptixSensorStore;
    typedef BaseVisualStore<OptixVisual>     OptixVisualStore;
    typedef BaseGeometryStore<OptixGeometry> OptixGeometryStore;
    typedef BaseSubMeshStore<OptixSubMesh>   OptixSubMeshStore;
    typedef BaseMaterialMap<OptixMaterial>   OptixMaterialMap;

    typedef std::shared_ptr<OptixSceneStore>    OptixSceneStorePtr;
    typedef std::shared_ptr<OptixNodeStore>     OptixNodeStorePtr;
    typedef std::shared_ptr<OptixLightStore>    OptixLightStorePtr;
    typedef std::shared_ptr<OptixSensorStore>   OptixSensorStorePtr;
    typedef std::shared_ptr<OptixVisualStore>   OptixVisualStorePtr;
    typedef std::shared_ptr<OptixGeometryStore> OptixGeometryStorePtr;
    typedef std::shared_ptr<OptixSubMeshStore>  OptixSubMeshStorePtr;
    typedef std::shared_ptr<OptixMaterialMap>   OptixMaterialMapPtr;
    }
  }
}
#endif
