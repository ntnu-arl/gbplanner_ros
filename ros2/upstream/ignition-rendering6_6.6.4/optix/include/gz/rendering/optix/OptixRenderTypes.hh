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
#ifndef GZ_RENDERING_OPTIX_OPTIXRENDERTYPES_HH_
#define GZ_RENDERING_OPTIX_OPTIXRENDERTYPES_HH_

#include "gz/rendering/base/BaseRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class OptixArrowVisual;
    class OptixAxisVisual;
    class OptixBox;
    class OptixCamera;
    class OptixCone;
    class OptixCylinder;
    class OptixDirectionalLight;
    class OptixGeometry;
    class OptixGrid;
    class OptixJointVisual;
    class OptixLight;
    class OptixMaterial;
    class OptixMesh;
    class OptixMeshFactory;
    class OptixNode;
    class OptixObject;
    class OptixObjectFactory;
    class OptixPointLight;
    class OptixRenderEngine;
    class OptixScene;
    class OptixSensor;
    class OptixSphere;
    class OptixSpotLight;
    class OptixSubMesh;
    class OptixVisual;
    class OptixRenderTarget;
    class OptixRenderTexture;
    class OptixRenderWindow;
    class OptixLightManager;

    typedef BaseSceneStore<OptixScene>       OptixSceneStore;
    typedef BaseNodeStore<OptixNode>         OptixNodeStore;
    typedef BaseLightStore<OptixLight>       OptixLightStore;
    typedef BaseSensorStore<OptixSensor>     OptixSensorStore;
    typedef BaseVisualStore<OptixVisual>     OptixVisualStore;
    typedef BaseGeometryStore<OptixGeometry> OptixGeometryStore;
    typedef BaseSubMeshStore<OptixSubMesh>   OptixSubMeshStore;
    typedef BaseMaterialMap<OptixMaterial>   OptixMaterialMap;

    typedef shared_ptr<OptixArrowVisual>          OptixArrowVisualPtr;
    typedef shared_ptr<OptixAxisVisual>           OptixAxisVisualPtr;
    typedef shared_ptr<OptixBox>                  OptixBoxPtr;
    typedef shared_ptr<OptixCamera>               OptixCameraPtr;
    typedef shared_ptr<OptixCone>                 OptixConePtr;
    typedef shared_ptr<OptixCylinder>             OptixCylinderPtr;
    typedef shared_ptr<OptixDirectionalLight>     OptixDirectionalLightPtr;
    typedef shared_ptr<OptixGeometry>             OptixGeometryPtr;
    typedef shared_ptr<OptixGrid>                 OptixGridPtr;
    typedef shared_ptr<OptixJointVisual>          OptixJointVisualPtr;
    typedef shared_ptr<OptixLight>                OptixLightPtr;
    typedef shared_ptr<OptixMaterial>             OptixMaterialPtr;
    typedef shared_ptr<OptixMesh>                 OptixMeshPtr;
    typedef shared_ptr<OptixMeshFactory>          OptixMeshFactoryPtr;
    typedef shared_ptr<OptixNode>                 OptixNodePtr;
    typedef shared_ptr<OptixObject>               OptixObjectPtr;
    typedef shared_ptr<OptixObjectFactory>        OptixObjectFactoryPtr;
    typedef shared_ptr<OptixPointLight>           OptixPointLightPtr;
    typedef shared_ptr<OptixScene>                OptixScenePtr;
    typedef shared_ptr<OptixSensor>               OptixSensorPtr;
    typedef shared_ptr<OptixSphere>               OptixSpherePtr;
    typedef shared_ptr<OptixSpotLight>            OptixSpotLightPtr;
    typedef shared_ptr<OptixSubMesh>              OptixSubMeshPtr;
    typedef shared_ptr<OptixVisual>               OptixVisualPtr;
    typedef shared_ptr<OptixSceneStore>           OptixSceneStorePtr;
    typedef shared_ptr<OptixNodeStore>            OptixNodeStorePtr;
    typedef shared_ptr<OptixLightStore>           OptixLightStorePtr;
    typedef shared_ptr<OptixSensorStore>          OptixSensorStorePtr;
    typedef shared_ptr<OptixVisualStore>          OptixVisualStorePtr;
    typedef shared_ptr<OptixGeometryStore>        OptixGeometryStorePtr;
    typedef shared_ptr<OptixSubMeshStore>         OptixSubMeshStorePtr;
    typedef shared_ptr<OptixMaterialMap>          OptixMaterialMapPtr;
    typedef shared_ptr<OptixRenderTarget>         OptixRenderTargetPtr;
    typedef shared_ptr<OptixRenderTexture>        OptixRenderTexturePtr;
    typedef shared_ptr<OptixRenderWindow>         OptixRenderWindowPtr;
    typedef shared_ptr<OptixLightManager>         OptixLightManagerPtr;
    }
  }
}
#endif
