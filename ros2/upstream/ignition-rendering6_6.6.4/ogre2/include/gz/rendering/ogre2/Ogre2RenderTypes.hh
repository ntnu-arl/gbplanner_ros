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
#ifndef GZ_RENDERING_OGRE2_OGRE2RENDERTYPES_HH_
#define GZ_RENDERING_OGRE2_OGRE2RENDERTYPES_HH_

#include <memory>

#include "gz/rendering/config.hh"
#include "gz/rendering/base/BaseRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class Ogre2ArrowVisual;
    class Ogre2AxisVisual;
    class Ogre2BoundingBoxCamera;
    class Ogre2Camera;
    class Ogre2Capsule;
    class Ogre2COMVisual;
    class Ogre2DepthCamera;
    class Ogre2DirectionalLight;
    class Ogre2Geometry;
    class Ogre2GizmoVisual;
    class Ogre2GpuRays;
    class Ogre2Grid;
    class Ogre2Heightmap;
    class Ogre2InertiaVisual;
    class Ogre2JointVisual;
    class Ogre2Light;
    class Ogre2LightVisual;
    class Ogre2LidarVisual;
    class Ogre2Marker;
    class Ogre2Material;
    class Ogre2Mesh;
    class Ogre2MeshFactory;
    class Ogre2Node;
    class Ogre2Object;
    class Ogre2ParticleEmitter;
    class Ogre2PointLight;
    class Ogre2RayQuery;
    class Ogre2RenderEngine;
    class Ogre2RenderTarget;
    class Ogre2RenderTargetMaterial;
    class Ogre2RenderTexture;
    class Ogre2RenderWindow;
    class Ogre2Scene;
    class Ogre2SegmentationCamera;
    class Ogre2Sensor;
    class Ogre2SpotLight;
    class Ogre2SubMesh;
    class Ogre2ThermalCamera;
    class Ogre2Visual;
    class Ogre2WireBox;

    typedef BaseGeometryStore<Ogre2Geometry>      Ogre2GeometryStore;
    typedef BaseLightStore<Ogre2Light>            Ogre2LightStore;
    typedef BaseNodeStore<Ogre2Node>              Ogre2NodeStore;
    typedef BaseSceneStore<Ogre2Scene>            Ogre2SceneStore;
    typedef BaseSensorStore<Ogre2Sensor>          Ogre2SensorStore;
    typedef BaseSubMeshStore<Ogre2SubMesh>        Ogre2SubMeshStore;
    typedef BaseVisualStore<Ogre2Visual>          Ogre2VisualStore;

    typedef BaseMaterialMap<Ogre2Material>        Ogre2MaterialMap;

    typedef shared_ptr<Ogre2ArrowVisual>          Ogre2ArrowVisualPtr;
    typedef shared_ptr<Ogre2AxisVisual>           Ogre2AxisVisualPtr;
    typedef shared_ptr<Ogre2BoundingBoxCamera>    Ogre2BoundingBoxCameraPtr;
    typedef shared_ptr<Ogre2Camera>               Ogre2CameraPtr;
    typedef shared_ptr<Ogre2Capsule>              Ogre2CapsulePtr;
    typedef shared_ptr<Ogre2COMVisual>            Ogre2COMVisualPtr;
    typedef shared_ptr<Ogre2DepthCamera>          Ogre2DepthCameraPtr;
    typedef shared_ptr<Ogre2DirectionalLight>     Ogre2DirectionalLightPtr;
    typedef shared_ptr<Ogre2Geometry>             Ogre2GeometryPtr;
    typedef shared_ptr<Ogre2GizmoVisual>          Ogre2GizmoVisualPtr;
    typedef shared_ptr<Ogre2GpuRays>              Ogre2GpuRaysPtr;
    typedef shared_ptr<Ogre2Grid>                 Ogre2GridPtr;
    typedef shared_ptr<Ogre2Heightmap>            Ogre2HeightmapPtr;
    typedef shared_ptr<Ogre2InertiaVisual>        Ogre2InertiaVisualPtr;
    typedef shared_ptr<Ogre2JointVisual>          Ogre2JointVisualPtr;
    typedef shared_ptr<Ogre2Light>                Ogre2LightPtr;
    typedef shared_ptr<Ogre2LightVisual>          Ogre2LightVisualPtr;
    typedef shared_ptr<Ogre2LidarVisual>          Ogre2LidarVisualPtr;
    typedef shared_ptr<Ogre2Marker>               Ogre2MarkerPtr;
    typedef shared_ptr<Ogre2Material>             Ogre2MaterialPtr;
    typedef shared_ptr<Ogre2Mesh>                 Ogre2MeshPtr;
    typedef shared_ptr<Ogre2MeshFactory>          Ogre2MeshFactoryPtr;
    typedef shared_ptr<Ogre2Node>                 Ogre2NodePtr;
    typedef shared_ptr<Ogre2Object>               Ogre2ObjectPtr;
    typedef shared_ptr<Ogre2ParticleEmitter>      Ogre2ParticleEmitterPtr;
    typedef shared_ptr<Ogre2PointLight>           Ogre2PointLightPtr;
    typedef shared_ptr<Ogre2RayQuery>             Ogre2RayQueryPtr;
    typedef shared_ptr<Ogre2RenderEngine>         Ogre2RenderEnginePtr;
    typedef shared_ptr<Ogre2RenderTarget>         Ogre2RenderTargetPtr;
    typedef shared_ptr<Ogre2RenderTexture>        Ogre2RenderTexturePtr;
    typedef shared_ptr<Ogre2RenderWindow>         Ogre2RenderWindowPtr;
    typedef shared_ptr<Ogre2Scene>                Ogre2ScenePtr;
    typedef shared_ptr<Ogre2SegmentationCamera>
      Ogre2SegmentationCameraPtr;
    typedef shared_ptr<Ogre2Sensor>               Ogre2SensorPtr;
    typedef shared_ptr<Ogre2SpotLight>            Ogre2SpotLightPtr;
    typedef shared_ptr<Ogre2SubMesh>              Ogre2SubMeshPtr;
    typedef shared_ptr<Ogre2ThermalCamera>        Ogre2ThermalCameraPtr;
    typedef shared_ptr<Ogre2Visual>               Ogre2VisualPtr;
    typedef shared_ptr<Ogre2WireBox>              Ogre2WireBoxPtr;

    typedef shared_ptr<Ogre2GeometryStore>        Ogre2GeometryStorePtr;
    typedef shared_ptr<Ogre2LightStore>           Ogre2LightStorePtr;
    typedef shared_ptr<Ogre2NodeStore>            Ogre2NodeStorePtr;
    typedef shared_ptr<Ogre2RenderTargetMaterial> Ogre2RenderTargetMaterialPtr;
    typedef shared_ptr<Ogre2SceneStore>           Ogre2SceneStorePtr;
    typedef shared_ptr<Ogre2SensorStore>          Ogre2SensorStorePtr;
    typedef shared_ptr<Ogre2SubMeshStore>         Ogre2SubMeshStorePtr;
    typedef shared_ptr<Ogre2VisualStore>          Ogre2VisualStorePtr;

    typedef shared_ptr<Ogre2MaterialMap>          Ogre2MaterialMapPtr;
    }
  }
}
#endif
