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
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINERENDERTYPES_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINERENDERTYPES_HH_

#include "ignition/rendering/base/BaseRenderTypes.hh"

namespace ignition {
namespace rendering {
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

class LuxCoreEngineCamera;
class LuxCoreEngineScene;
class LuxCoreEngineSensor;
class LuxCoreEngineNode;
class LuxCoreEngineObject;
class LuxCoreEngineMaterial;
class LuxCoreEngineRenderTarget;

class LuxCoreEngineLight;
class LuxCoreEngineDirectionalLight;
class LuxCoreEnginePointLight;

class LuxCoreEngineVisual;
class LuxCoreEngineGeometry;
class LuxCoreEngineMesh;
class LuxCoreEngineSubMesh;
class LuxCoreEngineMeshFactory;

typedef shared_ptr<LuxCoreEngineCamera> LuxCoreEngineCameraPtr;
typedef shared_ptr<LuxCoreEngineScene> LuxCoreEngineScenePtr;
typedef BaseSceneStore<LuxCoreEngineScene> LuxCoreEngineSceneStore;
typedef shared_ptr<LuxCoreEngineSceneStore> LuxCoreEngineSceneStorePtr;
typedef shared_ptr<LuxCoreEngineSensor> LuxCoreEngineSensorPtr;
typedef shared_ptr<LuxCoreEngineNode> LuxCoreEngineNodePtr;
typedef shared_ptr<LuxCoreEngineObject> LuxCoreEngineObjectPtr;
typedef shared_ptr<LuxCoreEngineMaterial> LuxCoreEngineMaterialPtr;
typedef BaseMaterialMap<LuxCoreEngineMaterial> LuxCoreEngineMaterialMap;
typedef shared_ptr<LuxCoreEngineMaterialMap> LuxCoreEngineMaterialMapPtr;
typedef BaseSensorStore<LuxCoreEngineSensor> LuxCoreEngineSensorStore;
typedef shared_ptr<LuxCoreEngineSensorStore> LuxCoreEngineSensorStorePtr;
typedef shared_ptr<LuxCoreEngineRenderTarget> LuxCoreEngineRenderTargetPtr;

typedef shared_ptr<LuxCoreEngineLight> LuxCoreEngineLightPtr;
typedef BaseLightStore<LuxCoreEngineLight> LuxCoreEngineLightStore;
typedef shared_ptr<LuxCoreEngineLightStore> LuxCoreEngineLightStorePtr;
typedef shared_ptr<LuxCoreEngineDirectionalLight>
    LuxCoreEngineDirectionalLightPtr;
typedef shared_ptr<LuxCoreEnginePointLight> LuxCoreEnginePointLightPtr;

typedef shared_ptr<LuxCoreEngineVisual> LuxCoreEngineVisualPtr;
typedef shared_ptr<LuxCoreEngineMesh> LuxCoreEngineMeshPtr;
typedef shared_ptr<LuxCoreEngineSubMesh> LuxCoreEngineSubMeshPtr;
typedef BaseSubMeshStore<LuxCoreEngineSubMesh> LuxCoreEngineSubMeshStore;
typedef shared_ptr<LuxCoreEngineSubMeshStore> LuxCoreEngineSubMeshStorePtr;
typedef shared_ptr<LuxCoreEngineGeometry> LuxCoreEngineGeometryPtr;
typedef shared_ptr<LuxCoreEngineMeshFactory> LuxCoreEngineMeshFactoryPtr;
typedef BaseVisualStore<LuxCoreEngineVisual> LuxCoreEngineVisualStore;
typedef shared_ptr<LuxCoreEngineVisualStore> LuxCoreEngineVisualStorePtr;
typedef BaseGeometryStore<LuxCoreEngineGeometry> LuxCoreEngineGeometryStore;
typedef shared_ptr<LuxCoreEngineGeometryStore> LuxCoreEngineGeometryStorePtr;

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition
#endif
