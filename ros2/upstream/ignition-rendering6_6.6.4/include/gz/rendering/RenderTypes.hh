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
#ifndef GZ_RENDERING_RENDERTYPES_HH_
#define GZ_RENDERING_RENDERTYPES_HH_

#include <memory>
#include <gz/rendering/config.hh>


/// \typedef IGN_VISIBILITY_ALL
/// \brief Render everything visibility mask.
#define IGN_VISIBILITY_ALL             0x0FFFFFFF

/// \typedef IGN_VISIBILITY_SELECTION
/// \brief Renders only objects that can be selected.
#define IGN_VISIBILITY_SELECTION       0x10000000

/// \typedef IGN_VISIBILITY_GUI
/// \brief Render GUI visuals mask.
#define IGN_VISIBILITY_GUI             0x00000001

/// \typedef IGN_VISIBILITY_SELECTABLE
/// \brief Render visuals that are selectable mask.
#define IGN_VISIBILITY_SELECTABLE      0x00000002

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    template <class T>
    using shared_ptr = std::shared_ptr<T>;

    class ArrowVisual;
    class AxisVisual;
    class BoundingBoxCamera;
    class Camera;
    class Capsule;
    class COMVisual;
    class DepthCamera;
    class DirectionalLight;
    class DistortionPass;
    class GaussianNoisePass;
    class Geometry;
    class GizmoVisual;
    class GpuRays;
    class Grid;
    class Heightmap;
    class Image;
    class InertiaVisual;
    class Light;
    class LightVisual;
    class JointVisual;
    class LidarVisual;
    class Light;
    class Marker;
    class Material;
    class Mesh;
    class Node;
    class Object;
    class ObjectFactory;
    class ParticleEmitter;
    class PointLight;
    class RayQuery;
    class RenderEngine;
    class RenderPass;
    class RenderPassSystem;
    class RenderTarget;
    class RenderTexture;
    class RenderWindow;
    class Scene;
    class SegmentationCamera;
    class Sensor;
    class ShaderParams;
    class SpotLight;
    class SubMesh;
    class Text;
    class ThermalCamera;
    class Visual;
    class WireBox;

    /// \typedef ArrowVisualPtr
    /// \brief Shared pointer to ArrowVisual
    typedef shared_ptr<ArrowVisual> ArrowVisualPtr;

    /// \typedef AxisVisualPtr
    /// \brief Shared pointer to AxisVisual
    typedef shared_ptr<AxisVisual> AxisVisualPtr;

    /// \typedef CameraPtr
    /// \brief Shared pointer to Camera
    typedef shared_ptr<Camera> CameraPtr;

    /// \typedef DepthCameraPtr
    /// \brief Shared pointer to DepthCamera
    typedef shared_ptr<DepthCamera> DepthCameraPtr;

    /// \typedef ThermalCameraPtr
    /// \brief Shared pointer to ThermalCamera
    typedef shared_ptr<ThermalCamera> ThermalCameraPtr;

    /// \typedef BoundingBoxCameraPtr
    /// \brief Shared pointer to BoundingBoxCamera
    typedef shared_ptr<BoundingBoxCamera> BoundingBoxCameraPtr;

    /// \typedef SegmentationCameraPtr
    /// \brief Shared pointer to Segmentation Camera
    typedef shared_ptr<SegmentationCamera> SegmentationCameraPtr;

    /// \typedef GpuRaysPtr
    /// \brief Shared pointer to GpuRays
    typedef shared_ptr<GpuRays> GpuRaysPtr;

    /// \typedef DirectionalLightPtr
    /// \brief Shared pointer to DirectionalLight
    typedef shared_ptr<DirectionalLight> DirectionalLightPtr;

    /// \typedef DistortionPassPtr
    /// \brief Shared pointer to DistortionPass
    typedef shared_ptr<DistortionPass> DistortionPassPtr;

    /// \typedef GaussianNoisePassPtr
    /// \brief Shared pointer to GaussianNoisePass
    typedef shared_ptr<GaussianNoisePass> GaussianNoisePassPtr;

    /// \typedef GeometryPtr
    /// \brief Shared pointer to Geometry
    typedef shared_ptr<Geometry> GeometryPtr;

    /// \typedef GizmoVisualPtr
    /// \brief Shared pointer to GizmoVisual
    typedef shared_ptr<GizmoVisual> GizmoVisualPtr;

    /// \typedef CapsulePtr
    /// \brief Shared pointer to Capsule
    typedef shared_ptr<Capsule> CapsulePtr;

    /// \typedef GridPtr
    /// \brief Shared pointer to Grid
    typedef shared_ptr<Grid> GridPtr;

    /// \typedef JointVisualPtr
    /// \brief Shared pointer to JointVisual
    typedef shared_ptr<JointVisual> JointVisualPtr;

    /// \typedef HeightmapPtr
    /// \brief Shared pointer to Heightmap
    typedef shared_ptr<Heightmap> HeightmapPtr;

    /// \typedef ImagePtr
    /// \brief Shared pointer to Image
    typedef shared_ptr<Image> ImagePtr;

    /// \typedef InertiaVisualPtr
    /// \def Shared pointer to InertiaVisual
    typedef shared_ptr<InertiaVisual> InertiaVisualPtr;

    /// \typedef LightPtr
    /// \brief Shared pointer to Light
    typedef shared_ptr<Light> LightPtr;

    /// \typedef COMVisualPtr
    /// \brief Shared pointer to COMVisual
    typedef shared_ptr<COMVisual> COMVisualPtr;

    /// \typedef LightVisualPtr
    /// \brief Shared pointer to Light
    typedef shared_ptr<LightVisual> LightVisualPtr;

    /// \typedef LidarVisualPtr
    /// \brief Shared pointer to LidarVisual
    typedef shared_ptr<LidarVisual> LidarVisualPtr;

    /// \typedef MaterialPtr
    /// \brief Shared pointer to Material
    typedef shared_ptr<Material> MaterialPtr;

    /// \typedef MarkerPtr
    /// \brief Shared pointer to Marker
    typedef shared_ptr<Marker> MarkerPtr;

    /// \typedef MeshPtr
    /// \brief Shared pointer to Mesh
    typedef shared_ptr<Mesh> MeshPtr;

    /// \typedef NodePtr
    /// \brief Shared pointer to Node
    typedef shared_ptr<Node> NodePtr;

    /// \typedef ObjectPtr
    /// \brief Shared pointer to Object
    typedef shared_ptr<Object> ObjectPtr;

    /// \typedef ObjectFactoryPtr
    /// \brief Shared pointer to ObjectFactory
    typedef shared_ptr<ObjectFactory> ObjectFactoryPtr;

    /// \typedef ParticleEmitterPtr
    /// \brief Shared pointer to ParticleEmitter
    typedef shared_ptr<ParticleEmitter> ParticleEmitterPtr;

    /// \typedef PointLightPtr
    /// \brief Shared pointer to PointLight
    typedef shared_ptr<PointLight> PointLightPtr;

    /// \typedef RayQueryPtr
    /// \brief Shared pointer to RayQuery
    typedef shared_ptr<RayQuery> RayQueryPtr;

    /// \typedef RenderPassPtr
    /// \brief Shared pointer to RenderPass
    typedef shared_ptr<RenderPass> RenderPassPtr;

    /// \typedef RenderPassSystemPtr
    /// \brief Shared pointer to RenderPassSystem
    typedef shared_ptr<RenderPassSystem> RenderPassSystemPtr;

    /// \typedef RenderTargetPtr
    /// \brief Shared pointer to RenderTarget
    typedef shared_ptr<RenderTarget> RenderTargetPtr;

    /// \typedef RenderTexturePtr
    /// \brief Shared pointer to RenderTexture
    typedef shared_ptr<RenderTexture> RenderTexturePtr;

    /// \typedef RenderWindowPtr
    /// \brief Shared pointer to RenderWindow
    typedef shared_ptr<RenderWindow> RenderWindowPtr;
    /// \typedef ScenePtr
    /// \brief Shared pointer to Scene
    typedef shared_ptr<Scene> ScenePtr;

    /// \typedef SensorPtr
    /// \brief Shared pointer to Sensor
    typedef shared_ptr<Sensor> SensorPtr;

    /// \brief Shared pointer to ShaderParams
    typedef shared_ptr<ShaderParams> ShaderParamsPtr;

    /// \typedef SpotLightPtr
    /// \brief Shared pointer to SpotLight
    typedef shared_ptr<SpotLight> SpotLightPtr;

    /// \typedef SubMeshPtr
    /// \brief Shared pointer to SubMesh
    typedef shared_ptr<SubMesh> SubMeshPtr;

    /// \typedef TextPtr
    /// \brief Shared pointer to Text
    typedef shared_ptr<Text> TextPtr;

    /// \typedef VisualPtr
    /// \brief Shared pointer to Visual
    typedef shared_ptr<Visual> VisualPtr;

    /// \typedef WireBoxPtr
    /// \brief Shared pointer to WireBox
    typedef shared_ptr<WireBox> WireBoxPtr;

    /// \typedef const ArrowVisualPtr
    /// \brief Shared pointer to const ArrowVisual
    typedef shared_ptr<const ArrowVisual> ConstArrowVisualPtr;

    /// \typedef const AxisVisualPtr
    /// \brief Shared pointer to const AxisVisual
    typedef shared_ptr<const AxisVisual> ConstAxisVisualPtr;

    /// \typedef const CameraPtr
    /// \brief Shared pointer to const Camera
    typedef shared_ptr<const Camera> ConstCameraPtr;

    /// \typedef const DepthCameraPtr
    /// \brief Shared pointer to const DepthCamera
    typedef shared_ptr<const DepthCamera> ConstDepthCameraPtr;

    /// \typedef const ThermalCameraPtr
    /// \brief Shared pointer to const ThermalCamera
    typedef shared_ptr<const ThermalCamera> ConstThermalCameraPtr;

    /// \typedef const BoundingBoxCameraPtr
    /// \brief Shared pointer to const BoundingBox Camera
    typedef shared_ptr<const BoundingBoxCamera> ConstBoundingBoxCameraPtr;

    /// \typedef const SegmentationCameraPtr
    /// \brief Shared pointer to const Segmentation Camera
    typedef shared_ptr<const SegmentationCamera> ConstSegmentationCameraPtr;

    /// \typedef const GpuRaysPtr
    /// \brief Shared pointer to const GpuRays
    typedef shared_ptr<const GpuRays> ConstGpuRaysPtr;

    /// \typedef const DirectionalLightPtr
    /// \brief Shared pointer to const DirectionalLight
    typedef shared_ptr<const DirectionalLight> ConstDirectionalLightPtr;

    /// \typedef const ConstGaussianNoisePass
    /// \brief Shared pointer to const GaussianNoisePass
    typedef shared_ptr<const GaussianNoisePass> ConstGaussianNoisePass;

    /// \typedef const GeometryPtr
    /// \brief Shared pointer to const Geometry
    typedef shared_ptr<const Geometry> ConstGeometryPtr;

    /// \typedef const GizmoVisualPtr
    /// \brief Shared pointer to const GizmoVisual
    typedef shared_ptr<const GizmoVisual> ConstGizmoVisualPtr;

    /// \typedef const JointVisualPtr
    /// \brief Shared pointer to const JointVisual
    typedef shared_ptr<const JointVisual> ConstJointVisualPtr;

    /// \typedef const HeightmapPtr
    /// \brief Shared pointer to const Heightmap
    typedef shared_ptr<const Heightmap> ConstHeightmapPtr;

    /// \typedef const ImagePtr
    /// \brief Shared pointer to const Image
    typedef shared_ptr<const Image> ConstImagePtr;

    /// \typedef const LightPtr
    /// \brief Shared pointer to const Light
    typedef shared_ptr<const Light> ConstLightPtr;

    /// \typedef const LidarVisualPtr
    /// \brief Shared pointer to const LidarVisual
    typedef shared_ptr<const LidarVisual> ConstLidarVisualPtr;

    /// \typedef const MaterialPtr
    /// \brief Shared pointer to const Material
    typedef shared_ptr<const Material> ConstMaterialPtr;

    /// \typedef const MeshPtr
    /// \brief Shared pointer to const Mesh
    typedef shared_ptr<const Mesh> ConstMeshPtr;

    /// \typedef const NodePtr
    /// \brief Shared pointer to const Node
    typedef shared_ptr<const Node> ConstNodePtr;

    /// \typedef const ObjectPtr
    /// \brief Shared pointer to const Object
    typedef shared_ptr<const Object> ConstObjectPtr;

    /// \typedef const ObjectFactoryPtr
    /// \brief Shared pointer to const ObjectFactory
    typedef shared_ptr<const ObjectFactory> ConstObjectFactoryPtr;

    /// \typedef const ParticleEmitterPtr
    /// \brief Shared pointer to const ParticleEmitter
    typedef shared_ptr<const ParticleEmitter> ConstParticleEmitterPtr;

    /// \typedef const PointLightPtr
    /// \brief Shared pointer to const PointLight
    typedef shared_ptr<const PointLight> ConstPointLightPtr;

    /// \typedef RayQueryPtr
    /// \brief Shared pointer to RayQuery
    typedef shared_ptr<const RayQuery> ConstRayQueryPtr;

    /// \typedef const RenderPassPtr
    /// \brief Shared pointer to const RenderPass
    typedef shared_ptr<const RenderPass> ConstRenderPassPtr;

    /// \typedef const RenderPassSystemPtr
    /// \brief Shared pointer to const RenderPassSystem
    typedef shared_ptr<const RenderPassSystem> ConstRenderPassSystemPtr;

    /// \typedef const RenderTargetPtr
    /// \brief Shared pointer to const RenderTarget
    typedef shared_ptr<const RenderTarget> ConstRenderTargetPtr;

    /// \typedef const RenderTexturePtr
    /// \brief Shared pointer to const RenderTexture
    typedef shared_ptr<const RenderTexture> ConstRenderTexturePtr;

    /// \typedef const RenderWindowPtr
    /// \brief Shared pointer to const RenderWindow
    typedef shared_ptr<const RenderWindow> ConstRenderWindowPtr;

    /// \typedef const ScenePtr
    /// \brief Shared pointer to const Scene
    typedef shared_ptr<const Scene> ConstScenePtr;

    /// \typedef const SensorPtr
    /// \brief Shared pointer to const Sensor
    typedef shared_ptr<const Sensor> ConstSensorPtr;

    /// \brief Shared pointer to const ShaderParams
    typedef shared_ptr<const ShaderParams> ConstShaderParamsPtr;

    /// \typedef const SpotLightPtr
    /// \brief Shared pointer to const SpotLight
    typedef shared_ptr<const SpotLight> ConstSpotLightPtr;

    /// \typedef const SubMeshPtr
    /// \brief Shared pointer to const SubMesh
    typedef shared_ptr<const SubMesh> ConstSubMeshPtr;

    /// \typedef const SubMeshPtr
    /// \brief Shared pointer to const SubMesh
    typedef shared_ptr<const Text> ConstTextPtr;

    /// \typedef const VisualPtr
    /// \brief Shared pointer to const Visual
    typedef shared_ptr<const Visual> ConstVisualPtr;
    }
  }
}
#endif
