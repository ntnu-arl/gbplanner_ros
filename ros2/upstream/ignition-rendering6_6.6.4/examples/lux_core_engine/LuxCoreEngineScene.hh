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
#ifndef IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINESCENE_HH_
#define IGNITION_RENDERING_LUXCOREENGINE_LUXCOREENGINESCENE_HH_

#include <luxcore/luxcore.h>

#include "ignition/rendering/base/BaseScene.hh"

#include "LuxCoreEngineCamera.hh"
#include "LuxCoreEngineLight.hh"
#include "LuxCoreEngineMaterial.hh"
#include "LuxCoreEngineMeshFactory.hh"
#include "LuxCoreEngineObject.hh"
#include "LuxCoreEngineRenderTypes.hh"
#include "LuxCoreEngineVisual.hh"

namespace ignition {
namespace rendering {
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

/// \brief LuxCore implementation of the scene class
class LuxCoreEngineScene : public BaseScene
{
  /// \brief Log message
  /// \param[in] _msg Message to log
  public: static void LogHandler(const char *msg);

  /// \brief Constructor
  /// \param[in] _id scene id
  /// \param[in] _name scene name
  protected: LuxCoreEngineScene(unsigned int _id, const std::string &_name);

  /// \brief Destructor
  public: virtual ~LuxCoreEngineScene();

  // Documentation inherited.
  public: virtual void Fini() override;

  // Documentation Inherited
  public: virtual RenderEngine *Engine() const override;

  // Documentation Inherited
  public: virtual VisualPtr RootVisual() const override;

  // Documentation Inherited
  public: virtual math::Color AmbientLight() const override;

  // Documentation Inherited
  public: virtual void SetAmbientLight(const math::Color &_color) override;

  // Documentation Inherited
  protected: virtual DirectionalLightPtr CreateDirectionalLightImpl(
                 unsigned int _id, const std::string &_name) override;

  // Documentation Inherited
  protected: virtual PointLightPtr CreatePointLightImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation Inherited
  protected: virtual SpotLightPtr CreateSpotLightImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation Inherited
  protected: virtual CameraPtr CreateCameraImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual DepthCameraPtr CreateDepthCameraImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation Inherited
  protected: virtual VisualPtr CreateVisualImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual ArrowVisualPtr CreateArrowVisualImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual AxisVisualPtr CreateAxisVisualImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual GeometryPtr CreateBoxImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual GeometryPtr CreateConeImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual GeometryPtr CreateCylinderImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual GeometryPtr CreatePlaneImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual GeometryPtr CreateSphereImpl(unsigned int _id,
                 const std::string &_name) override;

  /// \brief create mesh descriptor and implement mesh in the scene
  /// \param[in] _id mesh id
  /// \param[in] _name mesh name
  /// \param[in] _desc mesh descriptor
  protected: virtual MeshPtr CreateMeshImpl(unsigned int _id,
                 const std::string &_name,
                 const std::string &_meshName);

  // Documentation inherited
  protected: virtual MeshPtr CreateMeshImpl(unsigned int _id,
                 const std::string &_name,
                 const MeshDescriptor &_desc) override;

  // Documentation inherited
  protected: virtual CapsulePtr CreateCapsuleImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual GridPtr CreateGridImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual MarkerPtr CreateMarkerImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual LidarVisualPtr CreateLidarVisualImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual HeightmapPtr CreateHeightmapImpl(unsigned int _id,
                 const std::string &_name,
                 const HeightmapDescriptor &_desc) override;

  // Documentation inherited
  protected: virtual WireBoxPtr CreateWireBoxImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual MaterialPtr CreateMaterialImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual RenderTexturePtr CreateRenderTextureImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual RenderWindowPtr CreateRenderWindowImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual RayQueryPtr CreateRayQueryImpl(unsigned int _id,
                                           const std::string &_name) override;

  // Documentation inherited
  protected: virtual COMVisualPtr CreateCOMVisualImpl(unsigned int _id,
                                             const std::string &_name) override;

  // Documentation inherited
  protected: virtual InertiaVisualPtr CreateInertiaVisualImpl(unsigned int _id,
                 const std::string &_name) override;

  // Documentation inherited
  protected: virtual JointVisualPtr CreateJointVisualImpl(unsigned int _id,
                 const std::string &_name)override;

  // Documentation inherited
  protected: virtual LightStorePtr Lights() const override;

  // Documentation inherited
  protected: virtual SensorStorePtr Sensors() const override;

  // Documentation inherited
  protected: virtual VisualStorePtr Visuals() const override;

  // Documentation inherited
  protected: virtual MaterialMapPtr Materials() const override;

  // Documentation inherited
  protected: virtual bool LoadImpl() override;

  // Documentation inherited
  protected: virtual bool InitImpl() override;

  // Documentation inherited
  public: rendering::LightVisualPtr CreateLightVisualImpl(
            unsigned int _id, const std::string &_name) override;

  /// \brief Create mesh factory
  protected: void CreateMeshFactory();

  /// \brief Create lights, sensors, visuals and material stores
  protected: void CreateStores();

  /// \brief Initialize object by assigning variables and loading it
  /// \param[in] _object object pointer
  /// \param[in] _id  object id
  /// \param[in] _name object name
  /// \return true when initialized
  protected: bool InitObject(LuxCoreEngineObjectPtr _object, unsigned int _id,
                             const std::string &_name);

  /// \brief Get scene from the shared pointer
  /// \return return scene pointer
  private: LuxCoreEngineScenePtr SharedThis();

  /// \brief Get LuxCore scene
  /// \return lux scene
  public: luxcore::Scene *SceneLux();

  /// \brief Pointer to the LuxCore scene
  protected: luxcore::Scene *sceneLux{nullptr};

  /// \brief Pointer to the mesh factory
  protected: LuxCoreEngineMeshFactoryPtr meshFactory;

  /// \brief A list of materials used in the scene
  protected: LuxCoreEngineMaterialMapPtr materials;

  /// \brief A list of sensors in the scene
  protected: LuxCoreEngineSensorStorePtr sensors;

  /// \brief A list of visuals in the scene
  protected: LuxCoreEngineVisualStorePtr visuals;

  /// \brief A list of lights in the scene
  protected: LuxCoreEngineLightStorePtr lights;

  private: friend class LuxCoreEngineRenderEngine;
};

} // namespace IGNITION_RENDERING_VERSION_NAMESPACE
} // namespace rendering
} // namespace ignition

#endif
