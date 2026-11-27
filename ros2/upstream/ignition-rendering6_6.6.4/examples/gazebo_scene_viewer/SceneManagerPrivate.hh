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
#ifndef IGNITION_RENDERING_EXAMPLES_SCENEMANAGERPRIVATE_HH
#define IGNITION_RENDERING_EXAMPLES_SCENEMANAGERPRIVATE_HH

#include <map>
#include <string>
#include <vector>
#include <memory>

#include <gz/common/Time.hh>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/ShaderType.hh"
#include "gazebo/transport/Node.hh"

namespace ignition
{
  namespace rendering
  {
    class CurrentSceneManager;

    class NewSceneManager;

    class SceneManagerPrivate
    {
      private: typedef std::map<unsigned int, std::string> RemovalMap;

      public: SceneManagerPrivate();

      public: ~SceneManagerPrivate();

      public: void Load();

      public: void Init();

      public: void Fini();

      public: unsigned int SceneCount() const;

      public: bool HasScene(unsigned int _id) const;

      public: bool HasScene(const std::string &_name) const;

      public: bool HasScene(ConstScenePtr _scene) const;

      public: ScenePtr Scene(unsigned int _id) const;

      public: ScenePtr Scene(const std::string &_name) const;

      public: ScenePtr SceneAt(unsigned int _index) const;

      public: void AddScene(ScenePtr _scene);

      public: ScenePtr RemoveScene(unsigned int _id);

      public: ScenePtr RemoveScene(const std::string &_name);

      public: ScenePtr RemoveScene(ScenePtr _scene);

      public: ScenePtr RemoveSceneAt(unsigned int _index);

      public: void RemoveScenes();

      public: void UpdateScenes();

      public: void OnRequest(::ConstRequestPtr &_requestMsg);

      public: void OnResponse(::ConstResponsePtr &_responseMsg);

      public: void OnSceneResponse(::ConstResponsePtr &_responseMsg);

      public: void OnRemovalResponse(::ConstResponsePtr &_responseMsg);

      public: void OnSceneUpdate(::ConstScenePtr _sceneMsg);

      public: void OnLightUpdate(::ConstLightPtr &_lightMsg);

      public: void OnModelUpdate(::ConstModelPtr &_modelMsg);

      public: void OnJointUpdate(::ConstJointPtr &_jointMsg);

      public: void OnVisualUpdate(::ConstVisualPtr &_visualMsg);

      public: void OnSensorUpdate(::ConstSensorPtr &_sensorMsg);

      public: void OnPoseUpdate(::ConstPosesStampedPtr &_posesMsg);

      private: void OnRemovalUpdate(const std::string &_name);

      private: void SendSceneRequest();

      private: void PromoteNewScenes();

      private: void DemoteCurrentScenes();

      private: std::unique_ptr<CurrentSceneManager> currentSceneManager;

      private: std::unique_ptr<NewSceneManager> newSceneManager;

      private: gazebo::transport::NodePtr transportNode;

      private: gazebo::transport::PublisherPtr requestPub;

      private: gazebo::transport::SubscriberPtr requestSub;

      private: gazebo::transport::SubscriberPtr responseSub;

      private: gazebo::transport::SubscriberPtr lightSub;

      private: gazebo::transport::SubscriberPtr modelSub;

      private: gazebo::transport::SubscriberPtr jointSub;

      private: gazebo::transport::SubscriberPtr visualSub;

      private: gazebo::transport::SubscriberPtr sensorSub;

      private: gazebo::transport::SubscriberPtr poseSub;

      private: gazebo::event::ConnectionPtr preRenderConn;

      private: int sceneRequestId;

      private: bool promotionNeeded;

      private: RemovalMap requestedRemovals;

      private: std::mutex generalMutex;

      private: std::mutex poseMutex;
    };

    class SubSceneManager
    {
      protected: typedef gazebo::msgs::Geometry::Type GeomType;

      protected: typedef void (SubSceneManager::*GeomFunc)
                     (const gazebo::msgs::Geometry&, VisualPtr);

      public: SubSceneManager();

      public: virtual ~SubSceneManager();

      public: virtual unsigned int SceneCount() const;

      public: virtual bool HasScene(unsigned int _id) const;

      public: virtual bool HasScene(const std::string &_name) const;

      public: virtual bool HasScene(ConstScenePtr _scene) const;

      public: virtual ScenePtr Scene(unsigned int _id) const;

      public: virtual ScenePtr Scene(const std::string &_name) const;

      public: virtual ScenePtr SceneAt(unsigned int _index) const;

      public: virtual void AddScene(ScenePtr _scene);

      public: virtual ScenePtr RemoveScene(unsigned int _id);

      public: virtual ScenePtr RemoveScene(const std::string &_name);

      public: virtual ScenePtr RemoveScene(ScenePtr _scene);

      public: virtual ScenePtr RemoveSceneAt(unsigned int _index);

      public: virtual void RemoveScenes();

      public: virtual void UpdateScenes();

      public: virtual void OnLightUpdate(::ConstLightPtr &_lightMsg);

      public: virtual void OnModelUpdate(::ConstModelPtr &_modelMsg);

      public: virtual void OnJointUpdate(::ConstJointPtr &_jointMsg);

      public: virtual void OnVisualUpdate(::ConstVisualPtr &_visualMsg);

      public: virtual void OnSensorUpdate(::ConstSensorPtr &_sensorMsg);

      public: virtual void OnPoseUpdate(::ConstPosesStampedPtr &_posesMsg) = 0;

      public: virtual void OnRemovalUpdate(const std::string &_name);

      public: virtual void Clear();

      protected: virtual void ProcessMessages();

      protected: virtual void ClearMessages();

      protected: virtual void ProcessLights() = 0;

      protected: virtual void ProcessLight(
          const gazebo::msgs::Light &_lightMsg);

      public: virtual void ProcessLight(
          const gazebo::msgs::Light &_lightMsg, VisualPtr _parent);

      protected: virtual void ProcessDirectionalLight(
                     const gazebo::msgs::Light &_lightMsg, VisualPtr _parent);

      protected: virtual void ProcessDirectionalLightImpl(
          const gazebo::msgs::Light &_lightMsg, DirectionalLightPtr _light);

      protected: virtual DirectionalLightPtr DirectionalLight(
                     const gazebo::msgs::Light &_lightMsg, VisualPtr _parent);

      protected: virtual DirectionalLightPtr CreateDirectionalLight(
                     const gazebo::msgs::Light &_lightMsg);

      protected: virtual void ProcessPointLight(
          const gazebo::msgs::Light &_lightMsg, VisualPtr _parent);

      protected: virtual void ProcessPointLightImpl(
          const gazebo::msgs::Light &_lightMsg, PointLightPtr _light);

      protected: virtual PointLightPtr PointLight(
                     const gazebo::msgs::Light &_lightMsg, VisualPtr _parent);

      protected: virtual PointLightPtr CreatePointLight(
                     const gazebo::msgs::Light &_lightMsg);

      protected: virtual void ProcessSpotLight(
          const gazebo::msgs::Light &_lightMsg, VisualPtr _parent);

      protected: virtual void ProcessSpotLightImpl(
          const gazebo::msgs::Light &_lightMsg, SpotLightPtr _light);

      protected: virtual SpotLightPtr SpotLight(
          const gazebo::msgs::Light &_lightMsg, VisualPtr _parent);

      protected: virtual SpotLightPtr CreateSpotLight(
                     const gazebo::msgs::Light &_lightMsg);

      protected: virtual void ProcessLightImpl(
          const gazebo::msgs::Light &_lightMsg, LightPtr _light);

      protected: virtual void ProcessSensors() = 0;

      protected: virtual void ProcessSensor(
          const gazebo::msgs::Sensor &_sensorMsg);

      protected: virtual void ProcessSensor(
          const gazebo::msgs::Sensor &_sensorMsg, VisualPtr _parent);

      protected: virtual void ProcessCamera(
          const gazebo::msgs::Sensor &_sensorMsg, VisualPtr _parent);

      protected: virtual CameraPtr Camera(
          const gazebo::msgs::Sensor &_sensorMsg, VisualPtr _parent);

      protected: virtual CameraPtr CreateCamera(
         const gazebo::msgs::Sensor &_sensorMsg);

      protected: virtual void ProcessModels() = 0;

      protected: virtual void ProcessModel(
          const gazebo::msgs::Model &_modelMsg);

      protected: virtual void ProcessModel(
          const gazebo::msgs::Model &_modelMsg, VisualPtr _parent);

      protected: virtual VisualPtr Model(const gazebo::msgs::Model &_modelMsg,
                     VisualPtr _parent);

      protected: virtual void ProcessJoints() = 0;

      protected: virtual void ProcessJoint(
          const gazebo::msgs::Joint &_jointMsg);

      protected: virtual void ProcessJoint(
          const gazebo::msgs::Joint &_jointMsg, VisualPtr _parent);

      protected: virtual VisualPtr Joint(const gazebo::msgs::Joint &_jointMsg,
                     VisualPtr _parent);

      protected: virtual void ProcessVisuals() = 0;

      protected: virtual void ProcessVisual(
          const gazebo::msgs::Visual &_visualMsg);

      protected: virtual void ProcessVisual(
          const gazebo::msgs::Visual &_visualMsg, VisualPtr _parent);

      protected: virtual VisualPtr Visual(
          const gazebo::msgs::Visual &_visualMsg, VisualPtr _parent);

      protected: virtual void ProcessLink(const gazebo::msgs::Link &_linkMsg,
                     VisualPtr _parent);

      protected: virtual VisualPtr Link(const gazebo::msgs::Link &_linkMsg,
                     VisualPtr _parent);

      protected: virtual VisualPtr Visual(bool _hasId, unsigned int _id,
                     const std::string &_name, VisualPtr _parent);

      protected: virtual VisualPtr CreateVisual(bool _hasId, unsigned int _id,
                     const std::string &_name);

      protected: virtual void ProcessGeometry(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessBox(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessCone(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessCapsule(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessCylinder(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessEmpty(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessMesh(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessPlane(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual void ProcessSphere(
          const gazebo::msgs::Geometry &_geometryMsg, VisualPtr _parent);

      protected: virtual MaterialPtr CreateMaterial(
                  const gazebo::msgs::Material &_materialMsg);

      protected: virtual void ProcessPoses() = 0;

      protected: virtual void ProcessPose(const gazebo::msgs::Pose &_poseMsg);

      protected: virtual void SetPose(NodePtr _node,
                     const gazebo::msgs::Pose &_poseMsg);

      protected: virtual void SetScale(VisualPtr _visual,
                     const gazebo::msgs::Vector3d &_scaleMsg);

      protected: virtual void ProcessRemovals() = 0;

      protected: virtual void ProcessRemoval(const std::string &_name);

      protected: virtual VisualPtr Parent(const std::string &_name);

      protected: static math::Color Convert(
          const gazebo::msgs::Color &_colorMsg);

      protected: static math::Pose3d Convert(
          const gazebo::msgs::Pose &_poseMsg);

      protected: static math::Vector3d Convert(
          const gazebo::msgs::Vector3d &_vecMsg);

      protected: static math::Quaterniond Convert(
                     const gazebo::msgs::Quaternion &_quatMsg);

      protected: static ShaderType Convert(
          gazebo::msgs::Material::ShaderType _type);

      private: void CreateGeometryFunctionMap();

      protected: ScenePtr activeScene;

      protected: std::vector<ScenePtr> scenes;

      protected: std::chrono::steady_clock::duration timePosesReceived;

      protected: std::vector<gazebo::msgs::Light> lightMsgs;

      protected: std::vector<gazebo::msgs::Model> modelMsgs;

      protected: std::vector<gazebo::msgs::Joint> jointMsgs;

      protected: std::vector<gazebo::msgs::Visual> visualMsgs;

      protected: std::vector<gazebo::msgs::Sensor> sensorMsgs;

      protected: std::vector<std::string> approvedRemovals;

      protected: std::map<GeomType, GeomFunc> geomFunctions;
    };

    class CurrentSceneManager :
      public virtual SubSceneManager
    {
      public: CurrentSceneManager();

      public: virtual ~CurrentSceneManager();

      public: virtual void OnPoseUpdate(::ConstPosesStampedPtr &_posesMsg);

      protected: virtual void ClearMessages();

      protected: virtual void ProcessLights();

      protected: virtual void ProcessModels();

      protected: virtual void ProcessJoints();

      protected: virtual void ProcessVisuals();

      protected: virtual void ProcessSensors();

      protected: virtual void ProcessPoses();

      protected: virtual void ProcessRemovals();

      private: std::map<std::string, gazebo::msgs::Pose> poseMsgs;
    };

    class NewSceneManager :
      public virtual SubSceneManager
    {
      public: NewSceneManager();

      public: virtual ~NewSceneManager();

      public: virtual void SetSceneData(const std::string &_data);

      public: virtual void OnPoseUpdate(::ConstPosesStampedPtr &_posesMsg);

      protected: virtual void ProcessMessages();

      protected: virtual void ClearMessages();

      protected: virtual void ProcessScene();

      protected: virtual void ProcessLights();

      protected: virtual void ProcessModels();

      protected: virtual void ProcessJoints();

      protected: virtual void ProcessVisuals();

      protected: virtual void ProcessSensors();

      protected: virtual void ProcessPoses();

      protected: virtual void ProcessPoses(
          const gazebo::msgs::PosesStamped &_posesMsg);

      protected: virtual void ProcessRemovals();

      protected: bool sceneReceived;

      protected: gazebo::msgs::Scene sceneMsg;

      private: std::vector<gazebo::msgs::PosesStamped> posesMsgs;
    };
  }
}
#endif
