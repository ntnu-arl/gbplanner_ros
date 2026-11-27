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
#ifndef GZ_RENDERING_EXAMPLES_CUSTOM_SCENE_VIEWER_SCENEBUILDER_HH_
#define GZ_RENDERING_EXAMPLES_CUSTOM_SCENE_VIEWER_SCENEBUILDER_HH_

#include <string>
#include "TestTypes.hh"

namespace ignition
{
  namespace rendering
  {
    class SceneBuilder
    {
      public: SceneBuilder();

      public: virtual ~SceneBuilder();

      public: virtual void SetScenes(const SceneList &_scenes);

      public: virtual void SetCameras(const CameraList &_cameras);

      public: virtual void BuildScenes();

      public: virtual void ResetCameras();

      public: virtual void UpdateScenes();

      protected: virtual void ClearScene(ScenePtr _scene);

      protected: virtual void BuildScene(ScenePtr _scene);

      protected: virtual void ResetCamera(CameraPtr _camera);

      protected: virtual void UpdateScene(ScenePtr _scene);

      protected: virtual void RegisterMaterials();

      protected: virtual void RegisterMaterials(ScenePtr _scene);

      protected: SceneList scenes;

      protected: CameraList cameras;

      protected: static unsigned int tick;
    };

    class SimpleSceneBuilder :
      public SceneBuilder
    {
      public: SimpleSceneBuilder();

      public: virtual ~SimpleSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);

      protected: virtual void ResetCamera(CameraPtr _camera);

      protected: virtual void UpdateScene(ScenePtr _scene);

      protected: static const std::string LIGHT;

      protected: static const std::string SPHERE;

      protected: static const std::string PLANE;
    };

    class AllShapesSceneBuilder :
      public SimpleSceneBuilder
    {
      public: AllShapesSceneBuilder();

      public: virtual ~AllShapesSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);

      protected: static const std::string CONE;

      protected: static const std::string CYLINDER;

      protected: static const std::string BOX;
    };

    class TextureSceneBuilder :
      public AllShapesSceneBuilder
    {
      public: TextureSceneBuilder();

      public: virtual ~TextureSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);
    };

    class NormalMapSceneBuilder :
      public TextureSceneBuilder
    {
      public: NormalMapSceneBuilder();

      public: virtual ~NormalMapSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);
    };

    class ReflectionSceneBuilder :
      public TextureSceneBuilder
    {
      public: ReflectionSceneBuilder();

      public: virtual ~ReflectionSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);
    };

    class NormalReflectionSceneBuilder :
      public NormalMapSceneBuilder
    {
      public: NormalReflectionSceneBuilder();

      public: virtual ~NormalReflectionSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);
    };

    class TransparencySceneBuilder :
      public TextureSceneBuilder
    {
      public: TransparencySceneBuilder();

      public: virtual ~TransparencySceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);
    };

    class ShadowSceneBuilder :
      public SimpleSceneBuilder
    {
      public: ShadowSceneBuilder(unsigned int _n, double _dist = 1.0);

      public: virtual ~ShadowSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);

      protected: virtual void AddLight(PointLightPtr _light, ScenePtr _scene);

      protected: unsigned int n;

      protected: double dist;
    };

    class BoxSceneBuilder :
      public SceneBuilder
    {
      public: BoxSceneBuilder();

      public: virtual ~BoxSceneBuilder();

      protected: virtual void BuildScene(ScenePtr _scene);

      protected: virtual void ResetCamera(CameraPtr _camera);

      protected: virtual void UpdateScene(ScenePtr _scene);

      protected: static const std::string LIGHT;

      protected: static const std::string BOX;
    };
  }
}

#endif
