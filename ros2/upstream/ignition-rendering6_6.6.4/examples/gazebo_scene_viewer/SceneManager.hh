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
#ifndef IGNITION_RENDERING_EXAMPLES_SCENEMANAGER_HH
#define IGNITION_RENDERING_EXAMPLES_SCENEMANAGER_HH

#if __cplusplus > 201703L
    #include <filesystem>
    using fs = std::filesystem;
#else
    #include <experimental/filesystem>
    using namespace std;
    namespace fs = std::experimental::filesystem::v1;
#endif

#include <string>
#include <gz/common/SingletonT.hh>
#include "gz/rendering/RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    /// \class SceneManager SceneManager.hh ignition/rendering/SceneManager.hh
    /// \brief Manages a collection of scenes. The class provides a single
    /// interface for modifications, allowing multiple scenes to stay
    /// synchronized. This class currently consumes Gazebo-specific protobuf
    /// messages, but will be updated to be Gazebo-agnostic in the future.
    class SceneManager :
      public virtual common::SingletonT<SceneManager>
    {
      /// \brief Constructor
      public: SceneManager();

      /// \brief Destructor
      public: ~SceneManager();

      /// \brief Load resources
      public: void Load();

      /// \brief Initialize manager
      public: void Init();

      /// \brief Destroy manager
      public: void Fini();

      /// \brief Get number of managed scenes
      /// \return The number of managed scenes
      public: unsigned int SceneCount() const;

      /// \brief Determine if a scene with the given ID exists
      /// \param[in] _id ID of the scene in question
      /// \return True if the specified scene is exists
      public: bool HasScene(unsigned int _id) const;

      /// \brief Determine if a scene with the given name exists
      /// \param[in] _name Name of the scene in question
      /// \return True if the specified scene is exists
      public: bool HasScene(const std::string &_name) const;

      /// \brief Determine if the given scene exists
      /// \param[in] _scene Scene in question
      /// \return True if the specified scene is exists
      public: bool HasScene(ConstScenePtr _scene) const;

      /// \brief Get scene with the given ID. If no scene exists with the given
      /// ID, NULL will be returned.
      /// \param[in] _id ID of the desired scene
      /// \return The specified scene
      public: ScenePtr Scene(unsigned int _id) const;

      /// \brief Get scene with the given name. If no scene exists with the
      /// given name, NULL will be returned.
      /// \param[in] _id ID of the desired scene
      /// \return The specified scene
      public: ScenePtr Scene(const std::string &_name) const;

      /// \brief Get scene at the given index. If no scene exists at the
      /// given index, NULL will be returned.
      /// \param[in] _index Index of the desired scene
      /// \return The specified scene
      public: ScenePtr SceneAt(unsigned int _index) const;

      /// \brief Add the given scene. If the given scene has already been added
      /// then no work will be done.
      /// \param[in] _scene Scene to be added
      public: void AddScene(ScenePtr _scene);

      /// \brief Remove the scene with the given ID. If not scene exists with
      /// the given ID, then no work will be done.
      /// \param[in] _id ID of the scene to be removed
      /// \return The removed scene
      public: ScenePtr RemoveScene(unsigned int _id);

      /// \brief Remove the scene with the given name. If not scene exists with
      /// the given name, then no work will be done.
      /// \param[in] _name Name of the scene to be removed
      /// \return The removed scene
      public: ScenePtr RemoveScene(const std::string &_name);

      /// \brief Remove the given scene. If the given scene has not yet been
      /// added, then no work will be done.
      /// \param[in] _scene Scene to be removed
      /// \return The removed scene
      public: ScenePtr RemoveScene(ScenePtr _scene);

      /// \brief Remove the scene at the given index. If not scene exists at
      /// the given index, then no work will be done.
      /// \param[in] _index Index of the scene to be removed
      /// \return The removed scene
      public: ScenePtr RemoveSceneAt(unsigned int _index);

      /// \brief Remove all scenes
      public: void RemoveScenes();

      /// \brief Update all scenes
      public: void UpdateScenes();

      /// \brief Private implementation pointer
      private: class SceneManagerPrivate *pimpl;

      /// \brief Required SingletonT friendship
      private: friend class SingletonT<SceneManager>;
    };
  }
}
#endif
