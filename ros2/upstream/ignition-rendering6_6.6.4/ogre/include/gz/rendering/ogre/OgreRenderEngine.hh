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
#ifndef GZ_RENDERING_OGRE_OGRERENDERENGINE_HH_
#define GZ_RENDERING_OGRE_OGRERENDERENGINE_HH_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/SingletonT.hh>

#include "gz/rendering/RenderEnginePlugin.hh"
#include "gz/rendering/base/BaseRenderEngine.hh"
#include "gz/rendering/base/BaseRenderTypes.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/Export.hh"

namespace Ogre
{
  class LogManager;
  class Root;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class OgreRenderEnginePrivate;

    /// \brief Plugin for loading ogre render engine
    class IGNITION_RENDERING_OGRE_VISIBLE OgreRenderEnginePlugin :
      public RenderEnginePlugin
    {
      /// \brief Constructor
      public: OgreRenderEnginePlugin();

      /// \brief Destructor
      public: ~OgreRenderEnginePlugin() = default;

      /// \brief Get the name of the render engine loaded by this plugin.
      /// \return Name of render engine
      public: std::string Name() const;

      /// \brief Get a pointer to the render engine loaded by this plugin.
      /// \return Render engine instance
      public: RenderEngine *Engine() const;
    };

    class IGNITION_RENDERING_OGRE_VISIBLE OgreRenderEngine :
      public virtual BaseRenderEngine,
      public common::SingletonT<OgreRenderEngine>
    {
      /// \enum OgreRenderPathType
      /// \brief The type of rendering path used by the rendering engine.
      public: enum OgreRenderPathType
              {
                /// \brief No rendering is done.
                NONE = 0,
                /// \brief Most basic rendering, with least fidelity.
                VERTEX = 1,
                /// \brief Utilizes the RTT shader system.
                FORWARD = 2,
                /// \brief Utilizes deferred rendering. Best fidelity.
                DEFERRED = 3,
                /// \brief Count of the rendering path enums.
                RENDER_PATH_COUNT
              };

      /// \brief Constructor
      private: OgreRenderEngine();

      public: virtual ~OgreRenderEngine();

      public: virtual void Destroy() override;

      public: virtual bool IsEnabled() const override;

      public: virtual std::string Name() const override;

      public: OgreRenderPathType RenderPathType() const;

      public: void AddResourcePath(const std::string &_uri) override;

      public: virtual Ogre::Root *OgreRoot() const;

      public: std::string CreateRenderWindow(const std::string &_handle,
                  const unsigned int _width, const unsigned int _height,
                  const double _ratio, const unsigned int _antiAliasing);

      /// \brief Get a list of all supported FSAA levels for this render system
      /// \return a list of FSAA levels
      public: std::vector<unsigned int> FSAALevels() const;

      protected: virtual ScenePtr CreateSceneImpl(unsigned int _id,
                  const std::string &_name) override;

      protected: virtual SceneStorePtr Scenes() const override;

      /// \brief Engine implementation of Load function.
      /// \param[in] _params Parameters to be passed to the render engine.
      /// Current accepts the following parameters and values:
      /// "useCurrentGLContext" : "1" or "0". Use current OpenGL context for
      ///                                     rendering
      protected: virtual bool LoadImpl(
          const std::map<std::string, std::string> &_params) override;

      protected: virtual bool InitImpl() override;

      private: void LoadAttempt();

      private: void CreateLogger();

      private: void CreateContext();

      private: void CreateRoot();

      private: void CreateOverlay();

      private: void LoadPlugins();

      private: void CreateRenderSystem();

      private: void CreateResources();

      private: void CreateRenderWindow();

      private: void CheckCapabilities();

      private: void InitAttempt();

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
      /// \internal
      /// \brief Get a pointer to the Ogre overlay system.
      /// \return Pointer to the OGRE overlay system.
      public: Ogre::OverlaySystem *OverlaySystem() const;

      private: Ogre::OverlaySystem *ogreOverlaySystem = nullptr;

#endif
      private: OgreSceneStorePtr scenes;

      private: OgreRenderPathType renderPathType;

      private: Ogre::Root *ogreRoot = nullptr;

      private: Ogre::LogManager *ogreLogManager = nullptr;

      /// \brief Paths to ogre plugins
      private: std::vector<std::string> ogrePaths;

#if !defined(__APPLE__) && !defined(_WIN32)
      private: void *dummyDisplay = nullptr;

      private: void *dummyContext = nullptr;
#endif

      private: uint64_t dummyWindowId = 0u;

      /// \brief True to use the current opengl context
      private: bool useCurrentGLContext = false;

      private: std::unique_ptr<OgreRenderEnginePrivate> dataPtr;

      private: friend class common::SingletonT<OgreRenderEngine>;
    };
    }
  }
}
#endif
