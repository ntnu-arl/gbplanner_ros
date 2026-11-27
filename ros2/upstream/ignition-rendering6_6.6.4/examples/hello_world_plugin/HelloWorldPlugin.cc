/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#include <gz/common/SingletonT.hh>

#include <gz/plugin/Register.hh>

#include "gz/rendering/RenderEnginePlugin.hh"
#include "gz/rendering/base/BaseRenderEngine.hh"
#include "gz/rendering/base/BaseRenderTypes.hh"

namespace mock
{
  /// \brief The render engine class which implements a render engine.
  class HelloWorldRenderEngine :
    public virtual gz::rendering::BaseRenderEngine,
    public gz::common::SingletonT<HelloWorldRenderEngine>
  {
    // Documentation Inherited.
    public: virtual bool IsEnabled() const override
    {
      return true;
    }

    // Documentation Inherited.
    public: virtual std::string Name() const override
    {
      return "HelloWorldPlugin";
    }

    // Documentation Inherited.
    protected: virtual bool LoadImpl(const std::map<std::string,
                   std::string> &_params) override
    {
      return true;
    }

    /// \brief Initialize the render engine.
    /// \return True if the operation is successful
    protected: virtual bool InitImpl() override
    {
      return true;
    }

    /// \brief Get a pointer to the list of scenes managed by the render
    /// engine.
    /// \return list of scenes
    protected: virtual gz::rendering::SceneStorePtr Scenes()
                     const override
    {
      return nullptr;
    }

    /// \brief Create a scene.
    /// \param[in] _id Unique scene Id
    /// \parampin] _name Name of scene
    protected: virtual gz::rendering::ScenePtr
                   CreateSceneImpl(unsigned int _id,
                   const std::string &_name) override
    {
      return nullptr;
    }

    /// \brief Singelton setup.
    private: friend class gz::common::SingletonT<HelloWorldRenderEngine>;
  };

  /// \brief Plugin for loading the HelloWorld render engine.
  class HelloWorldPlugin :
    public gz::rendering::RenderEnginePlugin
  {
    /// \brief Get the name of the render engine loaded by this plugin.
    /// \return Name of render engine
    public: std::string Name() const override
    {
      return HelloWorldRenderEngine::Instance()->Name();
    }

    /// \brief Get a pointer to the render engine loaded by this plugin.
    /// \return Render engine instance
    public: gz::rendering::RenderEngine *Engine() const override
    {
      return HelloWorldRenderEngine::Instance();
    }
  };
}

// Register this plugin
IGNITION_ADD_PLUGIN(mock::HelloWorldPlugin,
                    gz::rendering::RenderEnginePlugin)
