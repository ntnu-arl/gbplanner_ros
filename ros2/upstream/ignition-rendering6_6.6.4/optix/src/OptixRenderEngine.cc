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

#include <vector>

#include <gz/plugin/Register.hh>

#include "gz/rendering/RenderEngineManager.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixScene.hh"
#include "gz/rendering/optix/OptixStorage.hh"
#include "gz/rendering/optix/OptixRenderEngine.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////

const std::string OptixRenderEngine::PTX_PREFIX("cuda_compile_ptx_generated_");

const std::string OptixRenderEngine::PTX_SUFFIX(".cu.ptx");

//////////////////////////////////////////////////
OptixRenderEnginePlugin::OptixRenderEnginePlugin()
{
}

//////////////////////////////////////////////////
std::string OptixRenderEnginePlugin::Name() const
{
  return OptixRenderEngine::Instance()->Name();
}

//////////////////////////////////////////////////
RenderEngine *OptixRenderEnginePlugin::Engine() const
{
  return OptixRenderEngine::Instance();
}

//////////////////////////////////////////////////
OptixRenderEngine::OptixRenderEngine()
{
}

//////////////////////////////////////////////////
OptixRenderEngine::~OptixRenderEngine()
{
}

//////////////////////////////////////////////////
bool OptixRenderEngine::Fini()
{
  return true;
}

//////////////////////////////////////////////////
std::string OptixRenderEngine::Name() const
{
  return "optix";
}

//////////////////////////////////////////////////
std::string OptixRenderEngine::PtxFile(const std::string& _fileBase) const
{
  // TODO: actual implement system path system

  std::vector<std::string> folders;

  const char *env= std::getenv("IGN_RENDERING_RESOURCE_PATH");
  std::string resourcePath = (env) ? std::string(env) :
      IGN_RENDERING_RESOURCE_PATH;
  resourcePath = common::joinPaths(resourcePath, "optix");
  folders.push_back(resourcePath);

  for (auto &p : this->resourcePaths)
    folders.push_back(common::joinPaths(p, "optix"));

  std::string file = PTX_PREFIX + _fileBase + PTX_SUFFIX;

  for (auto folder : folders)
  {
    std::string uri = common::joinPaths(folder, file);

    if (common::exists(uri))
    {
      return uri;
    }
  }

  return file;
}

//////////////////////////////////////////////////
ScenePtr OptixRenderEngine::CreateSceneImpl(unsigned int _id,
    const std::string &_name)
{
  auto scene = OptixScenePtr(new OptixScene(_id, _name));
  this->scenes->Add(scene);
  return scene;
}

//////////////////////////////////////////////////
SceneStorePtr OptixRenderEngine::Scenes() const
{
  return this->scenes;
}

//////////////////////////////////////////////////
bool OptixRenderEngine::LoadImpl(
    const std::map<std::string, std::string> &/*_params*/)
{
  return true;
}

//////////////////////////////////////////////////
bool OptixRenderEngine::InitImpl()
{
  this->scenes = OptixSceneStorePtr(new OptixSceneStore);
  return true;
}

// Register this plugin
IGNITION_ADD_PLUGIN(OptixRenderEnginePlugin,
                    rendering::RenderEnginePlugin)
