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
#ifndef GZ_RENDERING_OBJECT_HH_
#define GZ_RENDERING_OBJECT_HH_

#include <string>
#include "gz/rendering/config.hh"
#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \class Object Object.hh ignition/rendering/Object.hh
    /// \brief Represents an object present in the scene graph. This includes
    /// sub-meshes, materials, render targets, as well as posable nodes.
    class IGNITION_RENDERING_VISIBLE Object
    {
      /// \brief Destructor
      public: virtual ~Object() { }

      /// \brief Get the object ID. This ID will be unique across all objects
      /// inside a given scene, but necessarily true for objects across
      /// different scenes.
      /// \return The object ID
      public: virtual unsigned int Id() const = 0;

      /// \brief Get the object name. This name will be unique across all
      /// objects inside a given scene, but necessarily true for objects across
      /// different scenes.
      /// \return The object name
      public: virtual std::string Name() const = 0;

      /// \brief Get the Scene that created this object.
      /// \return The parent scene
      public: virtual ScenePtr Scene() const = 0;

      /// \brief Prepare this object and any of its children for rendering.
      /// This should be called for each object in a scene just before
      /// rendering, which can be achieved by a single call to Scene::PreRender
      public: virtual void PreRender() = 0;

      /// \brief Post process this object and any of its children after
      /// rendering.
      public: virtual void PostRender() = 0;

      /// \brief Destroy any resources associated with this object. Invoking
      /// any other functions after destroying an object will result in
      /// undefined behavior.
      public: virtual void Destroy() = 0;
    };
    }
  }
}
#endif
