/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_RENDERPASSSYSTEM_HH_
#define GZ_RENDERING_RENDERPASSSYSTEM_HH_

#include <map>
#include <memory>
#include <string>
#include <typeinfo>

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"
#include "gz/rendering/RenderPass.hh"
#include "gz/rendering/RenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class RenderPassSystemPrivate;

    /* \class RenderPassFactory RenderPassSystem.hh \
     * ignition/rendering/RenderPassSystem.hh
     */
    /// \brief A factory interface for creating render passes
    class IGNITION_RENDERING_VISIBLE RenderPassFactory
    {
      /// \brief Instantiate new render pass
      /// \return New render pass
      public: virtual RenderPass *New() const = 0;
    };

    /* \class RenderPassSystem RenderPassSystem.hh \
     * ignition/rendering/RenderPassSystem.hh
     */
    /// \brief A class for creating and managing render passes
    class IGNITION_RENDERING_VISIBLE RenderPassSystem
    {
      /// \brief Constructor
      public: RenderPassSystem();

      /// \brief Destructor
      public: virtual ~RenderPassSystem();

      /// \brief Templated function for creating render passes
      /// \return Pointer to the render pass created
      public: template<typename T> RenderPassPtr Create()
              {
                return this->CreateImpl(typeid(T).name());
              }

      /// \brief Register a render pass factory to the system
      /// \param[in] _type Render pass type, i.e. type id of render pass class
      /// \param[in] _factory Factory used to create the render pass
      public: static void Register(const std::string &_type,
          RenderPassFactory *_factory);

      /// \brief Implementation for creating render passes
      /// \param[in] _type Render pass type, i.e. type id of render pass class
      /// \return Pointer to the render pass created
      private: RenderPassPtr CreateImpl(const std::string &_type);

      /// \brief A map of render pass type id name to its factory class
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      private: static std::map<std::string, RenderPassFactory *> renderPassMap;

      /// \internal
      /// \brief Pointer to private data class
      private: std::unique_ptr<RenderPassSystemPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };

    /// \brief Render pass registration macro
    ///
    /// Use this macro to register render pass with the render pass factory.
    #define IGN_RENDERING_REGISTER_RENDER_PASS(classname, interface) \
    class classname##Factory : public gz::rendering::RenderPassFactory \
    { \
      public: classname##Factory() \
              { \
                gz::rendering::RenderPassSystem::Register( \
                    typeid(interface).name(), this); \
              } \
      public: RenderPass *New() const override \
              { \
                return new classname(); \
              } \
    }; \
    static classname##Factory global_##classname##Factory;
    }
  }
}
#endif
