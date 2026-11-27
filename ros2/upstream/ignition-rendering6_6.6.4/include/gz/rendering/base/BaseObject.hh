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
#ifndef GZ_RENDERING_BASE_BASEOBJECT_HH_
#define GZ_RENDERING_BASE_BASEOBJECT_HH_

#include <map>
#include <string>
#include <gz/common/SuppressWarning.hh>
#include "gz/rendering/Object.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    class IGNITION_RENDERING_VISIBLE BaseObject :
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      public virtual std::enable_shared_from_this<BaseObject>,
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
      public virtual Object
    {
      protected: BaseObject();

      public: virtual ~BaseObject();

      public: virtual unsigned int Id() const override;

      public: virtual std::string Name() const override;

      // Documentation inherited.
      public: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void PostRender() override;

      // Documentation inherited.
      public: virtual void Destroy() override;

      // TODO(anyone): make pure virtual
      protected: virtual void Load();

      // TODO(anyone): make pure virtual
      protected: virtual void Init();

      protected: unsigned int id;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      protected: std::string name;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}
#endif
