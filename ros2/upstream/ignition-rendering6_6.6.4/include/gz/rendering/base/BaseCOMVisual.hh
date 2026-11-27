/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef IGNITION_RENDERING_BASE_BASECOMVISUAL_HH_
#define IGNITION_RENDERING_BASE_BASECOMVISUAL_HH_

#include <string>

#include "ignition/common/Console.hh"

#include "ignition/rendering/base/BaseObject.hh"
#include "ignition/rendering/base/BaseRenderTypes.hh"
#include "ignition/rendering/COMVisual.hh"
#include "ignition/rendering/Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Base implementation of an center of mass visual
    template <class T>
    class BaseCOMVisual :
      public virtual COMVisual,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseCOMVisual();

      /// \brief Destructor
      public: virtual ~BaseCOMVisual();

      // Documentation inherited.
      protected: virtual void Init() override;

      // Documentation inherited.
      protected: virtual void PreRender() override;

      // Documentation inherited.
      public: virtual void SetInertial(
                  const ignition::math::Inertiald &_inertial) override;

      // Documentation inherited.
      public: virtual void SetMass(double _mass) override;

      // Documentation inherited
      public: virtual double Mass() const override;

      // Documentation inherited
      public: virtual ignition::math::Pose3d InertiaPose() const override;

      // Documentation inherited
      public: virtual VisualPtr SphereVisual() const override;

      /// \brief Get the radius of the CoM sphere
      /// \return Radius of the CoM sphere
      protected: double SphereRadius() const;

      /// \brief Parent visual name.
      protected: std::string parentName = "";

      /// \brief Parent mass.
      protected: double mass = 1.0;

      /// \brief Inertia pose in parent frame.
      protected: ignition::math::Pose3d inertiaPose =
          ignition::math::Pose3d::Zero;

      /// \brief Flag to indicate parent properties have changed.
      protected: bool dirtyCOMVisual = false;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseCOMVisual<T>::BaseCOMVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseCOMVisual<T>::~BaseCOMVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseCOMVisual<T>::PreRender()
    {
      T::PreRender();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCOMVisual<T>::Init()
    {
      T::Init();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseCOMVisual<T>::SetInertial(
          const ignition::math::Inertiald &_inertial)
    {
      this->inertiaPose = _inertial.Pose();

      this->SetMass(_inertial.MassMatrix().Mass());
    }

    template <class T>
    void BaseCOMVisual<T>::SetMass(double _mass)
    {
      if (_mass <= 0)
      {
        // Unrealistic mass, load with default mass
        if (_mass < 0)
        {
          ignlog << "The parent " << this->parentName
              << " has unrealistic mass, "
              << "unable to visualize sphere of equivalent mass.\n";
        }
        else
        {
          ignlog << "The parent " << this->parentName
              << " is static or has mass of 0, "
              << "so a sphere of equivalent mass will not be shown.\n";
        }
        return;
      }

      this->mass = _mass;
      this->dirtyCOMVisual = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseCOMVisual<T>::Mass() const
    {
      return this->mass;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Pose3d BaseCOMVisual<T>::InertiaPose() const
    {
      return this->inertiaPose;
    }

    //////////////////////////////////////////////////
    template <class T>
    VisualPtr BaseCOMVisual<T>::SphereVisual() const
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseCOMVisual<T>::SphereRadius() const
    {
      // Compute radius of sphere with density of lead and equivalent mass.
      double sphereRadius;
      double densityLead = 11340;
      sphereRadius = cbrt((0.75 * this->Mass()) / (IGN_PI * densityLead));

      return sphereRadius;
    }
    }
  }
}
#endif
