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
#ifndef IGNITION_RENDERING_BASELIDARVISUAL_HH_
#define IGNITION_RENDERING_BASELIDARVISUAL_HH_

#include <vector>

#include "ignition/rendering/LidarVisual.hh"
#include "ignition/rendering/base/BaseObject.hh"
#include "ignition/rendering/base/BaseRenderTypes.hh"
#include "ignition/rendering/Scene.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \brief Base implementation of a Lidar Visual
    template <class T>
    class BaseLidarVisual :
      public virtual LidarVisual,
      public virtual T
    {
      // Documentation inherited
      protected: BaseLidarVisual();

      // Documentation inherited
      public: virtual ~BaseLidarVisual();

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      // Documentation inherited
      public: virtual void ClearPoints() override;

      // Documentation inherited
      public: virtual void SetPoints(
              const std::vector<double> &_points) override;

      // Documentation inherited
      public: virtual void SetPoints(const std::vector<double> &_points,
                            const std::vector<ignition::math::Color> &_colors)
                            override;

      // Documentation inherited
      public: virtual void Update() override;

      // Documentation inherited
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void SetMinVerticalAngle(
                  double _minVerticalAngle) override;

      // Documentation inherited
      public: virtual double MinVerticalAngle() const override;

      // Documentation inherited
      public: virtual void SetMaxVerticalAngle(
                  double _maxVerticalAngle) override;

      // Documentation inherited
      public: virtual double MaxVerticalAngle() const override;

      // Documentation inherited
      public: virtual void SetMinHorizontalAngle(
                  double _minHorizontalAngle) override;

      // Documentation inherited
      public: virtual double MinHorizontalAngle() const override;

      // Documentation inherited
      public: virtual void SetMaxHorizontalAngle(
                  double _maxHorizontalAngle) override;

      // Documentation inherited
      public: virtual double MaxHorizontalAngle() const override;

      // Documentation inherited
      public: virtual void SetVerticalRayCount(
                  unsigned int _verticalRayCount) override;

      // Documentation inherited
      public: virtual unsigned int VerticalRayCount() const override;

      // Documentation inherited
      public: virtual void SetHorizontalRayCount(
                  unsigned int _horizontalRayCount) override;

      // Documentation inherited
      public: virtual unsigned int HorizontalRayCount() const override;

      // Documentation inherited
      public: virtual void SetMinRange(double _minRange) override;

      // Documentation inherited
      public: virtual double MinRange() const override;

      // Documentation inherited
      public: virtual void SetMaxRange(double _maxRange) override;

      // Documentation inherited
      public: virtual double MaxRange() const override;

      // Documentation inherited
      public: virtual void SetOffset(
                  const ignition::math::Pose3d _offset) override;

      // Documentation inherited
      public: virtual ignition::math::Pose3d  Offset() const override;

      // Documentation inherited
      public: virtual unsigned int PointCount() const override;

      // Documentation inherited
      public: virtual std::vector<double> Points() const override;

      // Documentation inherited
      public: virtual void SetType(const LidarVisualType _type) override;

      // Documentation inherited
      public: virtual LidarVisualType Type() const override;

      // Documentation inherited
      public: virtual void SetSize(double _size) override;

      // Documentation inherited
      public: virtual double Size() const override;

      /// \brief Create predefined materials for lidar visual
      public: virtual void CreateMaterials();

      // Documentation inherited
      public: virtual void SetDisplayNonHitting(bool _display) override;

      // Documentation inherited
      public: virtual bool DisplayNonHitting() const override;

      /// \brief Vertical minimal angle
      protected: double minVerticalAngle = 0;

      /// \brief Vertical maximum angle
      protected: double maxVerticalAngle = 0;

      /// \brief Vertical laser count
      protected: unsigned int verticalCount = 1u;

      /// \brief Angle between two vertical rays
      protected: double verticalAngleStep = 0;

      /// \brief Horizontal minimal angle
      protected: double minHorizontalAngle = 0;

      /// \brief Horizontal maximum angle
      protected: double maxHorizontalAngle = 0;

      /// \brief Horizontal laser count
      protected: unsigned int horizontalCount = 1u;

      /// \brief Angle between two horizontal rays
      protected: double horizontalAngleStep = 0;

      /// \brief Minimum Range
      protected: double minRange = 0;

      /// \brief Maximum Range
      protected: double maxRange = 0;

      /// \brief Option to display non-hitting rays
      protected: bool displayNonHitting = true;

      /// \brief Offset of visual
      protected: ignition::math::Pose3d offset = ignition::math::Pose3d::Zero;

      /// \brief Type of lidar visualisation
      protected: LidarVisualType lidarVisualType =
                      LidarVisualType::LVT_TRIANGLE_STRIPS;

      /// \brief Size of lidar visualisation
      protected: double size = 1.0;
    };

    /////////////////////////////////////////////////
    // BaseLidarVisual
    /////////////////////////////////////////////////
    template <class T>
    BaseLidarVisual<T>::BaseLidarVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    BaseLidarVisual<T>::~BaseLidarVisual()
    {
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::PreRender()
    {
      T::PreRender();
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::Destroy()
    {
      T::Destroy();
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::ClearPoints()
    {
        // no op
    }

    /////////////////////////////////////////////////
    template <class T>
    unsigned int BaseLidarVisual<T>::PointCount() const
    {
        return 0u;
    }

    /////////////////////////////////////////////////
    template <class T>
    std::vector<double> BaseLidarVisual<T>::Points() const
    {
        std::vector<double> d;
        return d;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::Update()
    {
      // no op
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetPoints(const std::vector<double> &)
    {
      // no op
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetPoints(const std::vector<double> &,
                                const std::vector<ignition::math::Color> &)
    {
      // no op
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::Init()
    {
      T::Init();
      this->CreateMaterials();
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetMinVerticalAngle(
          double _minVerticalAngle)
    {
      this->minVerticalAngle = _minVerticalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseLidarVisual<T>::MinVerticalAngle() const
    {
      return this->minVerticalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetMaxVerticalAngle(
                  double _maxVerticalAngle)
    {
      this->maxVerticalAngle = _maxVerticalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseLidarVisual<T>::MaxVerticalAngle() const
    {
      return this->maxVerticalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetVerticalRayCount(
          unsigned int _verticalRayCount)
    {
      if (_verticalRayCount == 0)
      {
        ignwarn << "Cannot have zero vertical rays. Setting value to 1."
               << std::endl;
        this->verticalCount = 1;
      }
      else
      {
        this->verticalCount = _verticalRayCount;
      }
    }

    /////////////////////////////////////////////////
    template <class T>
    unsigned int BaseLidarVisual<T>::VerticalRayCount() const
    {
      return this->verticalCount;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetMinHorizontalAngle(
          double _minHorizontalAngle)
    {
      this->minHorizontalAngle = _minHorizontalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseLidarVisual<T>::MinHorizontalAngle() const
    {
      return this->minHorizontalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetMaxHorizontalAngle(
          double _maxHorizontalAngle)
    {
      this->maxHorizontalAngle = _maxHorizontalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseLidarVisual<T>::MaxHorizontalAngle() const
    {
      return this->maxHorizontalAngle;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetHorizontalRayCount(
          unsigned int _horizontalRayCount)
    {
      if (_horizontalRayCount == 0)
      {
        ignwarn << "Cannot have zero horizontal rays. Setting value to 1."
               << std::endl;
        this->horizontalCount = 1u;
      }
      else
      {
        this->horizontalCount = _horizontalRayCount;
      }
    }

    /////////////////////////////////////////////////
    template <class T>
    unsigned int BaseLidarVisual<T>::HorizontalRayCount() const
    {
      return this->horizontalCount;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetMinRange(double _minRange)
    {
      this->minRange = _minRange;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseLidarVisual<T>::MinRange() const
    {
      return this->minRange;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetMaxRange(double _maxRange)
    {
      this->maxRange = _maxRange;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseLidarVisual<T>::MaxRange() const
    {
      return this->maxRange;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetOffset(const ignition::math::Pose3d _offset)
    {
      this->offset = _offset;
    }

    /////////////////////////////////////////////////
    template <class T>
    ignition::math::Pose3d BaseLidarVisual<T>::Offset() const
    {
      return this->offset;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetType(const LidarVisualType _type)
    {
      this->lidarVisualType = _type;
    }

    /////////////////////////////////////////////////
    template <class T>
    LidarVisualType BaseLidarVisual<T>::Type() const
    {
      return this->lidarVisualType;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetSize(double _size)
    {
      this->size = _size;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseLidarVisual<T>::Size() const
    {
      return this->size;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::SetDisplayNonHitting(bool _display)
    {
      this->displayNonHitting = _display;
    }

    /////////////////////////////////////////////////
    template <class T>
    bool BaseLidarVisual<T>::DisplayNonHitting() const
    {
      return this->displayNonHitting;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseLidarVisual<T>::CreateMaterials()
    {
      MaterialPtr mtl;

      if (!this->Scene()->MaterialRegistered("Lidar/BlueStrips"))
      {
        mtl = this->Scene()->CreateMaterial("Lidar/BlueStrips");
        mtl->SetAmbient(0.0, 0.0, 1.0);
        mtl->SetDiffuse(0.0, 0.0, 1.0);
        mtl->SetEmissive(0.0, 0.0, 1.0);
        mtl->SetTransparency(0.4);
        mtl->SetCastShadows(false);
        mtl->SetReceiveShadows(false);
        mtl->SetLightingEnabled(false);
        mtl->SetMetalness(0.0f);
        mtl->SetReflectivity(0.0);
      }

      if (!this->Scene()->MaterialRegistered("Lidar/LightBlueStrips"))
      {
        mtl = this->Scene()->CreateMaterial("Lidar/LightBlueStrips");
        mtl->SetAmbient(0.0, 0.0, 1.0);
        mtl->SetDiffuse(0.0, 0.0, 1.0);
        mtl->SetEmissive(0.0, 0.0, 1.0);
        mtl->SetTransparency(0.8);
        mtl->SetCastShadows(false);
        mtl->SetReceiveShadows(false);
        mtl->SetLightingEnabled(false);
        mtl->SetMetalness(0.0f);
        mtl->SetReflectivity(0.0);
      }

      if (!this->Scene()->MaterialRegistered("Lidar/TransBlack"))
      {
        mtl = this->Scene()->CreateMaterial("Lidar/TransBlack");
        mtl->SetAmbient(0.0, 0.0, 0.0);
        mtl->SetDiffuse(0.0, 0.0, 0.0);
        mtl->SetEmissive(0.0, 0.0, 0.0);
        mtl->SetTransparency(0.4);
        mtl->SetCastShadows(false);
        mtl->SetReceiveShadows(false);
        mtl->SetLightingEnabled(false);
        mtl->SetMetalness(0.5f);
        mtl->SetReflectivity(0.2);
      }

      if (!this->Scene()->MaterialRegistered("Lidar/BlueRay"))
      {
        mtl = this->Scene()->CreateMaterial("Lidar/BlueRay");
        mtl->SetAmbient(0.0, 0.0, 1.0);
        mtl->SetDiffuse(0.0, 0.0, 1.0);
        mtl->SetEmissive(0.0, 0.0, 1.0);
        mtl->SetSpecular(0.0, 0.0, 1.0);
        mtl->SetTransparency(0.0);
        mtl->SetCastShadows(false);
        mtl->SetReceiveShadows(false);
        mtl->SetLightingEnabled(false);
        mtl->SetMetalness(0.1f);
        mtl->SetReflectivity(0.2);
      }
      return;
    }
    }
  }
}
#endif
