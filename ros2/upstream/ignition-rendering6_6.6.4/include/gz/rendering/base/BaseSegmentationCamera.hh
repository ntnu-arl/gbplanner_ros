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
#ifndef IGNITION_RENDERING_BASE_BASESEGMENTATIONCAMERA_HH_
#define IGNITION_RENDERING_BASE_BASESEGMENTATIONCAMERA_HH_

#include <string>

#include <ignition/common/Event.hh>

#include "ignition/rendering/base/BaseCamera.hh"
#include "ignition/rendering/SegmentationCamera.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

    template <class T>
    class BaseSegmentationCamera :
      public virtual SegmentationCamera,
      public virtual BaseCamera<T>,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseSegmentationCamera();

      /// \brief Destructor
      public: virtual ~BaseSegmentationCamera();

      // Documentation inherited
      public: virtual void CreateSegmentationTexture() override = 0;

      // Documentation inherited
      public: virtual uint8_t *SegmentationData() const override;

      // Documentation inherited
      public: virtual ignition::common::ConnectionPtr
        ConnectNewSegmentationFrame(
          std::function<void(const uint8_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) override;

      // Documentation inherited
      public: virtual void SetSegmentationType(
        SegmentationType _type) override;

      // Documentation inherited
      public: virtual SegmentationType Type() const override;

      // Documentation inherited
      public: virtual void EnableColoredMap(bool _enable) override;

      // Documentation inherited
      public: virtual bool IsColoredMap() const override;

      // Documentation inherited
      public: virtual void SetBackgroundColor(
        const math::Color &_color) override;

      // Documentation inherited
      public: virtual void SetBackgroundLabel(int _label) override;

      // Documentation inherited
      public: virtual const math::Color &BackgroundColor() const override;

      // Documentation inherited
      public: virtual int BackgroundLabel() const override;

      // Documentation inherited
      public: void LabelMapFromColoredBuffer(
                  uint8_t *_labelBuffer) const override = 0;

      /// \brief The buffer that contains segmentation data
      protected: uint8_t *segmentationData {nullptr};

      /// \brief The segmentation type
      protected: SegmentationType type {SegmentationType::ST_SEMANTIC};

      /// \brief Whether a colored map is being generated (true) or label ID map
      /// is being generated (false)
      protected: bool isColoredMap {false};

      /// \brief The color of objects that are considered background (i.e.,
      /// objects that have no label)
      protected: math::Color backgroundColor {0, 0, 0};

      /// \brief The label of background objects
      protected: int backgroundLabel {0};
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseSegmentationCamera<T>::BaseSegmentationCamera()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseSegmentationCamera<T>::~BaseSegmentationCamera()
    {
      if (this->segmentationData)
      {
        delete [] this->segmentationData;
        this->segmentationData = nullptr;
      }
    }

    //////////////////////////////////////////////////
    template <class T>
    uint8_t *BaseSegmentationCamera<T>::SegmentationData() const
    {
      return this->segmentationData;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::common::ConnectionPtr BaseSegmentationCamera<T>::
      ConnectNewSegmentationFrame(
          std::function<void(const uint8_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>)
    {
      return nullptr;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSegmentationCamera<T>::SetSegmentationType(SegmentationType _type)
    {
      this->type = _type;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSegmentationCamera<T>::EnableColoredMap(bool _enable)
    {
      this->isColoredMap = _enable;
    }

    //////////////////////////////////////////////////
    template <class T>
    SegmentationType BaseSegmentationCamera<T>::Type() const
    {
      return this->type;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseSegmentationCamera<T>::IsColoredMap() const
    {
      return this->isColoredMap;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSegmentationCamera<T>::SetBackgroundColor(
        const math::Color &_color)
    {
      this->backgroundColor = _color;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseSegmentationCamera<T>::SetBackgroundLabel(int _label)
    {
      this->backgroundLabel = _label;
    }

    //////////////////////////////////////////////////
    template <class T>
    const math::Color &BaseSegmentationCamera<T>::BackgroundColor() const
    {
      return this->backgroundColor;
    }

    //////////////////////////////////////////////////
    template <class T>
    int BaseSegmentationCamera<T>::BackgroundLabel() const
    {
      return this->backgroundLabel;
    }
  }
  }
}
#endif
