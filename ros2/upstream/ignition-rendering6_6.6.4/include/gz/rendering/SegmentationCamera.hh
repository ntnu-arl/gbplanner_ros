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
#ifndef IGNITION_RENDERING_SEGMENTATIONCAMERA_HH_
#define IGNITION_RENDERING_SEGMENTATIONCAMERA_HH_

#include <functional>
#include <string>

#include <ignition/common/Event.hh>
#include <ignition/math/Color.hh>

#include "ignition/rendering/Camera.hh"


namespace ignition
{
  namespace rendering
  {
    /// \brief Segmentation types for Semantic / Panpoptic segmentation
    enum class SegmentationType
    {
      /// \brief Pixels of same label from different items
      /// have the same color & id.
      ST_SEMANTIC = 0,

      /// \brief Pixels of same label from different items, have different
      /// color & id. 1 channel for label id & 2 channels for instance id
      ST_PANOPTIC = 1
    };

    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    /// \class SegmentationCamera SegmentationCamera.hh
    /// ignition/rendering/SegmentationCamera.hh
    /// \brief Poseable Segmentation camera used for rendering the scene graph.
    /// This camera is designed to produce segmentation data, instead of a 2D
    /// image.
    class IGNITION_RENDERING_VISIBLE SegmentationCamera :
      public virtual Camera
    {
      /// \brief Destructor
      public: virtual ~SegmentationCamera() { }

      /// \brief Create a texture which will hold the segmentation data
      public: virtual void CreateSegmentationTexture() = 0;

      /// \brief Get the segmentation image data.
      /// If the segmentation type is panoptic, the first two channels are the
      /// instance count, and the last channel is the label.
      /// \return The labels-buffer as an array of unsigned ints
      public: virtual uint8_t *SegmentationData() const = 0;

      /// \brief Connect to the new Segmentation image event
      /// \param[in] _subscriber Subscriber callback function.
      /// The callback function arguments are:
      /// <segmentation data, width, height, channels, format>
      /// \return Pointer to the new Connection. This must be kept in scope
      public: virtual ignition::common::ConnectionPtr
        ConnectNewSegmentationFrame(
          std::function<void(const uint8_t *, unsigned int, unsigned int,
          unsigned int, const std::string &)>  _subscriber) = 0;

      /// \brief Set Segmentation Type
      /// \param[in] _type Segmentation Type
      public: virtual void SetSegmentationType(SegmentationType _type) = 0;

      /// \brief Get Segmentation Type
      /// \return Segmentation Type
      public: virtual SegmentationType Type() const = 0;

      /// \brief Enable Color map mode to generate colored semantics
      /// \param[in] _enable True to generate colored map, False to generate
      /// label id map
      public: virtual void EnableColoredMap(bool _enable) = 0;

      /// \brief Check if color map mode is enabled
      /// \return True if colored map, False if label id map
      public: virtual bool IsColoredMap() const = 0;

      /// \brief Set color for background & unlabeled items in the colored map
      /// \param[in] _color Color of background & unlabeled items
      public: virtual void SetBackgroundColor(const math::Color &_color) = 0;

      /// \brief Set label for background & unlabeled items in the semantic map
      /// \param[in] _label label of background & unlabeled items
      public: virtual void SetBackgroundLabel(int _label) = 0;

      /// \brief Get color for background & unlabeled items in the colored map
      /// \return Color of background & unlabeled items
      public: virtual const math::Color &BackgroundColor() const = 0;

      /// \brief Get label for background & unlabeled items in the semantic map
      /// \return label of background & unlabeled items
      public: virtual int BackgroundLabel() const = 0;

      /// \brief Convert the colored map stored in the internal buffer to label
      /// IDs map, so users get both the colored map and the corresponding IDs
      /// map. This function must be called before the next render loop and
      /// the colored map mode must be enabeled
      /// \param[out] _labelBuffer A buffer that is populated with  the label
      /// IDs map data. This output buffer must be allocated with the same size
      /// before calling
      public: virtual void LabelMapFromColoredBuffer(
        uint8_t *_labelBuffer) const = 0;
    };
  }
  }
}
#endif
