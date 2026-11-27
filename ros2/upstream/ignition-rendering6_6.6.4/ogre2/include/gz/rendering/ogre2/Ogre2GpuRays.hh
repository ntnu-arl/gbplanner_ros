/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GZ_RENDERING_OGRE2_OGRE2GPURAYS_HH_
#define GZ_RENDERING_OGRE2_OGRE2GPURAYS_HH_

#include <string>
#include <memory>

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/base/BaseGpuRays.hh"
#include "gz/rendering/ogre2/Export.hh"
#include "gz/rendering/ogre2/Ogre2RenderTarget.hh"
#include "gz/rendering/ogre2/Ogre2RenderTypes.hh"
#include "gz/rendering/ogre2/Ogre2Scene.hh"
#include "gz/rendering/ogre2/Ogre2Sensor.hh"

#include "gz/common/Event.hh"
#include "gz/common/Console.hh"

namespace Ogre
{
  class Material;
  class RenderTarget;
  class Texture;
  class Viewport;
}

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class Ogre2GpuRaysPrivate;

    /// \brief Gpu Rays used to render range data into an image buffer
    /// The ogre2 implementation takes a 2 pass process to generate
    /// the final range data.
    /// 1st Pass: Creates a cubemap of range data. The cubemap is created from
    /// six cameras looking in all directions. Depending on the min/max angles
    /// specified, not all cameras need to be created. Internally in the 1st
    /// pass shaders, we reconstruct 3d viewspace pos from depth buffer data
    /// then convert them into ranges, i.e. length(pos.xyz).
    /// 2nd Pass: Samples range data from cubemap using predefined rays. The
    /// rays are generated based on the specified vertical and horizontal
    /// min/max angles and no. of samples. Each ray is a direction vector that
    /// is used to sample/lookup the range data stored in the faces of the
    /// cubemap.
    class IGNITION_RENDERING_OGRE2_VISIBLE Ogre2GpuRays :
      public BaseGpuRays<Ogre2Sensor>
    {
      /// \brief Constructor
      protected: Ogre2GpuRays();

      /// \brief Destructor
      public: virtual ~Ogre2GpuRays();

      // Documentation inherited
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void Destroy() override;

      /// \brief Create dummy render texture. Needed to satisfy inheritance
      public: virtual void CreateRenderTexture();

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void PostRender() override;

      // Documentation inherited
      public: virtual const float *Data() const override;

      // Documentation inherited.
      public: virtual void Copy(float *_data) override;

      // Documentation inherited.
      public: virtual common::ConnectionPtr ConnectNewGpuRaysFrame(
                  std::function<void(const float *_frame, unsigned int _width,
                  unsigned int _height, unsigned int _channels,
                  const std::string &_format)> _subscriber) override;

      // Documentation inherited.
      public: virtual RenderTargetPtr RenderTarget() const override;

      /// \brief Set the number of samples in the width and height for the
      /// first pass texture.
      /// \param[in] _w Number of samples in the horizontal sweep
      /// \param[in] _h Number of samples in the vertical sweep
      private: virtual void Set1stTextureSize(const unsigned int _w,
          const unsigned int _h = 1);

      /// \brief Set the number of samples in the width and height for the
      /// second pass texture.
      /// \param[in] _w Number of samples in the horizontal sweep
      /// \param[in] _h Number of samples in the vertical sweep
      private: virtual void SetRangeCount(const unsigned int _w,
          const unsigned int _h = 1);

      // Documentation inherited.
      private: virtual void Render() override;

      /// \brief Configure camera.
      private: void ConfigureCamera();

     /// \brief Create an ortho camera.
      private: void CreateCamera();

      /// \brief Create the texture which is used to render gpu rays data.
      private: virtual void CreateGpuRaysTextures();

      /// \brief Update the render targets in the 1st pass
      private: void UpdateRenderTarget1stPass();

      /// \brief Update the 2nd pass render target
      private: void UpdateRenderTarget2ndPass();

      /// \brief Create texture that store cubemap uv coordinates and
      /// cubemap face index data
      private: void CreateSampleTexture();

      /// \brief Set up 1st pass material, texture, and compositor
      private: void Setup1stPass();

      /// \brief Set up 2nd pass material, texture, and compositor
      private: void Setup2ndPass();

      /// \brief Helper function to convert a direction vector to the
      /// index number of a cubemap face and texture uv coordinates on that face
      /// \param[in] _v Direction vector
      /// \param[out] _faceIndex Index of face to sample
      /// \return Texture UV coordinates on the face indicated by _faceIndex
      private: math::Vector2d SampleCubemap(const math::Vector3d &_v,
          unsigned int &_faceIndex);

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<Ogre2GpuRaysPrivate> dataPtr;

      /// \brief Only the scene can create a GpuRays sensor
      private: friend class Ogre2Scene;
    };
    }
  }
}
#endif
