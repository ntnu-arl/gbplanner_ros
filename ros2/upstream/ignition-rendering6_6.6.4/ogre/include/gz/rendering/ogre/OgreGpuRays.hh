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

#ifndef GZ_RENDERING_OGRE_OGREGPURAYS_HH_
#define GZ_RENDERING_OGRE_OGREGPURAYS_HH_

#include <string>
#include <vector>
#include <memory>
#include <sstream>

#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/RenderTypes.hh"
#include "gz/rendering/base/BaseGpuRays.hh"
#include "gz/rendering/ogre/OgreConversions.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderTarget.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/OgreMaterial.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreSensor.hh"

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // Forward declaration
    class OgreGpuRaysPrivate;

    /** \class OgreGpuRays OgreGpuRays.hh\
     * rendering/ogre/OgreGpuRays.hh
    **/
    /// \brief Gpu Rays used to render depth data into an image buffer
    class IGNITION_RENDERING_OGRE_VISIBLE OgreGpuRays :
      public BaseGpuRays<OgreSensor>, public Ogre::RenderObjectListener
    {
      /// \brief Constructor
      protected: OgreGpuRays();

      /// \brief Destructor
      public: virtual ~OgreGpuRays();

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

      /// \internal
      /// \brief Implementation of Ogre::RenderObjectListener
      public: virtual void notifyRenderSingleObject(Ogre::Renderable *_rend,
              const Ogre::Pass *_p, const Ogre::AutoParamDataSource *_s,
              const Ogre::LightList *_ll, bool _supp) override;

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

      /// \brief Configure cameras.
      private: void ConfigureCameras();

      /// \brief Create a mesh.
      private: void CreateMesh();

      /// \brief Create a canvas.
      private: void CreateCanvas();

      /// \brief Create an ortho camera.
      private: void CreateOrthoCam();

      /// \brief Create an ortho camera.
      private: void CreateCamera();

      /// \brief Create the texture which is used to render gpu rays data.
      private: virtual void CreateGpuRaysTextures();

      /// \brief Builds scaled Orthogonal Matrix from parameters.
      /// \param[in] _left Left clip.
      /// \param[in] _right Right clip.
      /// \param[in] _bottom Bottom clip.
      /// \param[in] _top Top clip.
      /// \param[in] _near Near clip.
      /// \param[in] _far Far clip.
      /// \return The Scaled orthogonal Ogre::Matrix4
      private: Ogre::Matrix4 BuildScaledOrthoMatrix(const float _left,
          const float _right, const float _bottom, const float _top,
          const float _near, const float _far);

      private: void UpdateRenderTarget(Ogre::RenderTarget *_target,
                                       Ogre::Material *_material,
                                       Ogre::Camera *_cam,
                                       const bool _updateTex);

      /// \brief Get Cos Horz field-of-view
      /// \return 2 * atan(tan(this->hfov/2) / cos(this->vfov/2))
      private: virtual double CosHorzFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _chfov Cos Horz FOV
      private: virtual void SetCosHorzFOV(const double _chfov);

      /// \brief Get Cos Vert field-of-view
      /// \return 2 * atan(tan(this->vfov/2) / cos(this->hfov/2))
      private: virtual double CosVertFOV() const;

      /// \brief Set the Cos Horz FOV
      /// \param[in] _cvfov Cos Horz FOV
      private: virtual void SetCosVertFOV(const double _cvfov);

      /// \brief Get (horizontal_max_angle + horizontal_min_angle) * 0.5
      /// \return (horizontal_max_angle + horizontal_min_angle) * 0.5
      private: virtual double HorzHalfAngle() const;

      /// \brief Get (vertical_max_angle + vertical_min_angle) * 0.5
      /// \return (vertical_max_angle + vertical_min_angle) * 0.5
      private: virtual double VertHalfAngle() const;

      /// \brief Set the horizontal half angle
      /// \param[in] _angle horizontal half angle
      private: virtual void SetHorzHalfAngle(const double _angle);

      /// \brief Set the vertical half angle
      /// \param[in] _angle vertical half angle
      private: virtual void SetVertHalfAngle(const double _angle);

      /// \internal
      /// \brief Pointer to private data.
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      private: std::unique_ptr<OgreGpuRaysPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      private: friend class OgreScene;
    };
    }
  }
}
#endif
