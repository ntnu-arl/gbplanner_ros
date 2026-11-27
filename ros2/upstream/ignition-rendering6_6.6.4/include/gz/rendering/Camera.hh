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
#ifndef GZ_RENDERING_CAMERA_HH_
#define GZ_RENDERING_CAMERA_HH_

#include <string>

#include <gz/common/Event.hh>
#include <gz/math/Matrix4.hh>

#include "gz/rendering/config.hh"
#include "gz/rendering/Image.hh"
#include "gz/rendering/PixelFormat.hh"
#include "gz/rendering/Sensor.hh"
#include "gz/rendering/Scene.hh"


namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Enum for projection types
    enum IGNITION_RENDERING_VISIBLE CameraProjectionType
    {
      /// \brief Perspective projection
      CPT_PERSPECTIVE,
      /// \brief Orthographic projection
      CPT_ORTHOGRAPHIC
    };

    /// \class Camera Camera.hh ignition/rendering/Camera.hh
    /// \brief Posable camera used for rendering the scene graph
    class IGNITION_RENDERING_VISIBLE Camera :
      public virtual Sensor
    {
      /// \brief Callback function for new frame render event listeners
      public: typedef std::function<void(const void*, unsigned int,
          unsigned int, unsigned int, const std::string&)> NewFrameListener;

      /// \brief Deconstructor
      public: virtual ~Camera() { }

      /// \brief Get the image width in pixels
      /// \return The image width in pixels
      public: virtual unsigned int ImageWidth() const = 0;

      /// \brief Set the image width in pixels
      /// \param[in] _width New image width in pixels
      public: virtual void SetImageWidth(unsigned int _width) = 0;

      /// \brief Get the image height in pixels
      /// \return The image height in pixels
      public: virtual unsigned int ImageHeight() const = 0;

      /// \brief Set the image height in pixels
      /// \param[in] _height New image height in pixels
      public: virtual void SetImageHeight(unsigned int _height) = 0;

      /// \brief Get the image pixel format. If the image pixel format has not
      /// been set with a valid value, PF_UNKNOWN will be returned.
      /// \return The image pixel format
      public: virtual PixelFormat ImageFormat() const = 0;

      /// \brief Set the image pixel format
      /// \param[in] _format New image pixel format
      public: virtual void SetImageFormat(PixelFormat _format) = 0;

      /// \brief Get the total image memory size in bytes
      /// \return The image memory size in bytes
      public: virtual unsigned int ImageMemorySize() const = 0;

      /// \brief Get the camera's horizontal field-of-view
      /// \return Angle containing the camera's horizontal field-of-view
      public: virtual math::Angle HFOV() const = 0;

      /// \brief Set the camera's horizontal field-of-view
      /// \param[in] _hfov Desired horizontal field-of-view
      public: virtual void SetHFOV(const math::Angle &_hfov) = 0;

      /// \brief Get the camera's aspect ratio
      /// \return The camera's aspect ratio
      public: virtual double AspectRatio() const = 0;

      /// \brief Set the camera's aspect ratio. This value determines the
      /// cameras vertical field-of-view. It is often the \code image_height /
      /// image_width \endcode but this is not necessarily true.
      /// \return The camera's aspect ratio
      public: virtual void SetAspectRatio(const double _ratio) = 0;

      // TODO(anyone): add auto-aspect ratio

      /// \brief Get the level of anti-aliasing used during rendering
      /// \return The level of anti-aliasing used during rendering
      public: virtual unsigned int AntiAliasing() const = 0;

      /// \brief Set the level of anti-aliasing used during rendering. If a
      /// value of 0 is given, no anti-aliasing will be performed. Higher values
      /// can significantly slow-down rendering times, depending on the
      /// underlying render engine.
      /// \param[in] _aa Level of anti-aliasing used during rendering
      public: virtual void SetAntiAliasing(const unsigned int _aa) = 0;

      /// \brief Get the camera's far clipping plane distance
      /// \return Far clipping plane distance
      public: virtual double FarClipPlane() const = 0;

      /// \brief Set the camera's far clipping plane distance
      /// \param[in] _far Far clipping plane distance
      public: virtual void SetFarClipPlane(const double _far) = 0;

      /// \brief Get the camera's near clipping plane distance
      /// \return Near clipping plane distance
      public: virtual double NearClipPlane() const = 0;

      /// \brief Set the camera's near clipping plane distance
      /// \param[in] _near Near clipping plane distance
      public: virtual void SetNearClipPlane(const double _near) = 0;

      /// \brief Renders the current scene using this camera. This function
      /// assumes PreRender() has already been called on the parent Scene,
      /// allowing the camera and the scene itself to prepare for rendering.
      public: virtual void Render() = 0;

      /// \brief Preforms any necessary final rendering work. Once rendering is
      /// complete the camera will alert any listeners of the new frame event.
      /// This function should only be called after a call to Render has
      /// successfully been executed.
      public: virtual void PostRender() = 0;

      /// \brief Get the visual for a given mouse position
      /// param[in] _mousePos mouse position
      //  \return visual for that position, null if no visual was found
      public: virtual VisualPtr VisualAt(const gz::math::Vector2i
                  &_mousePos) = 0;

      /// \brief Renders a new frame.
      /// This is a convenience function for single-camera scenes. It wraps the
      /// pre-render, render, and post-render into a single
      /// function. This should be used in applications with multiple cameras
      /// or multiple consumers of a single camera's images.
      public: virtual void Update() = 0;

      /// \brief Created an empty image buffer for capturing images. The
      /// resulting image will have sufficient memory allocated for subsequent
      /// calls to this camera's Capture function. However, any changes to this
      /// cameras properties may invalidate the condition.
      /// \return A newly allocated Image for storing this cameras images
      public: virtual Image CreateImage() const = 0;

      /// \brief Renders a new frame and writes the results to the given image.
      /// This is a convenience function for single-camera scenes. It wraps the
      /// pre-render, render, post-render, and get-image calls into a single
      /// function. This should be used in applications with multiple cameras
      /// or multiple consumers of a single camera's images.
      /// \param[out] _image Output image buffer
      public: virtual void Capture(Image &_image) = 0;

      /// \brief Writes the last rendered image to the given image buffer. This
      /// function can be called multiple times after PostRender has been
      /// called, without rendering the scene again. Calling this function
      /// before a single image has been rendered will have undefined behavior.
      /// \param[out] _image Output image buffer
      public: virtual void Copy(Image &_image) const = 0;

      /// \brief Writes the previously rendered frame to a file. This function
      /// can be called multiple times after PostRender has been called,
      /// without rendering the scene again. Calling this function before a
      /// single image has been rendered will have undefined behavior.
      /// \param[in] _name Name of the output file
      public: virtual bool SaveFrame(const std::string &_name) = 0;

      /// \brief Subscribes a new listener to this camera's new frame event
      /// \param[in] _listener New camera listener callback
      public: virtual common::ConnectionPtr ConnectNewImageFrame(
                  NewFrameListener _listener) = 0;

      /// \brief Create a render window.
      /// \return A pointer to the render window.
      public: virtual RenderWindowPtr CreateRenderWindow() = 0;

      /// \brief Get the projection matrix for this camera
      /// \return Camera projection matrix
      public: virtual math::Matrix4d ProjectionMatrix() const = 0;

      /// \brief Get the view matrix for this camera
      /// \return Camera view matrix
      public: virtual math::Matrix4d ViewMatrix() const = 0;

      /// \brief Set the projection matrix for this camera. This overrides
      /// the standard projection matrix computed based on camera parameters.
      /// \param[in] _matrix Camera projection matrix
      public: virtual void SetProjectionMatrix(const math::Matrix4d &_matrix)
          = 0;

      /// \brief Get the projection type for this camera
      /// \return Camera projection type
      public: virtual CameraProjectionType ProjectionType() const = 0;

      /// \brief Set the projection type for this camera
      /// This changes the projection matrix of the camera based on the camera
      /// projection type. A custom projection matrix can be specified via
      /// `SetProjectionMatrix` to override the provided one. To disable the
      /// custom projection matrix, just call this function again with the
      /// desired projection type.
      /// \param[in] _type Camera projection type
      /// \sa SetProjectionMatrix
      public: virtual void SetProjectionType(CameraProjectionType _type) = 0;

      /// \brief Project point in 3d world space to 2d screen space
      /// \param[in] _pt Point in 3d world space
      /// \return Point in 2d screen space
      public: virtual math::Vector2i Project(const math::Vector3d &_pt) const
           = 0;

      /// \brief Set a node for camera to track. The camera will automatically
      /// change its orientation to face the target being tracked. If null is
      /// specified, tracking is disabled. In contrast to SetFollowTarget
      /// the camera does not change its position when tracking is enabled.
      /// \param[in] _target Target node to track
      /// \param[in] _offset Track a point that is at an offset relative
      /// to target.
      /// \param[in] _worldFrame If true, the offset point to track will be
      /// treated in world frame and its position relative to the target
      /// node remains fixed regardless of the target node's rotation. Default
      /// is false, which means the camera tracks the point in target node's
      /// local frame.
      public: virtual void SetTrackTarget(const NodePtr &_target,
                  const math::Vector3d &_offset = math::Vector3d::Zero,
                  const bool _worldFrame = false) = 0;

      /// \brief Get the target node being tracked
      /// \return Target node being tracked.
      public: virtual NodePtr TrackTarget() const = 0;

      /// \brief Set track offset. Camera will track a point that's at an
      /// offset from the target node. The offset will be in the frame
      /// that is specified at the time the track target is set.
      /// \param[in] _offset Point offset to track
      public: virtual void SetTrackOffset(const math::Vector3d &_offset) = 0;

      /// \brief Get the track offset vector in the frame specified at the time
      /// the track target is set.
      /// \return Point offset from target.
      public: virtual math::Vector3d TrackOffset() const = 0;

      /// \brief Set track P Gain. Determines how fast the camera rotates
      /// to look at the target node. Valid range: [0-1]
      /// \param[in] _pGain P gain for camera tracking
      public: virtual void SetTrackPGain(const double _pGain) = 0;

      /// \brief Get the camera track rotation P gain.
      /// \return P gain for camera tracking
      public: virtual double TrackPGain() const = 0;

      /// \brief Set a node for camera to follow. The camera will automatically
      /// update its position to keep itself at the specified offset distance
      /// from the target being followed. If null is specified, camera follow is
      /// disabled. In contrast to SetTrackTarget, the camera does not change
      /// its orientation when following is enabled.
      /// \param[in] _target Target node to follow
      /// \param[in] _offset Tether the camera at an offset distance from the
      /// target node.
      /// \param[in] _worldFrame True to follow the target node at a
      /// distance that's fixed in world frame. Default is false which means
      /// the camera follows at fixed distance in target node's local frame.
      public: virtual void SetFollowTarget(const NodePtr &_target,
                  const math::Vector3d &_offset = math::Vector3d::Zero,
                  const bool _worldFrame = false) = 0;

      /// \brief Get the target node being followed
      /// \return Target node being tracked.
      public: virtual NodePtr FollowTarget() const = 0;

      /// \brief Set offset of camera from target node being followed. The
      /// offset will be in the frame that is specified at the time the follow
      /// target is set.
      /// \param[in] _offset Offset distance from target node.
      public: virtual void SetFollowOffset(const math::Vector3d &_offset) = 0;

      /// \brief Get the follow offset vector in the frame specified at the
      /// time the follow target is set.
      /// \return Offset of camera from target.
      public: virtual math::Vector3d FollowOffset() const = 0;

      /// \brief Set follow P Gain. Determines how fast the camera moves
      /// to follow the target node. Valid range: [0-1]
      /// \param[in] _pGain P gain for camera following
      public: virtual void SetFollowPGain(const double _pGain) = 0;

      /// \brief Get the camera follow movement P gain.
      /// \return P gain for camera following
      public: virtual double FollowPGain() const = 0;

      /// \brief Set a material that the camera should see on all objects
      /// \param[in] _material a material instance
      public: virtual void SetMaterial(const MaterialPtr &_material) = 0;

      /// \brief Get the OpenGL texture id associated with the render texture
      /// used by this camera. A valid id is returned only if the underlying
      /// render engine is OpenGL based.
      /// \return Texture Id of type GLuint.
      public: virtual unsigned int RenderTextureGLId() const = 0;

      /// \brief Add a render pass to the camera
      /// \param[in] _pass New render pass to add
      public: virtual void AddRenderPass(const RenderPassPtr &_pass) = 0;

      /// \brief Remove a render pass from the camera
      /// \param[in] _pass render pass to remove
      public: virtual void RemoveRenderPass(const RenderPassPtr &_pass) = 0;

      /// \brief Get the number of render passes applied to the camera
      /// \return Number of render passes applied
      public: virtual unsigned int RenderPassCount() const = 0;

      /// \brief Get a render passes by index
      /// \return Render pass at the specified index
      public: virtual RenderPassPtr RenderPassByIndex(unsigned int _index)
          const = 0;

      /// \internal
      /// \brief Notify that shadows are dirty and need to be regenerated
      public: virtual void SetShadowsDirty() = 0;
    };
    }
  }
}
#endif
