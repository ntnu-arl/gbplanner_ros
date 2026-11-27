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
#ifndef IGNITION_RENDERING_PARTICLEEMITTER_HH_
#define IGNITION_RENDERING_PARTICLEEMITTER_HH_

#include <string>
#include "ignition/math/Color.hh"
#include "ignition/math/Pose3.hh"
#include "ignition/math/Vector3.hh"
#include "ignition/rendering/config.hh"
#include "ignition/rendering/Export.hh"
#include "ignition/rendering/RenderTypes.hh"
#include "ignition/rendering/Visual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Enum for emitter types.
    enum IGNITION_RENDERING_VISIBLE EmitterType
    {
      /// \brief Point emitter.
      EM_POINT            = 0,

      /// \brief Box emitter.
      EM_BOX              = 1,

      /// \brief Cylinder emitter.
      EM_CYLINDER         = 2,

      /// \brief Ellipsoid emitter.
      EM_ELLIPSOID        = 3,

      /// \brief Total number of emitters (keep always at the end).
      EM_NUM_EMITTERS     = 4,
    };

    /// \class ParticleEmitter ParticleEmitter.hh
    /// ignition/rendering/ParticleEmitter.hh
    //
    /// \brief Class to manage a particle emitter.
    class IGNITION_RENDERING_VISIBLE ParticleEmitter :
      public virtual Visual
    {
      /// \brief Destructor
      public: virtual ~ParticleEmitter() {}

      /// \brief \brief Get the emitter type.
      /// \return Emitter type.
      /// \sa EmitterType.
      /// \sa SetType.
      public: virtual EmitterType Type() const = 0;

      /// \brief \brief Set the emitter type.
      /// Default value is EM_POINT.
      /// \param[in] _type Emitter type.
      /// \sa EmitterType.
      /// \sa Type.
      public: virtual void SetType(const EmitterType _type) = 0;

      /// \brief Get the size of the emitter where the particles are sampled.
      /// \return The emitter size. See SetEmitterSize() for the interpretation
      /// of the dimensions depending on the emitter type.
      /// \sa SetEmitterSize
      public: virtual ignition::math::Vector3d EmitterSize() const = 0;

      /// \brief Set the size of the emitter where the particles are sampled.
      /// Default value is (1, 1, 1).
      /// Note that the interpretation of the emitter area varies depending on
      /// the emmiter type:
      ///   - EM_POINT: The area is ignored.
      ///   - EM_BOX: The area is interpreted as width X height X depth.
      ///   - EM_CYLINDER: The area is interpreted as the bounding box of the
      ///                  cilinder. The cylinder is oriented along the Z-axis.
      ///   - EM_ELLIPSOID: The area is interpreted as the bounding box of an
      ///                   ellipsoid shaped area, i.e. a sphere or
      ///                   squashed-sphere area. The parameters are again
      ///                   identical to EM_BOX, except that the dimensions
      ///                   describe the widest points along each of the axes.
      /// \param[in] _size Size of the emitter (width, height, depth).
      /// \sa EmitterSize
      public: virtual void SetEmitterSize(
                  const ignition::math::Vector3d &_size) = 0;

      /// \brief Get how many particles per second should be emitted.
      /// \return Particles per second.
      /// \sa SetRate
      public: virtual double Rate() const = 0;

      /// \brief Set how many particles per second should be emitted.
      /// Default value is 10.
      /// \param[in] _rate Particles per second.
      /// \sa Rate
      public: virtual void SetRate(double _rate) = 0;

      /// \brief Get the number of seconds the emitter is active.
      /// A value of 0 means infinite duration.
      /// \return Total duration of the emitter (seconds).
      /// \sa SetDuration
      public: virtual double Duration() const = 0;

      /// \brief Set the number of seconds the emitter is active.
      /// A value of 0 means infinite duration.
      /// Default value is 0.
      /// \param[in] _duration Total duration of the emitter (seconds).
      /// \sa Duration
      public: virtual void SetDuration(double _duration) = 0;

      /// \brief Is the particle emitter enabled?
      /// \return True when enabled or false otherwise.
      /// \sa SetEmitting
      public: virtual bool Emitting() const = 0;

      /// \brief This is used to turn on or off particle emission.
      /// Default value is false.
      /// \param[in] _enable True for enabling the emission or false otherwise.
      /// \sa Emitting
      public: virtual void SetEmitting(bool _enable) = 0;

      /// \brief Get the particle dimensions (width, height, depth).
      /// \return Particle dimensions.
      /// \sa SetParticleSize
      public: virtual ignition::math::Vector3d ParticleSize() const = 0;

      /// \brief Set the particle dimensions (width, height, depth).
      /// Default value is {1, 1, 1}.
      /// \param[in] _size Particle dimensions.
      /// \sa ParticleSize
      public: virtual void SetParticleSize(
                  const ignition::math::Vector3d &_size) = 0;

      /// \brief Get the number of seconds each particle will ’live’ for before
      /// being destroyed.
      /// \return Lifetime of each particle (seconds).
      /// \sa SetLifetime
      public: virtual double Lifetime() const = 0;

      /// \brief Set the number of seconds each particle will ’live’ for before
      /// being destroyed.
      /// Default value is 5.
      /// \param[in] _lifetime Lifetime of each particle (seconds).
      /// \sa Lifetime
      public: virtual void SetLifetime(double _lifetime) = 0;

      /// \brief Get the material which all particles in the emitter will use.
      /// \return The material pointer.
      /// \sa SetMaterial
      public: virtual MaterialPtr Material() const = 0;

      /// \brief Sets the material which all particles in the emitter will use.
      /// \param[in] _material The material pointer.
      /// \sa Material
      public: virtual void SetMaterial(const MaterialPtr &_material) = 0;

      /// \brief Get the minimum velocity each particle is emitted (m/s).
      /// \return Minimum velocity.
      /// \sa MaxVelocity
      /// \sa SetVelocityRange
      public: virtual double MinVelocity() const = 0;

      /// \brief Get the maximum velocity each particle is emitted (m/s).
      /// \return Maximum velocity.
      /// \sa MinVelocity
      /// \sa SetVelocityRange
      public: virtual double MaxVelocity() const = 0;

      /// \brief Set a velocity range and each particle is emitted with a
      /// random velocity within this range (m/s).
      /// Default value is 1 for both velocities.
      /// \param[in] _minVelocity Minimum velocity.
      /// \param[in] _maxVelocity Maximum velocity.
      /// \sa MinVelocity
      /// \sa MaxVelocity
      public: virtual void SetVelocityRange(double _minVelocity,
                                            double _maxVelocity) = 0;

      /// \brief Get the starting color of the particles.
      /// \return Start color.
      /// \sa ColorEnd
      /// \sa SetColorRange
      public: virtual ignition::math::Color ColorStart() const = 0;

      /// \brief Get the end color of the particles.
      /// \return End color.
      /// \sa ColorStart
      /// \sa SetColorRange
      public: virtual ignition::math::Color ColorEnd() const = 0;

      /// \brief Sets a color for all particle emitted.
      /// The actual color will be interpolated between these two colors
      /// Color::White is the default color for the particles unless a specific
      /// function is used.
      /// Note that this function overrides the particle colors set with
      /// SetColorRangeImage().
      /// \param[in] _colorStart Start color.
      /// \param[in] _colorEnd End color.
      /// \sa ColorStart
      /// \sa ColorEnd
      public: virtual void SetColorRange(
                  const ignition::math::Color &_colorStart,
                  const ignition::math::Color &_colorEnd) = 0;

      /// \brief Get the amount by which to scale the particles in both x and y
      /// direction per second.
      /// Default value is 1.
      /// \return The scale rate.
      /// \sa SetScaleRate
      public: virtual double ScaleRate() const = 0;

      /// \brief Set the amount by which to scale the particles in both x and y
      /// direction per second.
      /// \param[in] _scaleRate The scale rate.
      /// \sa ScaleRate
      public: virtual void SetScaleRate(double _scaleRate) = 0;

      /// \brief Get the path to the color image used as an affector.
      /// \return The color image name or empty string if the image is not set.
      /// \sa SetColorRangeImage
      public: virtual std::string ColorRangeImage() const = 0;

      /// \brief Set the path to the color image used as an affector. This
      /// affector modifies the color of particles in flight. The colors are
      /// taken from a specified image file. The range of color values begins
      /// from the left side of the image and move to the right over the
      /// lifetime of the particle, therefore only the horizontal dimension of
      /// the image is used.
      /// Note that this function overrides the particle colors set with
      /// SetColorRange().
      /// \param[in] _image The color image name.
      /// \sa ColorRangeImage
      public: virtual void SetColorRangeImage(const std::string &_image) = 0;

      /// \brief Get the particle scatter ratio.
      /// \return The particle scatter ratio.
      /// \sa SetParticleScatterRatio
      public: virtual float ParticleScatterRatio() const = 0;

      /// \brief Set the particle scatter ratio.
      /// \param[in] _ratio The scatter ratio. The particle emitter's scatter
      /// ratio will only be set to _ratio if _ratio > 0.
      /// \sa ParticleScatterRatio
      public: virtual void SetParticleScatterRatio(float _ratio) = 0;
    };
    }
  }
}
#endif
