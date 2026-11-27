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

#ifndef IGNITION_RENDERING_BASE_BASEPARTICLEEMITTER_HH_
#define IGNITION_RENDERING_BASE_BASEPARTICLEEMITTER_HH_

#include <string>
#include "ignition/rendering/base/BaseScene.hh"
#include "ignition/rendering/base/BaseNode.hh"
#include "ignition/rendering/ParticleEmitter.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /* \class BaseParticleEmitter BaseParticleEmitter.hh \
     * ignition/rendering/base/BaseParticleEmitter.hh
     */
    /// \brief A base implementation of the ParticleEmitter class
    template <class T>
    class BaseParticleEmitter :
      public virtual ParticleEmitter,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseParticleEmitter();

      /// \brief Destructor
      public: virtual ~BaseParticleEmitter();

      // Documentation inherited
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void PreRender() override;

      /// \brief Reset the particle emitter visual state
      public: virtual void Reset();

      // Documentation inherited.
      public: virtual EmitterType Type() const override;

      // Documentation inherited.
      public: virtual void SetType(const EmitterType _type) override;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d EmitterSize() const override;

      // Documentation inherited.
      public: virtual void SetEmitterSize(
                  const ignition::math::Vector3d &_size) override;

      // Documentation inherited.
      public: virtual double Rate() const override;

      // Documentation inherited.
      public: virtual void SetRate(double _rate) override;

      // Documentation inherited.
      public: virtual double Duration() const override;

      // Documentation inherited.
      public: virtual void SetDuration(double _duration) override;

      // Documentation inherited.
      public: virtual bool Emitting() const override;

      // Documentation inherited.
      public: virtual void SetEmitting(bool _enable) override;

      // Documentation inherited.
      public: virtual ignition::math::Vector3d ParticleSize() const override;

      // Documentation inherited.
      public: virtual void SetParticleSize(
                  const ignition::math::Vector3d &_size) override;

      // Documentation inherited.
      public: virtual double Lifetime() const override;

      // Documentation inherited.
      public: virtual void SetLifetime(double _lifetime) override;

      // Documentation inherited.
      public: virtual MaterialPtr Material() const override;

      // Documentation inherited.
      public: virtual void SetMaterial(const MaterialPtr &_material) override;

      // Documentation inherited.
      public: virtual double MinVelocity() const override;

      // Documentation inherited.
      public: virtual double MaxVelocity() const override;

      // Documentation inherited.
      public: virtual void SetVelocityRange(double _minVelocity,
                                            double _maxVelocity) override;

      // Documentation inherited.
      public: virtual ignition::math::Color ColorStart() const override;

      // Documentation inherited.
      public: virtual ignition::math::Color ColorEnd() const override;

      // Documentation inherited.
      public: virtual void SetColorRange(
                  const ignition::math::Color &_colorStart,
                  const ignition::math::Color &_colorEnd) override;

      // Documentation inherited.
      public: virtual double ScaleRate() const override;

      // Documentation inherited.
      public: virtual void SetScaleRate(double _scaleRate) override;

      // Documentation inherited.
      public: virtual std::string ColorRangeImage() const override;

      // Documentation inherited.
      public: virtual void SetColorRangeImage(
                  const std::string &_image) override;

      // Documentation inherited.
      public: virtual float ParticleScatterRatio() const override;

      // Documentation inherited.
      public: virtual void SetParticleScatterRatio(float _ratio) override;

      /// \brief Emitter type.
      protected: EmitterType type = EM_POINT;

      /// \brief Emitter size.
      protected: ignition::math::Vector3d emitterSize =
          ignition::math::Vector3d::One;

      /// \brief Rate of emission.
      protected: double rate = 10;

      /// \brief Duration of the emitter.
      protected: double duration = 0;

      /// \brief Whether particle emitter is enabled or not.
      protected: bool emitting = false;

      /// \brief Particle size.
      protected: ignition::math::Vector3d particleSize = {1, 1, 1};

      /// \brief Particle lifetime.
      protected: double lifetime = 5;

      /// \brief Particle material.
      protected: MaterialPtr material = nullptr;

      /// \brief The minimum velocity of each particle.
      protected: double minVelocity = 1;

      /// \brief The maximum velocity of each particle.
      protected: double maxVelocity = 1;

      /// \brief The start color of a particle to choose from.
      protected: ignition::math::Color colorStart =
          ignition::math::Color::White;

      /// \brief The end color of a particle to choose from.
      protected: ignition::math::Color colorEnd =
          ignition::math::Color::White;

      /// \brief The scale rate.
      protected: double scaleRate = 1;

      /// \brief The color image.
      protected: std::string colorRangeImage = "";

      /// \brief The particle scatter ratio. This is used to determine the ratio
      /// of particles that will be detected by sensors. Increasing the ratio
      /// increases the scatter of the particles, which means there is a higher
      /// chance of particles reflecting and interfering with depth sensing,
      /// making the emitter appear more dense. Decreasing the ratio decreases
      /// the scatter of the particles, making it appear less dense. This value
      /// should be > 0.
      protected: float particleScatterRatio = 0.65f;

      /// \brief Only the scene can create a particle emitter
      private: friend class BaseScene;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseParticleEmitter<T>::BaseParticleEmitter()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseParticleEmitter<T>::~BaseParticleEmitter()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::Init()
    {
      T::Init();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::Reset()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::PreRender()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    EmitterType BaseParticleEmitter<T>::Type() const
    {
      return this->type;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetType(const EmitterType _type)
    {
      this->type = _type;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Vector3d BaseParticleEmitter<T>::EmitterSize() const
    {
      return this->emitterSize;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetEmitterSize(
                const ignition::math::Vector3d &_size)
    {
      this->emitterSize = _size;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseParticleEmitter<T>::Rate() const
    {
      return this->rate;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetRate(double _rate)
    {
      this->rate = _rate;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseParticleEmitter<T>::Duration() const
    {
      return this->duration;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetDuration(double _duration)
    {
      this->duration = _duration;
    }

    //////////////////////////////////////////////////
    template <class T>
    bool BaseParticleEmitter<T>::Emitting() const
    {
      return this->emitting;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetEmitting(bool _enable)
    {
      this->emitting = _enable;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Vector3d BaseParticleEmitter<T>::ParticleSize() const
    {
      return this->particleSize;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetParticleSize(
                const ignition::math::Vector3d &_size)
    {
      this->particleSize = _size;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseParticleEmitter<T>::Lifetime() const
    {
      return this->lifetime;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetLifetime(double _lifetime)
    {
      this->lifetime = _lifetime;
    }

    //////////////////////////////////////////////////
    template <class T>
    MaterialPtr BaseParticleEmitter<T>::Material() const
    {
      return this->material;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetMaterial(const MaterialPtr &_material)
    {
      this->material = _material;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseParticleEmitter<T>::MinVelocity() const
    {
      return this->minVelocity;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseParticleEmitter<T>::MaxVelocity() const
    {
      return this->maxVelocity;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetVelocityRange(double _minVelocity,
                                                  double _maxVelocity)
    {
      this->minVelocity = _minVelocity;
      this->maxVelocity = _maxVelocity;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Color BaseParticleEmitter<T>::ColorStart() const
    {
      return this->colorStart;
    }

    //////////////////////////////////////////////////
    template <class T>
    ignition::math::Color BaseParticleEmitter<T>::ColorEnd() const
    {
      return this->colorEnd;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetColorRange(
                  const ignition::math::Color &_colorStart,
                  const ignition::math::Color &_colorEnd)
    {
      this->colorStart = _colorStart;
      this->colorEnd = _colorEnd;
    }

    /////////////////////////////////////////////////
    template <class T>
    double BaseParticleEmitter<T>::ScaleRate() const
    {
      return this->scaleRate;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetScaleRate(double _scaleRate)
    {
      this->scaleRate = _scaleRate;
    }

    /////////////////////////////////////////////////
    template <class T>
    std::string BaseParticleEmitter<T>::ColorRangeImage() const
    {
      return this->colorRangeImage;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetColorRangeImage(const std::string &_image)
    {
      this->colorRangeImage = _image;
    }

    /////////////////////////////////////////////////
    template <class T>
    float BaseParticleEmitter<T>::ParticleScatterRatio() const
    {
      return this->particleScatterRatio;
    }

    /////////////////////////////////////////////////
    template <class T>
    void BaseParticleEmitter<T>::SetParticleScatterRatio(float _ratio)
    {
      if (_ratio > 0.0f)
        this->particleScatterRatio = _ratio;
    }
    }
  }
}
#endif
