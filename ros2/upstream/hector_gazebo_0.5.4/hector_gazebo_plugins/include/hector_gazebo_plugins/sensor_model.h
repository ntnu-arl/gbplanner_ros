//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
#define HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H

#include <sdf/sdf.hh>
#if (GAZEBO_MAJOR_VERSION < 8)
#include <gazebo/math/Vector3.hh>
#endif

#include <hector_gazebo_plugins/SensorModelConfig.h>
#include <numeric>

namespace gazebo {

using hector_gazebo_plugins::SensorModelConfig;

template <typename T>
class SensorModel_ {
public:
  SensorModel_();
  virtual ~SensorModel_();

  virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = std::string());

  virtual T operator()(const T& value) const { return value * scale_error + current_error_; }
  virtual T operator()(const T& value, double dt) { return value * scale_error + update(dt); }

  virtual T update(double dt);
  virtual void reset();
  virtual void reset(const T& value);

  virtual const T& getCurrentError() const { return current_error_; }
  virtual T getCurrentBias() const { return current_drift_ + offset; }
  virtual const T& getCurrentDrift() const { return current_drift_; }
  virtual const T& getScaleError() const { return scale_error; }

  virtual void setCurrentDrift(const T& new_drift) { current_drift_ = new_drift; }

  virtual void dynamicReconfigureCallback(SensorModelConfig &config, uint32_t level);

private:
  virtual bool LoadImpl(sdf::ElementPtr _element, T& _value);

public:
  T offset;
  T drift;
  T drift_frequency;
  T gaussian_noise;
  T scale_error;

private:
  T current_drift_;
  T current_error_;
};

template <typename T>
SensorModel_<T>::SensorModel_()
  : offset()
  , drift()
  , drift_frequency()
  , gaussian_noise()
{
  drift_frequency = 1.0/3600.0; // time constant 1h
  scale_error = 1.0;
  reset();
}

template <typename T>
SensorModel_<T>::~SensorModel_()
{
}

template <typename T>
void SensorModel_<T>::Load(sdf::ElementPtr _sdf, const std::string& prefix)
{
  std::string _offset, _drift, _drift_frequency, _gaussian_noise, _scale_error;

  if (prefix.empty()) {
    _offset              = "offset";
    _drift               = "drift";
    _drift_frequency     = "driftFrequency";
    _gaussian_noise      = "gaussianNoise";
    _scale_error         = "scaleError";
  } else {
    _offset              = prefix + "Offset";
    _drift               = prefix + "Drift";
    _drift_frequency     = prefix + "DriftFrequency";
    _gaussian_noise      = prefix + "GaussianNoise";
    _scale_error         = prefix + "ScaleError";
  }

  if (_sdf->HasElement(_offset))              LoadImpl(_sdf->GetElement(_offset), offset);
  if (_sdf->HasElement(_drift))               LoadImpl(_sdf->GetElement(_drift), drift);
  if (_sdf->HasElement(_drift_frequency))     LoadImpl(_sdf->GetElement(_drift_frequency), drift_frequency);
  if (_sdf->HasElement(_gaussian_noise))      LoadImpl(_sdf->GetElement(_gaussian_noise), gaussian_noise);
  if (_sdf->HasElement(_scale_error))         LoadImpl(_sdf->GetElement(_scale_error), scale_error);

  reset();
}

template <typename T>
bool SensorModel_<T>::LoadImpl(sdf::ElementPtr _element, T& _value) {
  if (!_element->GetValue()) return false;
  return _element->GetValue()->Get(_value);
}

namespace {
  template <typename T>
  static inline T SensorModelGaussianKernel(T mu, T sigma)
  {
    // using Box-Muller transform to generate two independent standard normally distributed normal variables
    // see wikipedia
    T U = (T)rand()/(T)RAND_MAX; // normalized uniform random variable
    T V = (T)rand()/(T)RAND_MAX; // normalized uniform random variable
    T X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
    X = sigma * X + mu;
    return X;
  }

  template <typename T>
  static inline T SensorModelInternalUpdate(T& current_drift, T drift, T drift_frequency, T offset, T gaussian_noise, double dt)
  {
    // current_drift = current_drift - dt * (current_drift * drift_frequency + SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift));
    current_drift = exp(-dt * drift_frequency) * current_drift + dt * SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift);
    return offset + current_drift + SensorModelGaussianKernel(T(), gaussian_noise);
  }
}

template <typename T>
T SensorModel_<T>::update(double dt)
{
  for(std::size_t i = 0; i < current_error_.size(); ++i) current_error_[i] = SensorModelInternalUpdate(current_drift_[i], drift[i], drift_frequency[i], offset[i], gaussian_noise[i], dt);
  return current_error_;
}

template <>
double SensorModel_<double>::update(double dt)
{
  current_error_ = SensorModelInternalUpdate(current_drift_, drift, drift_frequency, offset, gaussian_noise, dt);
  return current_error_;
}

#if (GAZEBO_MAJOR_VERSION >= 8)
template <>
ignition::math::Vector3d SensorModel_<ignition::math::Vector3d>::update(double dt)
{
  current_error_.X() = SensorModelInternalUpdate(current_drift_.X(), drift.X(), drift_frequency.X(), offset.X(), gaussian_noise.X(), dt);
  current_error_.Y() = SensorModelInternalUpdate(current_drift_.Y(), drift.Y(), drift_frequency.Y(), offset.Y(), gaussian_noise.Y(), dt);
  current_error_.Z() = SensorModelInternalUpdate(current_drift_.Z(), drift.Z(), drift_frequency.Z(), offset.Z(), gaussian_noise.Z(), dt);
  return current_error_;
}
#else
template <>
math::Vector3 SensorModel_<math::Vector3>::update(double dt)
{
  current_error_.x = SensorModelInternalUpdate(current_drift_.x, drift.x, drift_frequency.x, offset.x, gaussian_noise.x, dt);
  current_error_.y = SensorModelInternalUpdate(current_drift_.y, drift.y, drift_frequency.y, offset.y, gaussian_noise.y, dt);
  current_error_.z = SensorModelInternalUpdate(current_drift_.z, drift.z, drift_frequency.z, offset.z, gaussian_noise.z, dt);
  return current_error_;
}
#endif

template <typename T>
void SensorModel_<T>::reset()
{
  for(std::size_t i = 0; i < current_drift_.size(); ++i) current_drift_[i] = SensorModelGaussianKernel(T::value_type(), drift[i]);
  current_error_ = T();
}

template <>
void SensorModel_<double>::reset()
{
  current_drift_ = SensorModelGaussianKernel(0.0, drift);
  current_error_ = 0.0;
}

#if (GAZEBO_MAJOR_VERSION >= 8)
template <>
void SensorModel_<ignition::math::Vector3d>::reset()
{
  current_drift_.X() = SensorModelGaussianKernel(0.0, drift.X());
  current_drift_.Y() = SensorModelGaussianKernel(0.0, drift.Y());
  current_drift_.Z() = SensorModelGaussianKernel(0.0, drift.Z());
  current_error_ = ignition::math::Vector3d();
}
#else
template <>
void SensorModel_<math::Vector3>::reset()
{
  current_drift_.x = SensorModelGaussianKernel(0.0, drift.x);
  current_drift_.y = SensorModelGaussianKernel(0.0, drift.y);
  current_drift_.z = SensorModelGaussianKernel(0.0, drift.z);
  current_error_ = math::Vector3();
}
#endif

template <typename T>
void SensorModel_<T>::reset(const T& value)
{
  current_drift_ = value;
  current_error_ = T();
}

namespace helpers {
  template <typename T> struct scalar_value { static double toDouble(const T &orig) { return orig; } };
  template <typename T> struct scalar_value<std::vector<T> > { static double toDouble(const std::vector<T> &orig) { return (double) std::accumulate(orig.begin(), orig.end()) / orig.size(); } };
#if (GAZEBO_MAJOR_VERSION >= 8)
  template <> struct scalar_value<ignition::math::Vector3d> { static double toDouble(const ignition::math::Vector3d &orig) { return (orig.X() + orig.Y() + orig.Z()) / 3; } };
#else
  template <> struct scalar_value<math::Vector3> { static double toDouble(const math::Vector3 &orig) { return (orig.x + orig.y + orig.z) / 3; } };
#endif
}

template <typename T>
void SensorModel_<T>::dynamicReconfigureCallback(SensorModelConfig &config, uint32_t level)
{
  if (level == 1) {
    gaussian_noise  = config.gaussian_noise;
    offset          = config.offset;
    drift           = config.drift;
    drift_frequency = config.drift_frequency;
    scale_error     = config.scale_error;
  } else {
    config.gaussian_noise  = helpers::scalar_value<T>::toDouble(gaussian_noise);
    config.offset          = helpers::scalar_value<T>::toDouble(offset);
    config.drift           = helpers::scalar_value<T>::toDouble(drift);
    config.drift_frequency = helpers::scalar_value<T>::toDouble(drift_frequency);
    config.scale_error     = helpers::scalar_value<T>::toDouble(scale_error);
  }
}

typedef SensorModel_<double> SensorModel;
#if (GAZEBO_MAJOR_VERSION >= 8)
typedef SensorModel_<ignition::math::Vector3d> SensorModel3;
#else
typedef SensorModel_<math::Vector3> SensorModel3;
#endif

}

#endif // HECTOR_GAZEBO_PLUGINS_SENSOR_MODEL_H
