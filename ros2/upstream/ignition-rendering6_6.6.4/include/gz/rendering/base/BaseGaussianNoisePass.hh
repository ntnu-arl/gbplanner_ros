/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_BASE_BASEGAUSSIANNOISEPASS_HH_
#define GZ_RENDERING_BASE_BASEGAUSSIANNOISEPASS_HH_

#include <string>
#include <gz/math/Rand.hh>

#include "gz/rendering/GaussianNoisePass.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /* \class BaseGaussianNoisePass BaseGaussianNoisePass.hh \
     * ignition/rendering/base/BaseGaussianNoisePass.hh
     */
    /// \brief Base Gaussian noise render pass.
    template <class T>
    class BaseGaussianNoisePass :
      public virtual GaussianNoisePass,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseGaussianNoisePass();

      /// \brief Destructor
      public: virtual ~BaseGaussianNoisePass();

      // Documentation inherited.
      public: double Mean() const;

      // Documentation inherited.
      public: double StdDev() const;

      // Documentation inherited.
      public: double Bias() const;

      // Documentation inherited.
      public: void SetMean(double _mean);

      // Documentation inherited.
      public: void SetStdDev(double _stdDev);

      // Documentation inherited.
      public: void SetBiasMean(double _biasMean);

      // Documentation inherited.
      public: void SetBiasStdDev(double _biasStdDev);

      // Sample the bias from bias mean and bias standard deviation
      protected: void SampleBias();

      /// \brief Gaussian noise mean.
      protected: double mean = 0.0;

      /// \brief Standard deviation of Gaussian noise
      protected: double stdDev = 0.0;

      /// \brief Gaussian noise bias.
      protected: double bias = 0.0;

      /// \brief The mean of the Gaussian distribution from which bias values
      /// are drawn.
      protected: double biasMean = 0;

      /// \brief The standard deviation of the Gaussian distribution from
      /// which bias values are drawn.
      protected: double biasStdDev = 0;
    };

    //////////////////////////////////////////////////
    // BaseGaussianNoisePass
    //////////////////////////////////////////////////
    template <class T>
    BaseGaussianNoisePass<T>::BaseGaussianNoisePass()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseGaussianNoisePass<T>::~BaseGaussianNoisePass()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGaussianNoisePass<T>::Mean() const
    {
      return this->mean;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGaussianNoisePass<T>::StdDev() const
    {
      return this->stdDev;
    }

    //////////////////////////////////////////////////
    template <class T>
    double BaseGaussianNoisePass<T>::Bias() const
    {
      return this->bias;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGaussianNoisePass<T>::SetMean(double _mean)
    {
      this->mean = _mean;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGaussianNoisePass<T>::SetStdDev(double _stdDev)
    {
      this->stdDev = _stdDev;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGaussianNoisePass<T>::SetBiasMean(double _biasMean)
    {
      this->biasMean = _biasMean;
      this->SampleBias();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGaussianNoisePass<T>::SetBiasStdDev(double _biasStdDev)
    {
      this->biasStdDev = _biasStdDev;
      this->SampleBias();
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGaussianNoisePass<T>::SampleBias()
    {
      this->bias =
          gz::math::Rand::DblNormal(this->biasMean, this->biasStdDev);
      // With equal probability, we pick a negative bias (by convention,
      // rateBiasMean should be positive, though it would work fine if
      // negative).
      if (gz::math::Rand::DblUniform() < 0.5)
        this->bias = -this->bias;
    }
    }
  }
}
#endif
