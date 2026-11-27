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
#ifndef GZ_RENDERING_GAUSSIANNOISEPASS_HH_
#define GZ_RENDERING_GAUSSIANNOISEPASS_HH_

#include <string>
#include "gz/rendering/config.hh"
#include "gz/rendering/Export.hh"
#include "gz/rendering/RenderPass.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /* \class GaussianNoisePass GaussianNoisePass.hh \
     * ignition/rendering/GaussianNoisePass.hh
     */
    /// \brief A render pass that applies Gaussian noise to the render target
    class IGNITION_RENDERING_VISIBLE GaussianNoisePass
      : public virtual RenderPass
    {
      /// \brief Constructor
      public: GaussianNoisePass();

      /// \brief Destructor
      public: virtual ~GaussianNoisePass();

      /// \brief Accessor for mean.
      /// \return Mean of Gaussian noise.
      public: virtual double Mean() const = 0;

      /// \brief Accessor for stddev.
      /// \return Standard deviation of Gaussian noise.
      public: virtual double StdDev() const = 0;

      /// \brief Accessor for bias.
      /// \return Bias on output.
      public: virtual double Bias() const = 0;

      /// \brief Set mean.
      /// \param[in] _mean Mean of Gaussian noise.
      public: virtual void SetMean(double _mean) = 0;

      /// \brief Set stddev.
      /// \param[in] _stdDev Standard deviation of Gaussian noise.
      public: virtual void SetStdDev(double _stdDev) = 0;

      /// \brief Set the mean of the bias value. Bias is computed based on
      /// the bias mean and bias standard deviation.
      /// \sa SetBiasStdDev
      public: virtual void SetBiasMean(double _biasMean) = 0;

      /// \brief Set the standard deviation of the bias value. Bias is computed
      /// based on the bias mean and bias standard deviation.
      /// \sa SetBiasMean
      public: virtual void SetBiasStdDev(double _biasStdDev) = 0;
    };
    }
  }
}
#endif
