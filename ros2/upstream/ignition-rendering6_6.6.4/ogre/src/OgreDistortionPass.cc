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


#include <ignition/common/Console.hh>

#include "ignition/rendering/RenderPassSystem.hh"
#include "ignition/rendering/ogre/OgreIncludes.hh"
#include "ignition/rendering/ogre/OgreDistortionPass.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    class OgreDistortionPass::Implementation
    {
      /// \brief Scale applied to distorted image.
      public: ignition::math::Vector2d distortionScale = {1.0, 1.0};

      /// \brief True if the distorted image will be cropped to remove the
      /// black pixels at the corners of the image.
      public: bool distortionCrop = true;

      /// \brief Distortion compositor.
      public: Ogre::CompositorInstance *distortionInstance = nullptr;

      /// \brief Ogre Material that contains the distortion shader
      public: Ogre::MaterialPtr distortionMaterial;

      /// \brief Ogre Texture that contains the distortion map
      public: Ogre::TexturePtr distortionTexture;

      /// \brief Mapping of distorted to undistorted normalized pixels
      public: std::vector<ignition::math::Vector2d> distortionMap;

      /// \brief Width of distortion texture map
      public: unsigned int distortionTexWidth = 0u;

      /// \brief Height of distortion texture map
      public: unsigned int distortionTexHeight = 0u;

      /// \brief Distortion compositor listener
      /// uses <ignition/utils/ImplPtr.hh> from ign-rendering7
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      public: std::shared_ptr<DistortionCompositorListener>
          distortionCompositorListener;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };

    class DistortionCompositorListener
      : public Ogre::CompositorInstance::Listener
    {
      /// \brief Constructor, setting mean and standard deviation.
      public: DistortionCompositorListener(
          const Ogre::TexturePtr &_distortionTexture,
          const ignition::math::Vector2d &_distortionScale):
          distortionTexture(_distortionTexture),
          distortionScale(_distortionScale) {}

      /// \brief Callback that OGRE will invoke for us on each render call
      /// \param[in] _passID OGRE material pass ID.
      /// \param[in] _mat Pointer to OGRE material.
      public: virtual void notifyMaterialRender(unsigned int _passId,
                                                Ogre::MaterialPtr &_mat)
      {
        // If more compositors are added to the camera in addition to the
        // distortion compositor, an Ogre bug will cause the material for the
        // last camera initialized to be used for all cameras. This workaround
        // doesn't correct the material used, but it applies the correct texture
        // to this active material.
        _mat->getTechnique(0)->getPass(_passId)->getTextureUnitState(1)->
            setTexture(distortionTexture);

        // @todo Explore more efficent implementations as it is run every frame
        Ogre::GpuProgramParametersSharedPtr params =
            _mat->getTechnique(0)->getPass(_passId)
                     ->getFragmentProgramParameters();
        params->setNamedConstant("scale",
            Ogre::Vector3(
              1.0 / distortionScale.X(),
              1.0 / distortionScale.Y(),
              1.0));
      }

      private: const Ogre::TexturePtr &distortionTexture;

      private: const ignition::math::Vector2d &distortionScale;
    };
    }
  }
}

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreDistortionPass::OgreDistortionPass() :
    dataPtr(utils::MakeUniqueImpl<Implementation>())
{
}

//////////////////////////////////////////////////
OgreDistortionPass::~OgreDistortionPass()
{
}

//////////////////////////////////////////////////
void OgreDistortionPass::PreRender()
{
  if (!this->dataPtr->distortionInstance)
    return;
  if (this->enabled != this->dataPtr->distortionInstance->getEnabled())
    this->dataPtr->distortionInstance->setEnabled(this->enabled);
}

//////////////////////////////////////////////////
void OgreDistortionPass::CreateRenderPass()
{
  if (!this->ogreCamera)
  {
    ignerr << "No camera set for applying Distortion Pass" << std::endl;
    return;
  }

  if (this->dataPtr->distortionInstance)
  {
    ignerr << "Distortion pass already created. " << std::endl;
    return;
  }

  // If no distortion is required, immediately return.
  if (ignition::math::equal(this->k1, 0.0) &&
      ignition::math::equal(this->k2, 0.0) &&
      ignition::math::equal(this->k3, 0.0) &&
      ignition::math::equal(this->p1, 0.0) &&
      ignition::math::equal(this->p2, 0.0))
  {
    return;
  }

  float viewportWidth = static_cast<float>(this->ogreCamera->
      getViewport()->getActualWidth());
  float viewportHeight = static_cast<float>(this->ogreCamera->
      getViewport()->getActualHeight());

  // seems to work best with a square distortion map texture
  unsigned int texSize = static_cast<unsigned int>(viewportHeight >
      viewportWidth ? viewportHeight : viewportWidth);
  // calculate focal length from largest fov
  const double fov = viewportHeight > viewportWidth ?
      this->ogreCamera->getFOVy().valueRadians() :
      (this->ogreCamera->getFOVy().valueRadians() *
      this->ogreCamera->getAspectRatio());
  const double focalLength = texSize / (2 * tan(fov / 2));
  this->dataPtr->distortionTexWidth = texSize;
  this->dataPtr->distortionTexHeight = texSize;
  unsigned int imageSize =
      this->dataPtr->distortionTexWidth * this->dataPtr->distortionTexHeight;
  double colStepSize = 1.0 / this->dataPtr->distortionTexWidth;
  double rowStepSize = 1.0 / this->dataPtr->distortionTexHeight;

  // Half step-size vector to add to the value being placed in distortion map.
  // Necessary for compositor to correctly interpolate pixel values.
  const auto halfTexelSize =
      0.5 * ignition::math::Vector2d(rowStepSize, colStepSize);

  // initialize distortion map
  this->dataPtr->distortionMap.resize(imageSize);
  for (unsigned int i = 0; i < this->dataPtr->distortionMap.size(); ++i)
  {
    this->dataPtr->distortionMap[i] = -1;
  }

  ignition::math::Vector2d distortionCenterCoordinates(
      this->lensCenter.X() * this->dataPtr->distortionTexWidth,
      this->lensCenter.Y() * this->dataPtr->distortionTexWidth);

  // declare variables before the loop
  const auto unsetPixelVector =  ignition::math::Vector2d(-1, -1);
  ignition::math::Vector2d normalizedLocation,
      distortedLocation,
      newDistortedCoordinates,
      currDistortedCoordinates;
  unsigned int distortedIdx;
  int distortedCol,
      distortedRow;
  double normalizedColLocation, normalizedRowLocation;

  // fill the distortion map
  for (unsigned int mapRow = 0; mapRow < this->dataPtr->distortionTexHeight;
    ++mapRow)
  {
    normalizedRowLocation = mapRow*rowStepSize;
    for (unsigned int mapCol = 0; mapCol < this->dataPtr->distortionTexWidth;
      ++mapCol)
    {
      normalizedColLocation = mapCol*colStepSize;

      normalizedLocation[0] = normalizedColLocation;
      normalizedLocation[1] = normalizedRowLocation;

      distortedLocation = this->Distort(
          normalizedLocation,
          this->lensCenter,
          this->k1, this->k2, this->k3,
          this->p1, this->p2,
          this->dataPtr->distortionTexWidth,
          focalLength);

      // compute the index in the distortion map
      distortedCol = static_cast<int>(round(distortedLocation.X() *
        this->dataPtr->distortionTexWidth));
      distortedRow = static_cast<int>(round(distortedLocation.Y() *
        this->dataPtr->distortionTexHeight));

      // Note that the following makes sure that, for significant distortions,
      // there is not a problem where the distorted image seems to fold over
      // itself. This is accomplished by favoring pixels closer to the center
      // of distortion, and this change applies to both the legacy and
      // nonlegacy distortion modes.

      // Make sure the distorted pixel is within the texture dimensions
      if (distortedCol >= 0 && distortedRow >= 0 &&
          static_cast<unsigned int>(distortedCol) <
            this->dataPtr->distortionTexWidth &&
          static_cast<unsigned int>(distortedRow) <
            this->dataPtr->distortionTexHeight)
      {
        distortedIdx = distortedRow * this->dataPtr->distortionTexWidth +
          distortedCol;

        // check if the index has already been set
        if (this->dataPtr->distortionMap[distortedIdx] != unsetPixelVector)
        {
          // grab current coordinates that map to this destination
          currDistortedCoordinates =
            this->dataPtr->distortionMap[distortedIdx] *
            this->dataPtr->distortionTexWidth;

          // grab new coordinates to map to
          newDistortedCoordinates[0] = mapCol;
          newDistortedCoordinates[1] = mapRow;

          // use the new mapping if it is closer to the center of the distortion
          if (newDistortedCoordinates.Distance(distortionCenterCoordinates) <
              currDistortedCoordinates.Distance(distortionCenterCoordinates))
          {
            this->dataPtr->distortionMap[distortedIdx] = normalizedLocation +
              halfTexelSize;
          }
        }
        else
        {
          this->dataPtr->distortionMap[distortedIdx] = normalizedLocation +
            halfTexelSize;
        }
      }
      // else: mapping is outside of the image bounds.
      // This is expected and normal to ensure
      // no black borders; carry on
    }
  }

  // set up the distortion instance
  this->dataPtr->distortionMaterial =
    Ogre::MaterialManager::getSingleton().getByName(
        "Distortion");
  this->dataPtr->distortionMaterial =
    this->dataPtr->distortionMaterial->clone(
        this->ogreCamera->getName() + "_Distortion");

  // create the distortion map texture for the distortion instance
  std::string texName = this->ogreCamera->getName() + "_distortionTex";
  this->dataPtr->distortionTexture =
      Ogre::TextureManager::getSingleton().createManual(
          texName,
          "General",
          Ogre::TEX_TYPE_2D,
          this->dataPtr->distortionTexWidth,
          this->dataPtr->distortionTexHeight,
          0,
          Ogre::PF_FLOAT32_RGB);
  Ogre::HardwarePixelBufferSharedPtr pixelBuffer =
      this->dataPtr->distortionTexture->getBuffer();

  // fill the distortion map, while interpolating to fill dead pixels
  pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL);
  const Ogre::PixelBox &pixelBox = pixelBuffer->getCurrentLock();

#if OGRE_VERSION_MAJOR > 1 || OGRE_VERSION_MINOR >= 11
  // Ogre 1.11 changed Ogre::PixelBox::data from void* to uchar*, hence
  // reinterpret_cast is required here. static_cast is not allowed between
  // pointers of unrelated types (see, for instance, Standard § 3.9.1
  // Fundamental types)
  float *pDest = reinterpret_cast<float *>(pixelBox.data);
#else
  float *pDest = static_cast<float *>(pixelBox.data);
#endif

  for (unsigned int i = 0; i < this->dataPtr->distortionTexHeight; ++i)
  {
    for (unsigned int j = 0; j < this->dataPtr->distortionTexWidth; ++j)
    {
      ignition::math::Vector2d vec =
          this->dataPtr->distortionMap[i *
              this->dataPtr->distortionTexWidth + j];

      // perform interpolation on-the-fly:
      // check for empty mapping within the region and correct it by
      // interpolating the eight neighboring distortion map values.

      if (vec.X() < -0.5 && vec.Y() < -0.5)
      {
        ignition::math::Vector2d left =
            this->DistortionMapValueClamped(j - 1, i);
        ignition::math::Vector2d right =
            this->DistortionMapValueClamped(j + 1, i);
        ignition::math::Vector2d bottom =
            this->DistortionMapValueClamped(j, i + 1);
        ignition::math::Vector2d top =
            this->DistortionMapValueClamped(j, i - 1);

        ignition::math::Vector2d topLeft =
            this->DistortionMapValueClamped(j - 1, i - 1);
        ignition::math::Vector2d topRight =
            this->DistortionMapValueClamped(j + 1, i - 1);
        ignition::math::Vector2d bottomLeft =
            this->DistortionMapValueClamped(j - 1, i + 1);
        ignition::math::Vector2d bottomRight =
            this->DistortionMapValueClamped(j + 1, i + 1);

        ignition::math::Vector2d interpolated;
        double divisor = 0;
        if (right.X() > -0.5)
        {
          divisor++;
          interpolated += right;
        }
        if (left.X() > -0.5)
        {
          divisor++;
          interpolated += left;
        }
        if (top.X() > -0.5)
        {
          divisor++;
          interpolated += top;
        }
        if (bottom.X() > -0.5)
        {
          divisor++;
          interpolated += bottom;
        }

        if (bottomRight.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += bottomRight * 0.707;
        }
        if (bottomLeft.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += bottomLeft * 0.707;
        }
        if (topRight.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += topRight * 0.707;
        }
        if (topLeft.X() > -0.5)
        {
          divisor += 0.707;
          interpolated += topLeft * 0.707;
        }

        if (divisor > 0.5)
        {
          interpolated /= divisor;
        }
        *pDest++ = ignition::math::clamp(interpolated.X(), 0.0, 1.0);
        *pDest++ = ignition::math::clamp(interpolated.Y(), 0.0, 1.0);
      }
      else
      {
        *pDest++ = vec.X();
        *pDest++ = vec.Y();
      }

      // Z coordinate
      *pDest++ = 0;
    }
  }
  pixelBuffer->unlock();

  this->CalculateAndApplyDistortionScale();

  // set up the distortion map texture to be used in the pixel shader.
  this->dataPtr->distortionMaterial->getTechnique(0)->getPass(0)->
      createTextureUnitState(texName, 1);

  // create compositor instance
  this->dataPtr->distortionInstance =
    Ogre::CompositorManager::getSingleton().addCompositor(
        this->ogreCamera->getViewport(), "RenderPass/Distortion");
  this->dataPtr->distortionInstance->getTechnique()->getOutputTargetPass()->
      getPass(0)->setMaterial(this->dataPtr->distortionMaterial);

  this->dataPtr->distortionInstance->setEnabled(this->enabled);

  // add listener
  this->dataPtr->distortionCompositorListener.reset(new
        DistortionCompositorListener(this->dataPtr->distortionTexture,
                                     this->dataPtr->distortionScale));
  this->dataPtr->distortionInstance->addListener(
    this->dataPtr->distortionCompositorListener.get());
}

//////////////////////////////////////////////////
void OgreDistortionPass::Destroy()
{
  if (this->dataPtr->distortionInstance)
  {
    this->dataPtr->distortionInstance->setEnabled(false);
    if (this->dataPtr->distortionCompositorListener)
    {
      this->dataPtr->distortionInstance->removeListener(
          this->dataPtr->distortionCompositorListener.get());
    }
    Ogre::CompositorManager::getSingleton().removeCompositor(
        this->ogreCamera->getViewport(), "RenderPass/Distortion");

    this->dataPtr->distortionInstance = nullptr;
    this->dataPtr->distortionCompositorListener.reset();
  }
}

//////////////////////////////////////////////////
ignition::math::Vector2d OgreDistortionPass::Distort(
    const ignition::math::Vector2d &_in,
    const ignition::math::Vector2d &_center, double _k1, double _k2, double _k3,
    double _p1, double _p2, unsigned int _width, double _f)
{
  // apply Brown's distortion model, see
  // http://en.wikipedia.org/wiki/Distortion_%28optics%29#Software_correction

  ignition::math::Vector2d normalized2d = (_in - _center) * (_width / _f);
  ignition::math::Vector3d normalized(normalized2d.X(), normalized2d.Y(), 0);
  double rSq = normalized.X() * normalized.X() +
               normalized.Y() * normalized.Y();

  // radial
  ignition::math::Vector3d dist = normalized * (1.0 +
      _k1 * rSq +
      _k2 * rSq * rSq +
      _k3 * rSq * rSq * rSq);

  // tangential
  dist.X() += _p2 * (rSq + 2 * (normalized.X()*normalized.X())) +
      2 * _p1 * normalized.X() * normalized.Y();
  dist.Y() += _p1 * (rSq + 2 * (normalized.Y()*normalized.Y())) +
      2 * _p2 * normalized.X() * normalized.Y();

  return ((_center * _width) +
    ignition::math::Vector2d(dist.X(), dist.Y()) *_f) / _width;
}

//////////////////////////////////////////////////
ignition::math::Vector2d
    OgreDistortionPass::DistortionMapValueClamped(
    int _x, int _y) const
{
  if (_x < 0 || _x >= static_cast<int>(this->dataPtr->distortionTexWidth) ||
      _y < 0 || _y >= static_cast<int>(this->dataPtr->distortionTexHeight))
  {
    return ignition::math::Vector2d(-1, -1);
  }
  ignition::math::Vector2d res =
      this->dataPtr->distortionMap[_y * this->dataPtr->distortionTexWidth + _x];
  return res;
}

//////////////////////////////////////////////////
void OgreDistortionPass::CalculateAndApplyDistortionScale()
{
  if (this->dataPtr->distortionMaterial.isNull())
    return;

  // Scale up image if cropping enabled and valid
  if (this->dataPtr->distortionCrop && this->k1 < 0)
  {
    float viewportWidth = static_cast<float>(this->ogreCamera->getViewport()->
        getActualWidth());
    float viewportHeight = static_cast<float>(this->ogreCamera->getViewport()->
        getActualHeight());

    unsigned int texSize = static_cast<unsigned int>(viewportHeight >
        viewportWidth ? viewportHeight : viewportWidth);

    const double fov = viewportHeight > viewportWidth ?
        this->ogreCamera->getFOVy().valueRadians() :
        (this->ogreCamera->getFOVy().valueRadians() *
        this->ogreCamera->getAspectRatio());

    const double focalLength = texSize / (2 * tan(fov / 2));

    // I believe that if not used with a square distortion texture, this
    // calculation will result in stretching of the final output image.
    ignition::math::Vector2d boundA = this->Distort(
        ignition::math::Vector2d(0, 0),
        this->lensCenter,
        this->k1, this->k2, this->k3,
        this->p1, this->p2, this->dataPtr->distortionTexWidth,
        focalLength);
    ignition::math::Vector2d boundB = this->Distort(
        ignition::math::Vector2d(1, 1),
        this->lensCenter,
        this->k1, this->k2, this->k3,
        this->p1, this->p2, this->dataPtr->distortionTexWidth,
        focalLength);
    ignition::math::Vector2d newScale = boundB - boundA;
    // If distortionScale is extremely small, don't crop
    if (newScale.X() < 1e-7 || newScale.Y() < 1e-7)
    {
       ignerr << "Distortion model attempted to apply a scale parameter of ("
             << this->dataPtr->distortionScale.X() << ", "
             << this->dataPtr->distortionScale.Y()
             << ", which is invalid.\n";
    }
    else
      this->dataPtr->distortionScale = newScale;
  }
  // Otherwise no scaling
  else
    this->dataPtr->distortionScale = ignition::math::Vector2d(1, 1);
}

IGN_RENDERING_REGISTER_RENDER_PASS(OgreDistortionPass, DistortionPass)
