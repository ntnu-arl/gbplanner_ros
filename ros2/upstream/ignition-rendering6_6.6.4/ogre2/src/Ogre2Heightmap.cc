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

#include <chrono>

#include <ignition/common/Console.hh>
#include <ignition/common/Util.hh>

#include "ignition/rendering/ogre2/Ogre2Heightmap.hh"
#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2Light.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"

#include "Terra/Terra.h"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreCamera.h>
#include <OgreHlms.h>
#include <OgreHlmsManager.h>
#include <OgreImage2.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include "Terra/Hlms/OgreHlmsTerra.h"
#include "Terra/Hlms/OgreHlmsTerraDatablock.h"
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

//////////////////////////////////////////////////
class ignition::rendering::Ogre2HeightmapPrivate
{
  /// \brief Skirt min height. Leave it at -1 for automatic.
  /// Leave it at 0 for maximum skirt size (high performance hit)
  public: float skirtMinHeight{-1};

  /// \brief Cached value of skirtMinHeight auto-calculated by Terra
  /// so we can use it if skirtMinHeight becomes -1 again
  public: float autoSkirtValue;

  /// \brief The raw height values.
  public: std::vector<float> heights;

  /// \brief Size of the heightmap data.
  public: unsigned int dataSize{0u};

  /// \brief Pointer to ogre terra object
  public: std::unique_ptr<Ogre::Terra> terra{nullptr};
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2Heightmap::Ogre2Heightmap(const HeightmapDescriptor &_desc)
    : BaseHeightmap(_desc), dataPtr(std::make_unique<Ogre2HeightmapPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2Heightmap::~Ogre2Heightmap()
{
  this->DestroyImpl();
}

//////////////////////////////////////////////////
void Ogre2Heightmap::DestroyImpl()
{
  this->dataPtr->terra.reset();
}

//////////////////////////////////////////////////
void Ogre2Heightmap::Init()
{
  Ogre2Object::Init();

  if (this->descriptor.Data() == nullptr)
  {
    ignerr << "Failed to initialize: null heightmap data." << std::endl;
    return;
  }

  if (this->descriptor.Name().empty())
    this->descriptor.SetName(this->Name());

  // Add paths
  for (auto i = 0u; i < this->descriptor.TextureCount(); ++i)
  {
    auto texture = this->descriptor.TextureByIndex(i);
    Ogre2RenderEngine::Instance()->AddResourcePath(texture->Diffuse());
    Ogre2RenderEngine::Instance()->AddResourcePath(texture->Normal());
  }

  // \todo These parameters shouldn't be hardcoded, and instead parametrized so
  // that they can be made consistent across different libraries (like
  // ign-physics)
  bool flipY = false;
  // sampling size along image width and height
  const bool needsOgre1Compat =
      math::isPowerOfTwo(this->descriptor.Data()->Width() - 1u);
  const unsigned int srcWidth =
    needsOgre1Compat
      ? ((this->descriptor.Data()->Width() * this->descriptor.Sampling()) -
         this->descriptor.Sampling() + 1)
      : (this->descriptor.Data()->Width() * this->descriptor.Sampling());

  if (needsOgre1Compat)
  {
    ignwarn << "Heightmap final sampling should be 2^n"
           << std::endl << " which differs from ogre1's 2^n+1"
           << std::endl << "The last row and column will be cropped"
           << std::endl << "size = (width * sampling) - sampling + 1"
           << std::endl << "[" << srcWidth << "] = (["
           << this->descriptor.Data()->Width() << "] * ["
           << this->descriptor.Sampling() << "]) - ["
           << this->descriptor.Sampling() << "] + 1"
        << std::endl;
  }
  else if (!math::isPowerOfTwo(srcWidth))
  {
    ignerr << "Heightmap final sampling must satisfy 2^n."
           << std::endl << "size = width * sampling"
           << std::endl << "[" << srcWidth << "] = ["
           << this->descriptor.Data()->Width() << "] * ["
           << this->descriptor.Sampling() << "]"
        << std::endl;
    return;
  }

  const unsigned int newWidth =
    math::isPowerOfTwo(srcWidth) ? srcWidth : (srcWidth - 1u);

  math::Vector3d scale;
  scale.X(this->descriptor.Size().X() / newWidth);
  scale.Y(this->descriptor.Size().Y() / newWidth);
  scale.Z(1.0);

  // Construct the heightmap lookup table
  std::vector<float> lookup;
  this->descriptor.Data()->FillHeightMap(this->descriptor.Sampling(),
      srcWidth, this->descriptor.Size(), scale, flipY, lookup);
  this->dataPtr->heights.reserve(newWidth * newWidth);

  // Terra is optimized to work with UNORM heightmaps, therefore it assumes
  // lowest height is 0.
  // So we move the heightmap so that its min elevation = 0 before feeding to
  // ogre. It is later translated back by the setOrigin call.
  //
  // Obtain min and max elevation and bring everything to range [0; 1]
  // Terra should support non-normalized ranges but there are a couple
  // bugs preventing that, so it's just easier to normalize the data
  float minElevation = 0.0;
  float maxElevation = 0.0;

  for (unsigned int y = 0; y < newWidth; ++y)
  {
    for (unsigned int x = 0; x < newWidth; ++x)
    {
      const size_t index = y * srcWidth + x;
      float heightVal = lookup[index];

      // Sanity check in case we get NaNs from ign-common, this prevents a crash
      // in Ogre
      if (!std::isfinite(heightVal))
        heightVal = minElevation;

      minElevation = std::min(minElevation, heightVal);
      maxElevation = std::max(maxElevation, heightVal);
      this->dataPtr->heights.push_back(heightVal);
    }
  }

  // min and max elevations collected. Now normalize
  const float heightDiff = maxElevation - minElevation;
  const float invHeightDiff =
      fabsf( heightDiff ) < 1e-6f ? 1.0f : (1.0f / heightDiff);
  for (float &heightVal : this->dataPtr->heights)
  {
    heightVal = (heightVal - minElevation) * invHeightDiff;
    assert( heightVal >= 0 );
  }

  this->dataPtr->dataSize = newWidth;

  if (this->dataPtr->heights.empty())
  {
    ignerr << "Failed to load terrain. Heightmap data is empty" << std::endl;
    return;
  }

  // Create terrain group, which holds all the individual terrain instances.
  // Param 1: Pointer to the scene manager
  // Param 2: Alignment plane
  // Param 3: Number of vertices along one edge of the terrain (2^n+1).
  //          Terrains must be square, with each side a power of 2 in size
  // Param 4: World size of each terrain instance, in meters.

  auto ogreScene = std::dynamic_pointer_cast<Ogre2Scene>(this->Scene());

  Ogre::Image2 image;
  image.loadDynamicImage(this->dataPtr->heights.data(), newWidth, newWidth,
                         1u, Ogre::TextureTypes::Type2D,
                         Ogre::PFG_R32_FLOAT, false);

  const math::Vector3d newSize = this->descriptor.Size() *
                                 math::Vector3d(1.0, 1.0, heightDiff);

  // The position's Y sign ends up flipped
  math::Vector3d center(
      this->descriptor.Position().X(),
      -this->descriptor.Position().Y(),
      this->descriptor.Position().Z() + newSize.Z() * 0.5 + minElevation);

  Ogre::Root *ogreRoot = Ogre2RenderEngine::Instance()->OgreRoot();
  Ogre::SceneManager *ogreSceneManager = ogreScene->OgreSceneManager();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  // TODO(anyone): Gazebo doesn't support SCENE_STATIC scene nodes
  this->dataPtr->terra =
      std::make_unique<Ogre::Terra>(
        Ogre::Id::generateNewId<Ogre::MovableObject>(),
        &ogreSceneManager->_getEntityMemoryManager(
          Ogre::/*SCENE_STATIC*/SCENE_DYNAMIC),
        ogreSceneManager, 11u, ogreCompMgr, nullptr, true );
  // Does not cast shadows because it uses a raymarching implementation
  // instead of shadow maps. It does receive shadows from shadow maps though
  this->dataPtr->terra->setCastShadows(false);
  this->dataPtr->terra->load(
        image,
        Ogre2Conversions::Convert(center),
        Ogre2Conversions::Convert(newSize),
        this->descriptor.Name());
  this->dataPtr->autoSkirtValue =
      this->dataPtr->terra->getCustomSkirtMinHeight();
  this->dataPtr->terra->setDatablock(
        ogreRoot->getHlmsManager()->
        getHlms(Ogre::HLMS_USER3)->getDefaultDatablock());

  Ogre::Hlms *hlmsTerra =
          ogreRoot->getHlmsManager()->getHlms(Ogre::HLMS_USER3);

  IGN_ASSERT(dynamic_cast<Ogre::HlmsTerra*>(hlmsTerra),
             "HlmsTerra incorrectly setup, memory corrupted, or "
             "HlmsTerra::getType changed while this code is out of sync");

  Ogre::String datablockName = "IGN Terra " + this->name;

  Ogre::HlmsDatablock *datablockBase = hlmsTerra->createDatablock(
              datablockName, datablockName, Ogre::HlmsMacroblock(),
              Ogre::HlmsBlendblock(), Ogre::HlmsParamVec(), false);

  IGN_ASSERT(dynamic_cast<Ogre::HlmsTerraDatablock *>(datablockBase) != nullptr,
             "Corruption detected. This is impossible.");

  Ogre::HlmsTerraDatablock *datablock =
          static_cast<Ogre::HlmsTerraDatablock *>(datablockBase);

  Ogre::HlmsSamplerblock samplerblock;
  samplerblock.setAddressingMode(Ogre::TAM_WRAP);
  samplerblock.setFiltering(Ogre::TFO_ANISOTROPIC);
  samplerblock.mMaxAnisotropy = 8u;

  size_t numTextures = static_cast<size_t>(this->descriptor.TextureCount());

  if (numTextures >= 1u)
  {
    bool bCanUseFirstAsBase = false;

    using namespace Ogre;
    const HeightmapTexture *texture0 = this->descriptor.TextureByIndex(0);
    if (texture0->Normal().empty() &&
        abs(newSize.X() - texture0->Size()) < 1e-6 &&
        abs(newSize.Y() - texture0->Size()) < 1e-6 )
    {
      bCanUseFirstAsBase = true;
    }

    if ((numTextures > 4u && !bCanUseFirstAsBase) ||
        (numTextures > 5u && bCanUseFirstAsBase))
    {
      ignwarn << "Ogre2Heightmap currently supports up to 4 textures, "
                 "5 textures if the first one is diffuse-only & "
                 "texture size = terrain size. "
                 "The rest are ignored. Supplied: "
              << numTextures << std::endl;
      numTextures = bCanUseFirstAsBase ? 5u : 4u;
    }

    if (bCanUseFirstAsBase)
    {
      datablock->setTexture(static_cast<TerraTextureTypes>(TERRA_DIFFUSE),
                            texture0->Diffuse(), &samplerblock);
    }
    else
    {
      datablock->setTexture(static_cast<TerraTextureTypes>(TERRA_DETAIL0),
                            texture0->Diffuse(), &samplerblock);

      datablock->setTexture(static_cast<TerraTextureTypes>(TERRA_DETAIL0_NM),
                            texture0->Normal(), &samplerblock);

      const float sizeX =
              static_cast<float>(newSize.X() / texture0->Size());
      const float sizeY =
              static_cast<float>(newSize.Y() / texture0->Size());
      if (!texture0->Diffuse().empty() || !texture0->Normal().empty())
        datablock->setDetailMapOffsetScale(0, Vector4(0, 0, sizeX, sizeY));
    }

    for (size_t i = 1u; i < numTextures; ++i)
    {
      const size_t idxOffset = bCanUseFirstAsBase ? 1 : 0;
      const HeightmapTexture *texture = this->descriptor.TextureByIndex(i);

      datablock->setTexture(static_cast<TerraTextureTypes>(
                            TERRA_DETAIL0 + i - idxOffset),
                            texture->Diffuse(), &samplerblock);

      datablock->setTexture(static_cast<TerraTextureTypes>(
                            TERRA_DETAIL0_NM + i - idxOffset),
                            texture->Normal(), &samplerblock);

      const float sizeX =
              static_cast<float>(newSize.X() / texture->Size());
      const float sizeY =
              static_cast<float>(newSize.Y() / texture->Size());
      if (!texture->Diffuse().empty() || !texture->Normal().empty())
      {
          datablock->setDetailMapOffsetScale(
                      static_cast<uint8_t>(i - idxOffset),
                      Vector4(0, 0, sizeX, sizeY));
      }
    }


    size_t numBlends = static_cast<size_t>(this->descriptor.BlendCount());
    if ((numBlends > 3u && !bCanUseFirstAsBase) ||
        (numBlends > 4u && bCanUseFirstAsBase))
    {
      ignwarn << "Ogre2Heightmap currently supports up to 3 blends, "
                 "4 blends if the first one is diffuse-only & "
                 "texture size = terrain size. "
                 "The rest are ignored. Supplied: "
                 << numBlends << std::endl;
      numBlends = bCanUseFirstAsBase ? 4u : 3u;
    }

    Ogre::Vector4 minBlendHeights(0.0f);
    Ogre::Vector4 maxBlendHeights(0.0f);
    for (size_t i = 0; i < numBlends; ++i)
    {
      const size_t idxOffset = bCanUseFirstAsBase ? 0u : 1u;
      const HeightmapBlend *blend = this->descriptor.BlendByIndex(i);
      minBlendHeights[i + idxOffset] =
              static_cast<Ogre::Real>(blend->MinHeight());
      maxBlendHeights[i + idxOffset] =
              static_cast<Ogre::Real>(blend->MinHeight()+
                                      blend->FadeDistance());
    }
    datablock->setIgnWeightsHeights(minBlendHeights, maxBlendHeights);
  }

  this->dataPtr->terra->setDatablock(datablock);

  ignmsg << "Loading heightmap: " << this->descriptor.Name() << std::endl;
  auto time = std::chrono::steady_clock::now();

  ignmsg << "Heightmap loaded. Process took "
        <<  std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - time).count()
        << " ms." << std::endl;
}

//////////////////////////////////////////////////
void Ogre2Heightmap::PreRender()
{
}

///////////////////////////////////////////////////
void Ogre2Heightmap::UpdateForRender(Ogre::Camera *_activeCamera)
{
  if (this->dataPtr->skirtMinHeight >= 0)
  {
    this->dataPtr->terra->setCustomSkirtMinHeight(
          this->dataPtr->skirtMinHeight);
  }
  else
  {
    this->dataPtr->terra->setCustomSkirtMinHeight(
          this->dataPtr->autoSkirtValue);
  }

  // Get the first directional light
  Ogre2DirectionalLightPtr directionalLight;
  for (unsigned int i = 0; i < this->Scene()->LightCount(); ++i)
  {
     auto light = std::dynamic_pointer_cast<Ogre2DirectionalLight>(
         this->Scene()->LightByIndex(i));
    if (nullptr != light)
    {
      directionalLight = light;
      break;
    }
  }

  this->dataPtr->terra->setCamera(_activeCamera);
  if (directionalLight)
  {
    this->dataPtr->terra->update(
          Ogre2Conversions::Convert(directionalLight->Direction()));
  }
  else
  {
    this->dataPtr->terra->update(Ogre::Vector3::NEGATIVE_UNIT_Y);
  }
}

//////////////////////////////////////////////////
Ogre::MovableObject *Ogre2Heightmap::OgreObject() const
{
  return this->dataPtr->terra.get();
}

//////////////////////////////////////////////////
void Ogre2Heightmap::SetMaterial(MaterialPtr, bool)
{
  // no-op
}

//////////////////////////////////////////////////
MaterialPtr Ogre2Heightmap::Material() const
{
  return nullptr;
}

//////////////////////////////////////////////////
Ogre::Terra* Ogre2Heightmap::Terra()
{
  return this->dataPtr->terra.get();
}
