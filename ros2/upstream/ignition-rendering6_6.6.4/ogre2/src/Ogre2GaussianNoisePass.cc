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


#include <gz/common/Console.hh>

#include "gz/rendering/RenderPassSystem.hh"
#include "gz/rendering/ogre2/Ogre2GaussianNoisePass.hh"
#include "gz/rendering/ogre2/Ogre2RenderEngine.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <Compositor/OgreCompositorManager2.h>
#include <Compositor/OgreCompositorNodeDef.h>
#include <Compositor/Pass/PassQuad/OgreCompositorPassQuadDef.h>
#include <OgreMaterial.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreRoot.h>
#include <OgreTechnique.h>
#include <OgreVector3.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

/// \brief Private data for the Ogre2GaussianNoisePass class
class gz::rendering::Ogre2GaussianNoisePassPrivate
{
  /// brief Pointer to the Gaussian noise ogre material
  public: Ogre::Material *gaussianNoiseMat = nullptr;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2GaussianNoisePass::Ogre2GaussianNoisePass()
  : dataPtr(std::make_unique<Ogre2GaussianNoisePassPrivate>())
{
}

//////////////////////////////////////////////////
Ogre2GaussianNoisePass::~Ogre2GaussianNoisePass()
{
}

//////////////////////////////////////////////////
void Ogre2GaussianNoisePass::PreRender()
{
  if (!this->dataPtr->gaussianNoiseMat)
    return;

  if (!this->enabled)
    return;

  // modify material here (wont alter the base material!), called for
  // every drawn geometry instance (i.e. compositor render_quad)

  // Sample three values within the range [0,1.0] and set them for use in
  // the fragment shader, which will interpret them as offsets from (0,0)
  // to use when computing pseudo-random values.
  Ogre::Vector3 offsets(math::Rand::DblUniform(0.0, 1.0),
                        math::Rand::DblUniform(0.0, 1.0),
                        math::Rand::DblUniform(0.0, 1.0));
  // These calls are setting parameters that are declared in two places:
  // 1. media/materials/scripts/gaussian_noise.material, in
  //    fragment_program GaussianNoiseFS
  // 2. media/materials/scripts/gaussian_noise_fs.glsl
  Ogre::Pass *pass =
      this->dataPtr->gaussianNoiseMat->getTechnique(0)->getPass(0);
  Ogre::GpuProgramParametersSharedPtr psParams =
      pass->getFragmentProgramParameters();
  psParams->setNamedConstant("offsets", offsets);
  psParams->setNamedConstant("mean", static_cast<Ogre::Real>(this->mean));
  psParams->setNamedConstant("stddev",
      static_cast<Ogre::Real>(this->stdDev));
}

//////////////////////////////////////////////////
void Ogre2GaussianNoisePass::CreateRenderPass()
{
  static int gaussianNodeCounter = 0;

  auto engine = Ogre2RenderEngine::Instance();
  auto ogreRoot = engine->OgreRoot();
  Ogre::CompositorManager2 *ogreCompMgr = ogreRoot->getCompositorManager2();

  std::string nodeDefName = "GaussianNoiseNodeNode_"
      + std::to_string(gaussianNodeCounter);

  if (ogreCompMgr->hasNodeDefinition(nodeDefName))
    return;

  // The GaussianNoise material is defined in script (gaussian_noise.material).
  // clone the material
  std::string matName = "GaussianNoise";
  Ogre::MaterialPtr ogreMat =
      Ogre::MaterialManager::getSingleton().getByName(matName);
  if (!ogreMat)
  {
    ignerr << "Gaussian noise material not found: '" << matName << "'"
           << std::endl;
    return;
  }
  if (!ogreMat->isLoaded())
    ogreMat->load();
  std::string materialName = matName + "_" +
      std::to_string(gaussianNodeCounter);
  this->dataPtr->gaussianNoiseMat = ogreMat->clone(materialName).get();

  // create the compostior node definition

  // We need to programmatically create the compositor because we need to
  // configure it to use the cloned gaussian material created earlier.
  // The compositor workspace definition is equivalent to the following
  // ogre compositor script:
  // compositor_node GaussianNoiseNode
  // {
  //   // render texture input from previous render pass
  //   in 0 rt_input
  //   // render texture output to be passed to next render pass
  //   in 1 rt_output
  //
  //   // Only one target pass is needed.
  //   // rt_input is used as input to this pass and result is stored
  //   // in rt_output
  //   target rt_output
  //   {
  //     pass render_quad
  //     {
  //       material GaussianNoise // Use copy instead of original
  //       input 0 rt_input
  //     }
  //   }
  //   // pass the result to the next render pass
  //   out 0 rt_output
  //   // pass the rt_input render texture to the next render pass
  //   // where the texture is reused to store its result
  //   out 1 rt_input
  // }

  this->ogreCompositorNodeDefName = nodeDefName;
  gaussianNodeCounter++;

  Ogre::CompositorNodeDef *nodeDef =
      ogreCompMgr->addNodeDefinition(nodeDefName);

  // Input texture
  nodeDef->addTextureSourceName("rt_input", 0,
      Ogre::TextureDefinitionBase::TEXTURE_INPUT);
  nodeDef->addTextureSourceName("rt_output", 1,
      Ogre::TextureDefinitionBase::TEXTURE_INPUT);

  // rt_input target
  nodeDef->setNumTargetPass(1);
  Ogre::CompositorTargetDef *inputTargetDef =
      nodeDef->addTargetPass("rt_output");
  inputTargetDef->setNumPasses(1);
  {
    // quad pass
    Ogre::CompositorPassQuadDef *passQuad =
        static_cast<Ogre::CompositorPassQuadDef *>(
        inputTargetDef->addPass(Ogre::PASS_QUAD));
    passQuad->mMaterialName = materialName;
    passQuad->addQuadTextureSource(0, "rt_input");
  }
  nodeDef->mapOutputChannel(0, "rt_output");
  nodeDef->mapOutputChannel(1, "rt_input");
}

IGN_RENDERING_REGISTER_RENDER_PASS(Ogre2GaussianNoisePass, GaussianNoisePass)
