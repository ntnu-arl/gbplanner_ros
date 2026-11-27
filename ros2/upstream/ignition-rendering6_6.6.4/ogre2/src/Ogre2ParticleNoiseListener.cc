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

#include <string>

#include <ignition/math/Rand.hh>

#include "ignition/rendering/ogre2/Ogre2Includes.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Visual.hh"
#include "ignition/rendering/ogre2/Ogre2ParticleEmitter.hh"

#include "Ogre2ParticleNoiseListener.hh"

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2ParticleNoiseListener::Ogre2ParticleNoiseListener(
    Ogre2ScenePtr _scene, Ogre::MaterialPtr _ogreMaterial)
{
  this->scene = _scene;
  this->ogreMaterial = _ogreMaterial;
}

//////////////////////////////////////////////////
void Ogre2ParticleNoiseListener::cameraPreRenderScene(
    Ogre::Camera * _cam)
{
  // the code here is responsible for setting the depth variation of readings
  // returned by sensor in areas where particles are. It does so by adding
  // noise with high std dev values.
  // 1. Find first particle in the view of the sensor
  // 2. set the sensor noise for the particles to half the size of the
  // bounding box
  // \todo(anyone) noise std dev is set based on the first particle emitter the
  // sensor sees. Make this scale to multiple particle emitters!
  auto itor = this->scene->OgreSceneManager()->getMovableObjectIterator(
      Ogre::ParticleSystemFactory::FACTORY_TYPE_NAME);
  while (itor.hasMoreElements())
  {
    Ogre::MovableObject *object = itor.peekNext();
    Ogre::ParticleSystem *ps = dynamic_cast<Ogre::ParticleSystem *>(object);

    if (!ps)
    {
      itor.moveNext();
      continue;
    }

    Ogre::Aabb aabb = ps->getWorldAabbUpdated();
    if (std::isinf(aabb.getMinimum().length()) ||
        std::isinf(aabb.getMaximum().length()))
    {
      itor.moveNext();
      continue;
    }

    Ogre::AxisAlignedBox box = Ogre::AxisAlignedBox(aabb.getMinimum(),
        aabb.getMaximum());


    if (_cam->isVisible(box))
    {
      // set stddev to half of size of particle emitter aabb
      auto hs = box.getHalfSize() * 0.5;
      double particleStddev = hs.x;

      Ogre::Pass *pass = this->ogreMaterial->getTechnique(0)->getPass(0);
      Ogre::GpuProgramParametersSharedPtr psParams =
          pass->getFragmentProgramParameters();
      psParams->setNamedConstant("particleStddev",
          static_cast<float>(particleStddev));
      psParams->setNamedConstant("rnd",
          static_cast<float>(ignition::math::Rand::DblUniform(0.0, 1.0)));

      // get particle scatter ratio value from particle emitter user data
      // and pass that to the shaders
      float scatterRatio = 0.65f;
      Ogre::Any userAny = ps->getUserObjectBindings().getUserAny();
      if (!userAny.isEmpty() && userAny.getType() == typeid(unsigned int))
      {
        VisualPtr result;
        try
        {
          result = this->scene->VisualById(
              Ogre::any_cast<unsigned int>(userAny));
        }
        catch(Ogre::Exception &e)
        {
          ignerr << "Ogre Error:" << e.getFullDescription() << "\n";
        }
        Ogre2ParticleEmitterPtr emitterPtr =
          std::dynamic_pointer_cast<Ogre2ParticleEmitter>(result);
        if (emitterPtr)
          scatterRatio = emitterPtr->ParticleScatterRatio();
      }
      psParams->setNamedConstant("particleScatterRatio", scatterRatio);

      return;
    }
    itor.moveNext();
  }
}
