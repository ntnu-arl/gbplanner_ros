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
#ifndef IGNITION_RENDERING_OGRE2_OGRE2PARTICLENOISELISTENER_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2PARTICLENOISELISTENER_HH_

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Helper class for updating particle noise params
    class Ogre2ParticleNoiseListener : public Ogre::Camera::Listener
    {
      /// \brief constructor
      /// \param[in] _scene the scene manager responsible for rendering
      public: Ogre2ParticleNoiseListener(Ogre2ScenePtr _scene,
        Ogre::MaterialPtr _ogreMaterial);

      /// \brief destructor
      public: ~Ogre2ParticleNoiseListener() = default;

      /// \brief Callback when a camera is about to be rendered
      /// \param[in] _cam Ogre render camara which is about to render.
      private: virtual void cameraPreRenderScene(
          Ogre::Camera *_cam) override;

      /// \brief Pointer to scene
      private: Ogre2ScenePtr scene;

      /// \brief Pointer to ogre matieral with shaders for applying particle
      /// scattering effect to sensors
      private: Ogre::MaterialPtr ogreMaterial;

      /// \brief Particle scatter ratio. This is used to determine the ratio of
      /// particles that will be detected by sensors. Increasing the ratio
      /// increases the scatter of the particles, which means there is a higher
      /// chance of particles reflecting and interfering with depth sensing,
      /// making the emitter appear more dense. Decreasing the ratio decreases
      /// the scatter of the particles, making it appear less dense. This value
      /// should be > 0.
      private: float particleScatterRatio = 0.65f;
    };
    }
  }
}

#endif
