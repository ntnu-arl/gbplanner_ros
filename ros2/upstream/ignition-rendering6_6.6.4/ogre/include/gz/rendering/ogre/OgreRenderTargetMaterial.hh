/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE_OGRRENDERTARGETMATERIAL_HH_
#define GZ_RENDERING_OGRE_OGRRENDERTARGETMATERIAL_HH_

#include <vector>

#include "gz/rendering/config.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/Export.hh"

// Ogre::MaterialManager::Listener isn't a dll-interface class, this may cause
// issues
#ifdef _MSC_VER
  #pragma warning(push)
  #pragma warning(disable:4275)
#endif

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /// \brief Causes all objects in a scene to be rendered with the same
    /// material when rendered by a given RenderTarget.
    /// \internal
    ///
    /// On construction it registers as an Ogre::RenderTargetListener
    /// on the provided Ogre::RenderTarget, and sets the material scheme name
    /// to a value that is unlikely to exist.
    /// When the target is about to be rendered it adds itself as an
    /// Ogre::MaterialManager::Listener.
    /// Every time ogre tries to get a technique for a material it will call
    /// handleSchemeNotFound which returns the first supported technique on the
    /// material provided to this class's constructor.
    class IGNITION_RENDERING_OGRE_VISIBLE  OgreRenderTargetMaterial :
      public Ogre::RenderTargetListener,
      public Ogre::MaterialManager::Listener
    {
      /// \brief constructor
      /// \param[in] _scene the scene manager responsible for rendering
      /// \param[in] _renderTarget the RenderTarget this should apply to
      /// \param[in] _material the material to apply to all renderables
      public: OgreRenderTargetMaterial(OgreScenePtr _scene,
          Ogre::RenderTarget *_renderTarget, Ogre::Material *_material);

      /// \brief destructor
      public: ~OgreRenderTargetMaterial();

      /// \brief Callback when a render target is about to be rendered
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      private: virtual void preRenderTargetUpdate(
          const Ogre::RenderTargetEvent &_evt) override;

      /// \brief Callback when a render target is finisned being rendered
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      private: virtual void postRenderTargetUpdate(
          const Ogre::RenderTargetEvent &_evt) override;

      /// \brief Ogre callback that assigned same material to all renderables
      /// when the requested scheme is not found
      /// \param[in] _schemeIndex Index of scheme requested
      /// \param[in] _schemeName Name of scheme requested
      /// \param[in] _originalMaterial Orignal material that does not contain
      /// the requested scheme
      /// \param[in] _lodIndex The material level-of-detail
      /// \param[in] _rend Pointer to the Ogre::Renderable object requesting
      /// the use of the techinique
      /// \return The Ogre material technique to use when scheme is not found.
      public: virtual Ogre::Technique *handleSchemeNotFound(
                  uint16_t _schemeIndex, const Ogre::String &_schemeName,
                  Ogre::Material *_originalMaterial, uint16_t _lodIndex,
                  const Ogre::Renderable *_rend) override;

      /// \brief scene manager responsible for rendering
      private: OgreScenePtr scene;

      /// \brief render target that should see a uniform material
      private: Ogre::RenderTarget *renderTarget;

      /// \brief material that should be applied to all objects
      private: Ogre::Material *material;

      /// \brief name of the material scheme used by this applicator
      private: Ogre::String schemeName;
    };
    }
  }
}

#ifdef _MSC_VER
  #pragma warning(pop)
#endif

#endif
