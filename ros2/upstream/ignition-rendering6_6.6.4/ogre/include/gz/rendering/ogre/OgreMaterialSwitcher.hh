/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GZ_RENDERING_OGRE_OGREMATERIALSWITCHER_HH_
#define GZ_RENDERING_OGRE_OGREMATERIALSWITCHER_HH_

#include <map>
#include <string>

#include <gz/common/SuppressWarning.hh>
#include <gz/math/Color.hh>
#include "gz/rendering/config.hh"
#include "gz/rendering/ogre/Export.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declarations
    class OgreSelectionBuffer;

    /// \brief Helper class to assign unique colors to renderables
    class IGNITION_RENDERING_OGRE_VISIBLE OgreMaterialSwitcher :
// Ogre::MaterialManager::Listener isn't a dll-interface class, this may cause
// issues
#ifdef _MSC_VER
  #pragma warning(push)
  #pragma warning(disable:4275)
#endif
      public Ogre::MaterialManager::Listener,
#ifdef _MSC_VER
  #pragma warning(pop)
#endif
      Ogre::RenderTargetListener
    {
      /// \brief Constructor
      public: OgreMaterialSwitcher();

      /// \brief Destructor
      public: ~OgreMaterialSwitcher();

      /// \brief Get the entity with a specific color
      /// \param[in] _color The entity's color.
      public: std::string EntityName(
              const gz::math::Color &_color) const;

      /// \brief Reset the color value incrementor
      public: void Reset();

      /// \brief Ogre callback that assigns colors to new renderables when the
      /// requested scheme is not found
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
                  const Ogre::Renderable *_rend);

      /// \brief Ogre's pre render update callback
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      public: virtual void preRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      /// \brief Ogre's post render update callback
      /// \param[in] _evt Ogre render target event containing information about
      /// the source render target.
      public: virtual void postRenderTargetUpdate(
                  const Ogre::RenderTargetEvent &_evt);

      /// \brief Current unique color value
      private: gz::math::Color currentColor;

      /// \brief last entity assigned an unique color value
      private: std::string lastEntity;

      /// \brief last technique assigned to an entity.
      private: Ogre::Technique *lastTechnique = nullptr;

      /// \brief Color dictionary that maps the unique color value to
      /// renderable name
      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      private: std::map<unsigned int, std::string> colorDict;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief Increment unique color value that will be assigned to the
      /// next renderable
      private: void NextColor();

      /// \brief Selection Buffer class that make use of this class for
      /// selecting entitiies
      public: friend class OgreSelectionBuffer;

      /// \brief Plain material technique
      private: Ogre::Technique *plainTechnique = nullptr;

      /// \brief Overlay material technique
      private: Ogre::Technique *overlayTechnique = nullptr;
    };
    }
  }
}
#endif
