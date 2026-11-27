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

#ifndef IGNITION_RENDERING_OGRE2_OGRE2BBOXMATERIALSWITCHER_HH_
#define IGNITION_RENDERING_OGRE2_OGRE2BBOXMATERIALSWITCHER_HH_

#include <map>
#include <string>

#include "ignition/rendering/config.hh"
#include "ignition/rendering/ogre2/Export.hh"
#include "ignition/rendering/ogre2/Ogre2RenderTypes.hh"
#include "ignition/rendering/ogre2/Ogre2BoundingBoxCamera.hh"

namespace ignition
{
namespace rendering
{
inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {

/// \brief Helper class to assign ogre Id & label to each pixels
/// used in checking bounding boxes visibility in case of full mode
/// and to get bounding boxes boundaries in case of visible mode
// class BoundingBoxMaterialSwitcher : public Ogre::RenderTargetListener
class Ogre2BoundingBoxMaterialSwitcher : public Ogre::Camera::Listener
{
  /// \brief Constructor
  public: explicit Ogre2BoundingBoxMaterialSwitcher(Ogre2ScenePtr _scene);

  /// \brief Destructor
  public: ~Ogre2BoundingBoxMaterialSwitcher();

  /// \brief Get the top level model visual of a particular visual
  /// \param[in] _visual The visual who's top level model visual we are
  /// interested in
  /// \return The top level model visual of _visual
  private: VisualPtr TopLevelModelVisual(VisualPtr _visual) const;

  /// \brief Callback when a camara is about to be rendered
  /// \param[in] _cam Ogre camera pointer which is about to render
  private: virtual void cameraPreRenderScene(
    Ogre::Camera * _cam) override;

  /// \brief Callback when a camera is finisned being rendered
  /// \param[in] _cam Ogre camera pointer which has already render
  private: virtual void cameraPostRenderScene(
    Ogre::Camera * _cam) override;

  /// \brief A map of ogre sub item pointer to their original hlms material
  private: std::map<Ogre::SubItem *, Ogre::HlmsDatablock *> datablockMap;

  /// \brief Ogre v1 material consisting of a shader that changes the
  /// appearance of item to use a unique color for mouse picking
  private: Ogre::MaterialPtr plainMaterial;

  /// \brief Ogre v1 material consisting of a shader that changes the
  /// appearance of item to use a unique color for mouse picking. In
  /// addition, the depth check and depth write properties disabled.
  private: Ogre::MaterialPtr plainOverlayMaterial;

  /// \brief User Data Key to set the label
  private: const std::string labelKey = "label";

  /// \brief Label for background pixels in the ogre Ids map
  private: uint32_t backgroundLabel {255};

  /// \brief Map ogre ID to the top parent name of the item.
  /// used in multi-link models, key: ogreId, value: parent name
  private: std::map<uint32_t, std::string> ogreIdName;

  /// \brief Ogre2 Scene
  private: Ogre2ScenePtr scene;

  friend class Ogre2BoundingBoxCamera;
};
}
}  // namespace rendering
}  // namespace ignition
#endif
