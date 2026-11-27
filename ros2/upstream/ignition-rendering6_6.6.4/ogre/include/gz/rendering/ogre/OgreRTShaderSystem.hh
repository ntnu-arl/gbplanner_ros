/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef GZ_RENDERING_OGRE_OGRERTSHADERSYSTEM_HH_
#define GZ_RENDERING_OGRE_OGRERTSHADERSYSTEM_HH_

#include <memory>
#include <string>

#include <gz/common/SingletonT.hh>
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/Export.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    // forward declaration
    class OgreRTShaderSystemPrivate;

    /// \addtogroup ign_rendering
    /// \{

    /// \class OgreRTShaderSystem OgreRTShaderSystem.hh rendering/rendering.hh
    /// \brief Implements Ogre's Run-Time Shader system.
    ///
    /// This class allows Gazebo to generate per-pixel shaders for every
    /// material at run-time.
    class IGNITION_RENDERING_OGRE_VISIBLE OgreRTShaderSystem :
      public common::SingletonT<OgreRTShaderSystem>
    {
      /// \enum LightingModel
      /// \brief The type of lighting
      public: enum LightingModel
              {
                /// \brief Per-Vertex lighting: best performance.
                SSLM_PerVertexLighting,
                /// \brief Per-Pixel lighting: best look.
                SSLM_PerPixelLighting,
                /// \brief Normal Map lighting: lighting calculations have
                /// been stored in a light map (texture) using tangent space.
                SSLM_NormalMapLightingTangentSpace,
                /// \brief Normal Map lighting: lighting calculations have
                /// been stored in a light map (texture) using object space.
                SSLM_NormalMapLightingObjectSpace
              };

      /// \brief Constructor.
      private: OgreRTShaderSystem();

      /// \brief Destructor.
      private: virtual ~OgreRTShaderSystem();

      /// \brief Initialize the run time shader system.
      /// \return True if the run time shader system is initialized
      /// successfully.
      public: bool Init();

      /// \brief Finalize the shader system
      public: void Fini();

      /// \brief Clear the shader system
      public: void Clear();

      /// \brief Add a scene manager
      /// \param[in] _scene The scene to process
      public: void AddScene(OgreScenePtr _scene);

      /// \brief Remove a scene
      /// \param[in] _scene The scene to remove
      public: void RemoveScene(OgreScenePtr _scene);

      /// \brief Remove a scene
      /// \param[in] _scene Name of the scene to remove.
      public: void RemoveScene(const std::string &_scene);

      /// \brief Update the shaders. This should not be called frequently.
      public: void UpdateShaders();

      /// \brief Set an Ogre::Entity to use RT shaders.
      /// \param[in] _vis OgreSubMesh that will use the OgreRTShaderSystem.
      public: void AttachEntity(OgreSubMesh *_vis);

      /// \brief Remove and entity.
      /// \param[in] _vis Remove this visual.
      public: void DetachEntity(OgreSubMesh *_vis);

      /// \brief Set a viewport to use shaders.
      /// \param[in] _viewport The viewport to add.
      /// \param[in] _scene The scene that the viewport uses.
      public: static void AttachViewport(Ogre::Viewport *_viewport,
                                         OgreScenePtr _scene);

      /// \brief Set a viewport to not use shaders.
      /// \param[in] _viewport The viewport to remove.
      /// \param[in] _scene The scene that the viewport uses.
      public: static void DetachViewport(Ogre::Viewport *_viewport,
                                         OgreScenePtr _scene);

      /// \brief Set the lighting model to per pixel or per vertex.
      /// \param[in] _set True means to use per-pixel shaders.
      public: void SetPerPixelLighting(bool _set);

      /// \brief Generate shaders for an entity
      /// \param[in] _subMesh The submesh to remove shaders for.
      public: void RemoveShaders(OgreSubMesh *_subMesh);

      /// \brief Generate shaders for an entity
      /// \param[in] _subMesh The submesh to generate shaders for.
      public: void GenerateShaders(OgreSubMesh *_subMesh);

      /// \brief Apply shadows to a scene.
      /// \param[in] _scene The scene to receive shadows.
      public: void ApplyShadows(OgreScenePtr _scene);

      /// \brief Remove shadows from a scene.
      /// \param[in] _scene The scene to remove shadows from.
      public: void RemoveShadows(OgreScenePtr _scene);

      /// \brief Get the Ogre PSSM Shadows camera setup.
      /// \return The Ogre PSSM Shadows camera setup.
      public: Ogre::PSSMShadowCameraSetup *PSSMShadowCameraSetup() const;

      /// \brief Get paths for the shader system
      /// \param[out] _coreLibsPath Path to the core libraries.
      /// \param[out] _cachePath Path to where the generated shaders are
      /// stored.
      private: bool Paths(std::string &_coreLibsPath,
                             std::string &_cachePath);

      /// \brief Set the shadow texture size.
      /// \param[in] _size Size of shadow texture to set to. This must be a
      /// power of 2. The default size is 1024.
      /// \return True if size is set successfully, false otherwise.
      public: bool SetShadowTextureSize(const unsigned int _size);

      /// \brief Get the shadow texture size.
      /// \return Size of the shadow texture. The default size is 1024.
      public: unsigned int ShadowTextureSize() const;

      /// \brief Get if RTShaderSystem is initialized or not
      /// \return True if intialized.
      public: bool IsInitialized() const;

      /// \brief Update the RT shaders. The call will only take effect if
      /// shadow properties changed, e.g. texture size, or the shaders
      /// have been marked dirty by UpdateShaders, e.g. when entities are
      /// added. The function reapplies shadows if properties have changed,
      /// and iterates through all entities added to RTShaderSystem
      /// and regenerates shader programs for each entity if shaders are dirty.
      /// This function is currently called by OgreScene::PreRender
      /// \sa OgreScene::PreRender
      public: void Update();

      /// \brief Make the RTShader system a singleton.
      private: friend class common::SingletonT<OgreRTShaderSystem>;

      /// \brief Pointer to private data class
      private: std::unique_ptr<OgreRTShaderSystemPrivate> dataPtr;
    };
    /// \}
    }
  }
}
#endif
