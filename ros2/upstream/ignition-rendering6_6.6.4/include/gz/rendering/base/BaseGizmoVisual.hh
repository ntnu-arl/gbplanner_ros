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

#ifndef GZ_RENDERING_BASE_BASEGIZMOVISUAL_HH_
#define GZ_RENDERING_BASE_BASEGIZMOVISUAL_HH_

#include <map>
#include <string>
#include <gz/common/MeshManager.hh>
#include <gz/common/SuppressWarning.hh>

#include "gz/rendering/base/BaseScene.hh"
#include "gz/rendering/base/BaseNode.hh"
#include "gz/rendering/ArrowVisual.hh"
#include "gz/rendering/Camera.hh"
#include "gz/rendering/GizmoVisual.hh"

namespace ignition
{
  namespace rendering
  {
    inline namespace IGNITION_RENDERING_VERSION_NAMESPACE {
    //
    /* \class BaseGizmoVisual BaseGizmoVisuall.hh \
     * ignition/rendering/base/BaseGizmoVisual.hh
     */
    /// \brief A base implementation of the GizmoVisual class
    template <class T>
    class BaseGizmoVisual :
      public virtual GizmoVisual,
      public virtual T
    {
      /// \brief Constructor
      protected: BaseGizmoVisual();

      /// \brief Destructor
      public: virtual ~BaseGizmoVisual();

      // Documentation inherited
      public: virtual void Init() override;

      // Documentation inherited
      public: virtual void PreRender() override;

      // Documentation inherited
      public: virtual void SetTransformMode(TransformMode _mode) override;

      // Documentation inherited
      public: virtual TransformMode Mode() const override;

      // Documentation inherited
      public: virtual void SetActiveAxis(const math::Vector3d &_axis) override;

      // Documentation inherited
      public: virtual math::Vector3d ActiveAxis() const override;

      // Documentation inherited
      public: virtual TransformAxis AxisById(unsigned int _id) const override;

      // Documentation inherited
      public: virtual VisualPtr ChildByAxis(unsigned int _axis) const override;

      /// \brief Reset the gizmo visual state
      public: virtual void Reset();

      /// \brief Create materials used by the gizmo visual
      protected: void CreateMaterials();

      /// \brief Create gizmo visual for translation
      protected: void CreateTranslationVisual();

      /// \brief Create gizmo visual for rotation
      protected: void CreateRotationVisual();

      /// \brief Create gizmo visual for scale
      protected: void CreateScaleVisual();

      /// \brief Current gizmo mode
      protected: TransformMode mode = TransformMode::TM_NONE;

      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief A map of gizmo axis and their visuals
      protected: std::map<unsigned int, VisualPtr> visuals;

      /// \brief A map of gizmo axis  and their handle visuals
      protected: std::map<unsigned int, VisualPtr> handles;

      /// \brief Currently active visual.
      protected: VisualPtr activeVis;

      /// \brief Flag to indicate the mode has changed.
      protected: bool modeDirty = false;

      /// \brief Active axis
      protected: math::Vector3d axis = math::Vector3d::Zero;

      /// \brief A map of axis enums to materials
      protected: std::map<unsigned int, MaterialPtr> materials;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING

      /// \brief Material used by axes
      protected: enum AxisMaterial
                 {
                   /// \brief X axis
                   AM_X,
                   /// \brief Y axis
                   AM_Y,
                   /// \brief Z axis
                   AM_Z,
                   /// \brief Active axis
                   AM_ACTIVE,
                   /// \brief Origin
                   AM_O,
                   /// \brief handle
                   AM_HANDLE
                 };

      /// \brief Only the scene can create a GizmoVisual
      private: friend class BaseScene;
    };

    //////////////////////////////////////////////////
    template <class T>
    BaseGizmoVisual<T>::BaseGizmoVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    BaseGizmoVisual<T>::~BaseGizmoVisual()
    {
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::Init()
    {
      T::Init();

      this->CreateMaterials();
      this->CreateTranslationVisual();
      this->CreateRotationVisual();
      this->CreateScaleVisual();

      for (auto v : this->visuals)
        v.second->SetVisible(false);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::Reset()
    {
      this->visuals[TransformAxis::TA_TRANSLATION_X]->SetMaterial(
          this->materials[AM_X], false);
      this->visuals[TransformAxis::TA_TRANSLATION_Y]->SetMaterial(
          this->materials[AM_Y], false);
      this->visuals[TransformAxis::TA_TRANSLATION_Z]->SetMaterial(
          this->materials[AM_Z], false);
      this->visuals[TransformAxis::TA_ROTATION_X]->SetMaterial(
          this->materials[AM_X], false);
      this->visuals[TransformAxis::TA_ROTATION_Y]->SetMaterial(
          this->materials[AM_Y], false);
      this->visuals[TransformAxis::TA_ROTATION_Z]->SetMaterial(
          this->materials[AM_Z], false);
      this->visuals[TransformAxis::TA_SCALE_X]->SetMaterial(
          this->materials[AM_X], false);
      this->visuals[TransformAxis::TA_SCALE_Y]->SetMaterial(
          this->materials[AM_Y], false);
      this->visuals[TransformAxis::TA_SCALE_Z]->SetMaterial(
          this->materials[AM_Z], false);

      this->handles[TransformAxis::TA_TRANSLATION_X]->SetMaterial(
          this->materials[AM_HANDLE], false);
      this->handles[TransformAxis::TA_TRANSLATION_Y]->SetMaterial(
          this->materials[AM_HANDLE], false);
      this->handles[TransformAxis::TA_TRANSLATION_Z]->SetMaterial(
          this->materials[AM_HANDLE], false);

      this->handles[TransformAxis::TA_ROTATION_X]->SetMaterial(
          this->materials[AM_HANDLE], false);
      this->handles[TransformAxis::TA_ROTATION_Y]->SetMaterial(
          this->materials[AM_HANDLE], false);
      this->handles[TransformAxis::TA_ROTATION_Z]->SetMaterial(
          this->materials[AM_HANDLE], false);

      this->handles[TransformAxis::TA_SCALE_X]->SetMaterial(
          this->materials[AM_HANDLE], false);
      this->handles[TransformAxis::TA_SCALE_Y]->SetMaterial(
          this->materials[AM_HANDLE], false);
      this->handles[TransformAxis::TA_SCALE_Z]->SetMaterial(
          this->materials[AM_HANDLE], false);

      for (auto v : this->visuals)
        v.second->SetVisible(false);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::PreRender()
    {
      if (!this->modeDirty)
        return;

      this->Reset();

      if (this->mode == TransformMode::TM_NONE)
      {
        this->modeDirty = false;
        return;
      }

      if (this->mode & TransformMode::TM_TRANSLATION)
      {
        this->visuals[TransformAxis::TA_TRANSLATION_X]->SetVisible(true);
        this->visuals[TransformAxis::TA_TRANSLATION_Y]->SetVisible(true);
        this->visuals[TransformAxis::TA_TRANSLATION_Z]->SetVisible(true);
        this->visuals[TransformAxis::TA_TRANSLATION_Z << 1]->SetVisible(true);
        if (this->axis.X() > 0)
        {
          this->visuals[TransformAxis::TA_TRANSLATION_X]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_TRANSLATION_X]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
        if (this->axis.Y() > 0)
        {
          this->visuals[TransformAxis::TA_TRANSLATION_Y]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_TRANSLATION_Y]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
        if (this->axis.Z() > 0)
        {
          this->visuals[TransformAxis::TA_TRANSLATION_Z]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_TRANSLATION_Z]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
      }
      else if (this->mode & TransformMode::TM_ROTATION)
      {
        this->visuals[TransformAxis::TA_ROTATION_X]->SetVisible(true);
        this->visuals[TransformAxis::TA_ROTATION_Y]->SetVisible(true);
        this->visuals[TransformAxis::TA_ROTATION_Z]->SetVisible(true);
        this->visuals[TransformAxis::TA_ROTATION_Z << 1]->SetVisible(true);
        if (this->axis.X() > 0)
        {
          this->visuals[TransformAxis::TA_ROTATION_X]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_ROTATION_X]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
        if (this->axis.Y() > 0)
        {
          this->visuals[TransformAxis::TA_ROTATION_Y]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_ROTATION_Y]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
        if (this->axis.Z() > 0)
        {
          this->visuals[TransformAxis::TA_ROTATION_Z]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_ROTATION_Z]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
      }
      else if (this->mode & TransformMode::TM_SCALE)
      {
        this->visuals[TransformAxis::TA_SCALE_X]->SetVisible(true);
        this->visuals[TransformAxis::TA_SCALE_Y]->SetVisible(true);
        this->visuals[TransformAxis::TA_SCALE_Z]->SetVisible(true);
        if (this->axis.X() > 0)
        {
          this->visuals[TransformAxis::TA_SCALE_X]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_SCALE_X]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
        if (this->axis.Y() > 0)
        {
          this->visuals[TransformAxis::TA_SCALE_Y]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_SCALE_Y]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
        if (this->axis.Z() > 0)
        {
          this->visuals[TransformAxis::TA_SCALE_Z]->SetMaterial(
              this->materials[AM_ACTIVE], false);
          this->handles[TransformAxis::TA_SCALE_Z]->SetMaterial(
              this->materials[AM_HANDLE], false);
        }
      }

      this->modeDirty = false;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::SetTransformMode(TransformMode _mode)
    {
      if (this->mode == _mode)
        return;

      this->mode = _mode;
      // clear active axis when mode changes
      this->axis = math::Vector3d::Zero;
      this->modeDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    TransformMode BaseGizmoVisual<T>::Mode() const
    {
      return this->mode;
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::SetActiveAxis(const math::Vector3d &_axis)
    {
      if (this->axis == _axis)
        return;

      this->axis = _axis;
      this->modeDirty = true;
    }

    //////////////////////////////////////////////////
    template <class T>
    math::Vector3d BaseGizmoVisual<T>::ActiveAxis() const
    {
      return this->axis;
    }

    //////////////////////////////////////////////////
    template <class T>
    TransformAxis BaseGizmoVisual<T>::AxisById(unsigned int _id) const
    {
      for (auto v : this->visuals)
      {
        // each axis visual has a child handle so also check children for
        // matching id
        if (v.second->Id() == _id || v.second->ChildById(_id))
          return static_cast<TransformAxis>(v.first);
      }
      return TransformAxis::TA_NONE;
    }


    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::CreateMaterials()
    {
      MaterialPtr xMat = this->Scene()->Material("Default/TransRed")->Clone();
      MaterialPtr yMat = this->Scene()->Material("Default/TransGreen")->Clone();
      MaterialPtr zMat = this->Scene()->Material("Default/TransBlue")->Clone();
      MaterialPtr activeMat =
          this->Scene()->Material("Default/TransYellow")->Clone();

      // disable depth checking and writing, make them overlays
      xMat->SetDepthWriteEnabled(false);
      xMat->SetDepthCheckEnabled(false);
      yMat->SetDepthWriteEnabled(false);
      yMat->SetDepthCheckEnabled(false);
      zMat->SetDepthWriteEnabled(false);
      zMat->SetDepthCheckEnabled(false);
      activeMat->SetDepthWriteEnabled(false);
      activeMat->SetDepthCheckEnabled(false);

      MaterialPtr oMat = this->Scene()->Material("GizmoGray");
      if (!oMat)
      {
        oMat = this->Scene()->CreateMaterial("GizmoGray");
        oMat->SetAmbient(0.5, 0.5, 0.5);
        oMat->SetDiffuse(0.5, 0.5, 0.5);
        oMat->SetEmissive(0.5, 0.5, 0.5);
        oMat->SetTransparency(0.5);
        oMat->SetCastShadows(false);
        oMat->SetReceiveShadows(false);
        oMat->SetLightingEnabled(false);
        oMat->SetDepthWriteEnabled(false);
        oMat->SetDepthCheckEnabled(false);
      }

      MaterialPtr handleMat = this->Scene()->Material("GizmoHandle");
      if (!handleMat)
      {
        handleMat = this->Scene()->CreateMaterial("GizmoHandle");
        handleMat->SetAmbient(0.0, 0.0, 0.0);
        handleMat->SetDiffuse(0.0, 0.0, 0.0);
        handleMat->SetEmissive(0.0, 0.0, 0.0);
        handleMat->SetTransparency(1.0);
        handleMat->SetCastShadows(false);
        handleMat->SetReceiveShadows(false);
        handleMat->SetLightingEnabled(false);
        handleMat->SetDepthWriteEnabled(false);
        handleMat->SetDepthCheckEnabled(false);
      }

      this->materials[AM_X] = xMat;
      this->materials[AM_Y] = yMat;
      this->materials[AM_Z] = zMat;
      this->materials[AM_ACTIVE] = activeMat;
      this->materials[AM_O] = oMat;
      this->materials[AM_HANDLE] = handleMat;
    }
    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::CreateTranslationVisual()
    {
      VisualPtr transVis = this->Scene()->CreateVisual();

      // trans x
      VisualPtr transXVis = this->Scene()->CreateVisual();
      VisualPtr transShaftXVis = this->Scene()->CreateVisual();
      transShaftXVis->AddGeometry(this->Scene()->CreateCylinder());
      transShaftXVis->SetOrigin(0, 0, 0.5);
      transShaftXVis->SetLocalPosition(0, 0, 0.5);
      transShaftXVis->SetLocalScale(0.02, 0.02, 0.45);
      transXVis->AddChild(transShaftXVis);

      VisualPtr transHeadXVis = this->Scene()->CreateVisual();
      transHeadXVis->AddGeometry(this->Scene()->CreateCone());
      transHeadXVis->SetOrigin(0, 0, -0.5);
      transHeadXVis->SetLocalPosition(0, 0, 0.5);
      transHeadXVis->SetLocalScale(0.07, 0.07, 0.2);
      transXVis->AddChild(transHeadXVis);

      transXVis->SetMaterial(this->materials[AM_X], false);
      transXVis->SetLocalRotation(0, IGN_PI * 0.5, 0);
      transVis->AddChild(transXVis);

      // trans y
      VisualPtr transYVis = this->Scene()->CreateVisual();
      VisualPtr transShaftYVis = this->Scene()->CreateVisual();
      transShaftYVis->AddGeometry(this->Scene()->CreateCylinder());
      transShaftYVis->SetOrigin(0, 0, 0.5);
      transShaftYVis->SetLocalPosition(0, 0, 0.5);
      transShaftYVis->SetLocalScale(0.02, 0.02, 0.45);
      transYVis->AddChild(transShaftYVis);

      VisualPtr transHeadYVis = this->Scene()->CreateVisual();
      transHeadYVis->AddGeometry(this->Scene()->CreateCone());
      transHeadYVis->SetOrigin(0, 0, -0.5);
      transHeadYVis->SetLocalPosition(0, 0, 0.5);
      transHeadYVis->SetLocalScale(0.07, 0.07, 0.2);
      transYVis->AddChild(transHeadYVis);

      transYVis->SetMaterial(this->materials[AM_Y], false);
      transYVis->SetLocalRotation(-IGN_PI * 0.5, 0, 0);
      transVis->AddChild(transYVis);

      // trans z
      VisualPtr transZVis = this->Scene()->CreateVisual();
      VisualPtr transShaftZVis = this->Scene()->CreateVisual();
      transShaftZVis->AddGeometry(this->Scene()->CreateCylinder());
      transShaftZVis->SetOrigin(0, 0, 0.5);
      transShaftZVis->SetLocalPosition(0, 0, 0.5);
      transShaftZVis->SetLocalScale(0.02, 0.02, 0.45);
      transZVis->AddChild(transShaftZVis);

      VisualPtr transHeadZVis = this->Scene()->CreateVisual();
      transHeadZVis->AddGeometry(this->Scene()->CreateCone());
      transHeadZVis->SetOrigin(0, 0, -0.5);
      transHeadZVis->SetLocalPosition(0, 0, 0.5);
      transHeadZVis->SetLocalScale(0.07, 0.07, 0.2);
      transZVis->AddChild(transHeadZVis);

      transZVis->SetMaterial(this->materials[AM_Z], false);
      transVis->AddChild(transZVis);

      // trans origin
      VisualPtr transOrigin = this->Scene()->CreateVisual();
      transOrigin->AddGeometry(this->Scene()->CreateSphere());
      transOrigin->SetLocalScale(0.05, 0.05, 0.05);
      transOrigin->SetMaterial(this->materials[AM_O], false);
      transVis->AddChild(transOrigin);

      this->visuals[TransformAxis::TA_TRANSLATION_X] = transXVis;
      this->visuals[TransformAxis::TA_TRANSLATION_Y] = transYVis;
      this->visuals[TransformAxis::TA_TRANSLATION_Z] = transZVis;
      // Store the translation origin visual in this->visuals using a key
      // that's not already occupied by the TransformAxis enum
      this->visuals[TransformAxis::TA_TRANSLATION_Z << 1] = transOrigin;

      // translation handles
      VisualPtr transHandleXVis = this->Scene()->CreateVisual();
      transHandleXVis->AddGeometry(this->Scene()->CreateCylinder());
      transHandleXVis->SetLocalPosition(0, 0, 0.35);
      transHandleXVis->SetLocalScale(0.11, 0.11, 0.7);
      transHandleXVis->SetMaterial(this->materials[AM_HANDLE], false);
      transXVis->AddChild(transHandleXVis);

      VisualPtr transHandleYVis = this->Scene()->CreateVisual();
      transHandleYVis->AddGeometry(this->Scene()->CreateCylinder());
      transHandleYVis->SetLocalPosition(0, 0, 0.35);
      transHandleYVis->SetLocalScale(0.11, 0.11, 0.7);
      transHandleYVis->SetMaterial(this->materials[AM_HANDLE], false);
      transYVis->AddChild(transHandleYVis);

      VisualPtr transHandleZVis = this->Scene()->CreateVisual();
      transHandleZVis->AddGeometry(this->Scene()->CreateCylinder());
      transHandleZVis->SetLocalPosition(0, 0, 0.35);
      transHandleZVis->SetLocalScale(0.11, 0.11, 0.7);
      transHandleZVis->SetMaterial(this->materials[AM_HANDLE], false);
      transZVis->AddChild(transHandleZVis);

      this->handles[TransformAxis::TA_TRANSLATION_X] = transHandleXVis;
      this->handles[TransformAxis::TA_TRANSLATION_Y] = transHandleYVis;
      this->handles[TransformAxis::TA_TRANSLATION_Z] = transHandleZVis;

      this->AddChild(transVis);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::CreateRotationVisual()
    {
      common::MeshManager *meshMgr = common::MeshManager::Instance();
      std::string rotMeshName = "gizmo_rotate";
      if (!meshMgr->HasMesh(rotMeshName))
        meshMgr->CreateTube(rotMeshName, 1.0f, 1.02f, 0.02f, 1, 64, IGN_PI);

      std::string rotFullMeshName = "gizmo_rotate_full";
      if (!meshMgr->HasMesh(rotFullMeshName))
      {
        meshMgr->CreateTube(rotFullMeshName, 1.0f, 1.02f, 0.02f, 1, 64,
            2 * IGN_PI);
      }

      std::string rotHandleMeshName = "gizmo_rotate_handle";
      if (!meshMgr->HasMesh(rotHandleMeshName))
      {
        meshMgr->CreateTube(rotHandleMeshName, 0.95f, 1.07f, 0.1f, 1, 64,
            IGN_PI);
      }

      VisualPtr rotVis = this->Scene()->CreateVisual();

      // rotation x
      VisualPtr rotXVis = this->Scene()->CreateVisual();
      rotXVis->AddGeometry(this->Scene()->CreateMesh(rotMeshName));
      rotXVis->SetLocalRotation(0, IGN_PI * 0.5, 0);
      rotXVis->SetLocalScale(0.5, 0.5, 0.5);
      rotXVis->SetMaterial(this->materials[AM_X], false);
      rotVis->AddChild(rotXVis);

      // rotation y
      VisualPtr rotYVis = this->Scene()->CreateVisual();
      rotYVis->AddGeometry(this->Scene()->CreateMesh(rotMeshName));
      rotYVis->SetLocalRotation(IGN_PI * 0.5, 0, 0);
      rotYVis->SetLocalScale(0.5, 0.5, 0.5);
      rotYVis->SetMaterial(this->materials[AM_Y], false);
      rotVis->AddChild(rotYVis);

      // rotation z
      VisualPtr rotZVis = this->Scene()->CreateVisual();
      rotZVis->AddGeometry(this->Scene()->CreateMesh(rotMeshName));
      rotZVis->SetLocalScale(0.5, 0.5, 0.5);
      rotZVis->SetMaterial(this->materials[AM_Z], false);
      rotVis->AddChild(rotZVis);

      // rotation origin
      VisualPtr rotFullVis = this->Scene()->CreateVisual();
      rotFullVis->AddGeometry(this->Scene()->CreateMesh(rotFullMeshName));
      rotFullVis->SetLocalScale(0.5, 0.5, 0.5);
      rotFullVis->SetMaterial(this->materials[AM_O], false);
      rotVis->AddChild(rotFullVis);

      this->visuals[TransformAxis::TA_ROTATION_X] = rotXVis;
      this->visuals[TransformAxis::TA_ROTATION_Y] = rotYVis;
      this->visuals[TransformAxis::TA_ROTATION_Z] = rotZVis;
      // Store the full rotation visual in this->visuals using a key
      // that's not already occupied by the TransformAxis enum
      this->visuals[TransformAxis::TA_ROTATION_Z << 1] = rotFullVis;

      // rotation handles
      VisualPtr rotHandleXVis = this->Scene()->CreateVisual();
      rotHandleXVis->AddGeometry(this->Scene()->CreateMesh(rotHandleMeshName));
      rotHandleXVis->SetMaterial(this->materials[AM_HANDLE], false);
      rotXVis->AddChild(rotHandleXVis);

      VisualPtr rotHandleYVis = this->Scene()->CreateVisual();
      rotHandleYVis->AddGeometry(this->Scene()->CreateMesh(rotHandleMeshName));
      rotHandleYVis->SetMaterial(this->materials[AM_HANDLE], false);
      rotYVis->AddChild(rotHandleYVis);

      VisualPtr rotHandleZVis = this->Scene()->CreateVisual();
      rotHandleZVis->AddGeometry(this->Scene()->CreateMesh(rotHandleMeshName));
      rotHandleZVis->SetMaterial(this->materials[AM_HANDLE], false);
      rotZVis->AddChild(rotHandleZVis);

      this->handles[TransformAxis::TA_ROTATION_X] = rotHandleXVis;
      this->handles[TransformAxis::TA_ROTATION_Y] = rotHandleYVis;
      this->handles[TransformAxis::TA_ROTATION_Z] = rotHandleZVis;

      this->AddChild(rotVis);
    }

    //////////////////////////////////////////////////
    template <class T>
    void BaseGizmoVisual<T>::CreateScaleVisual()
    {
      VisualPtr scaleVis = this->Scene()->CreateVisual();

      // scale x
      VisualPtr scaleXVis = this->Scene()->CreateVisual();
      VisualPtr scaleShaftXVis = this->Scene()->CreateVisual();
      scaleShaftXVis->AddGeometry(this->Scene()->CreateCylinder());
      scaleShaftXVis->SetOrigin(0, 0, 0.5);
      scaleShaftXVis->SetLocalPosition(0, 0, 0.5);
      scaleShaftXVis->SetLocalScale(0.02, 0.02, 0.5);
      scaleXVis->AddChild(scaleShaftXVis);

      VisualPtr scaleHeadXVis = this->Scene()->CreateVisual();
      scaleHeadXVis->AddGeometry(this->Scene()->CreateBox());
      scaleHeadXVis->SetOrigin(0, 0, -0.5);
      scaleHeadXVis->SetLocalPosition(0, 0, 0.5);
      scaleHeadXVis->SetLocalScale(0.07, 0.07, 0.07);
      scaleXVis->AddChild(scaleHeadXVis);

      scaleXVis->SetMaterial(this->materials[AM_X], false);
      scaleXVis->SetLocalRotation(0, IGN_PI * 0.5, 0);
      scaleVis->AddChild(scaleXVis);

      // scale y
      VisualPtr scaleYVis = this->Scene()->CreateVisual();
      VisualPtr scaleShaftYVis = this->Scene()->CreateVisual();
      scaleShaftYVis->AddGeometry(this->Scene()->CreateCylinder());
      scaleShaftYVis->SetOrigin(0, 0, 0.5);
      scaleShaftYVis->SetLocalPosition(0, 0, 0.5);
      scaleShaftYVis->SetLocalScale(0.02, 0.02, 0.5);
      scaleYVis->AddChild(scaleShaftYVis);

      VisualPtr scaleHeadYVis = this->Scene()->CreateVisual();
      scaleHeadYVis->AddGeometry(this->Scene()->CreateBox());
      scaleHeadYVis->SetOrigin(0, 0, -0.5);
      scaleHeadYVis->SetLocalPosition(0, 0, 0.5);
      scaleHeadYVis->SetLocalScale(0.07, 0.07, 0.07);
      scaleYVis->AddChild(scaleHeadYVis);

      scaleYVis->SetMaterial(this->materials[AM_Y], false);
      scaleYVis->SetLocalRotation(-IGN_PI * 0.5, 0, 0);
      scaleVis->AddChild(scaleYVis);

      // scale z
      VisualPtr scaleZVis = this->Scene()->CreateVisual();
      VisualPtr scaleShaftZVis = this->Scene()->CreateVisual();
      scaleShaftZVis->AddGeometry(this->Scene()->CreateCylinder());
      scaleShaftZVis->SetOrigin(0, 0, 0.5);
      scaleShaftZVis->SetLocalPosition(0, 0, 0.5);
      scaleShaftZVis->SetLocalScale(0.02, 0.02, 0.5);
      scaleZVis->AddChild(scaleShaftZVis);

      VisualPtr scaleHeadZVis = this->Scene()->CreateVisual();
      scaleHeadZVis->AddGeometry(this->Scene()->CreateBox());
      scaleHeadZVis->SetOrigin(0, 0, -0.5);
      scaleHeadZVis->SetLocalPosition(0, 0, 0.5);
      scaleHeadZVis->SetLocalScale(0.07, 0.07, 0.07);
      scaleZVis->AddChild(scaleHeadZVis);

      scaleZVis->SetMaterial(this->materials[AM_Z], false);
      scaleVis->AddChild(scaleZVis);

      this->visuals[TransformAxis::TA_SCALE_X] = scaleXVis;
      this->visuals[TransformAxis::TA_SCALE_Y] = scaleYVis;
      this->visuals[TransformAxis::TA_SCALE_Z] = scaleZVis;

      // scale handles
      VisualPtr scaleHandleXVis = this->Scene()->CreateVisual();
      scaleHandleXVis->AddGeometry(this->Scene()->CreateCylinder());
      scaleHandleXVis->SetLocalPosition(0, 0, 0.285);
      scaleHandleXVis->SetLocalScale(0.11, 0.11, 0.57);
      scaleHandleXVis->SetMaterial(this->materials[AM_HANDLE], false);
      scaleXVis->AddChild(scaleHandleXVis);

      VisualPtr scaleHandleYVis = this->Scene()->CreateVisual();
      scaleHandleYVis->AddGeometry(this->Scene()->CreateCylinder());
      scaleHandleYVis->SetLocalPosition(0, 0, 0.285);
      scaleHandleYVis->SetLocalScale(0.11, 0.11, 0.57);
      scaleHandleYVis->SetMaterial(this->materials[AM_HANDLE], false);
      scaleYVis->AddChild(scaleHandleYVis);

      VisualPtr scaleHandleZVis = this->Scene()->CreateVisual();
      scaleHandleZVis->AddGeometry(this->Scene()->CreateCylinder());
      scaleHandleZVis->SetLocalPosition(0, 0, 0.285);
      scaleHandleZVis->SetLocalScale(0.11, 0.11, 0.57);
      scaleHandleZVis->SetMaterial(this->materials[AM_HANDLE], false);
      scaleZVis->AddChild(scaleHandleZVis);

      this->handles[TransformAxis::TA_SCALE_X] = scaleHandleXVis;
      this->handles[TransformAxis::TA_SCALE_Y] = scaleHandleYVis;
      this->handles[TransformAxis::TA_SCALE_Z] = scaleHandleZVis;

      this->AddChild(scaleVis);
    }

    //////////////////////////////////////////////////
    template <class T>
    VisualPtr BaseGizmoVisual<T>::ChildByAxis(unsigned int _axis) const
    {
      auto it = this->visuals.find(_axis);
      if (it != this->visuals.end())
        return it->second;

      return VisualPtr();
    }
    }
  }
}
#endif
