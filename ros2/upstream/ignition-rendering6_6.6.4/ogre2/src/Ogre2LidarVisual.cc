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

#ifdef __APPLE__
  #define GL_SILENCE_DEPRECATION
  #include <OpenGL/gl.h>
  #include <OpenGL/glext.h>
#else
#ifndef _WIN32
  #include <GL/gl.h>
#endif
#endif

#include <ignition/common/Console.hh>

#include "ignition/rendering/ogre2/Ogre2Conversions.hh"
#include "ignition/rendering/ogre2/Ogre2DynamicRenderable.hh"
#include "ignition/rendering/ogre2/Ogre2LidarVisual.hh"
#include "ignition/rendering/ogre2/Ogre2RenderEngine.hh"
#include "ignition/rendering/ogre2/Ogre2Scene.hh"
#include "ignition/rendering/ogre2/Ogre2Marker.hh"
#include "ignition/rendering/ogre2/Ogre2Geometry.hh"

#ifdef _MSC_VER
  #pragma warning(push, 0)
#endif
#include <OgreItem.h>
#include <OgreMaterialManager.h>
#include <OgreRoot.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#ifdef _MSC_VER
  #pragma warning(pop)
#endif

class ignition::rendering::Ogre2LidarVisualPrivate
{
  /// \brief Non Hitting DynamicLines Object to display
  public: std::vector<std::shared_ptr<Ogre2DynamicRenderable>> noHitRayStrips;

  /// \brief Hitting DynamicLines Object to display
  public: std::vector<std::shared_ptr<Ogre2DynamicRenderable>> rayStrips;

  /// \brief Dead Zone Geometry DynamicLines Object to display
  public: std::vector<std::shared_ptr<Ogre2DynamicRenderable>> deadZoneRayFans;

  /// \brief Lidar Ray DynamicLines Object to display
  public: std::vector<std::shared_ptr<Ogre2DynamicRenderable>> rayLines;

  /// \brief Lidar Points DynamicLines Object to display
  public: std::vector<std::shared_ptr<Ogre2DynamicRenderable>> points;

  /// \brief Lidar visual type
  public: LidarVisualType lidarVisType =
            LidarVisualType::LVT_TRIANGLE_STRIPS;

  /// \brief Current value of DisplayNonHitting parameter
  public: bool currentDisplayNonHitting = true;

  /// \brief The current lidar points data
  public: std::vector<double> lidarPoints;

  /// \brief True if new points data is received
  public: bool receivedData = false;

  /// \brief The visibility of the visual
  public: bool visible = true;

  /// \brief Pointer to point cloud material.
  /// Used when LidarVisualType = LVT_POINTS.
  public: Ogre::MaterialPtr pointsMat;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
Ogre2LidarVisual::Ogre2LidarVisual()
  : dataPtr(new Ogre2LidarVisualPrivate)
{
}

//////////////////////////////////////////////////
Ogre2LidarVisual::~Ogre2LidarVisual()
{
  // no ops
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::PreRender()
{
  // no ops
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::Destroy()
{
  BaseLidarVisual::Destroy();
  for (auto ray : this->dataPtr->noHitRayStrips)
  {
    ray->Clear();
    ray.reset();
  }

  for (auto ray : this->dataPtr->rayStrips)
  {
    ray->Clear();
    ray.reset();
  }

  for (auto ray : this->dataPtr->rayLines)
  {
    ray->Clear();
    ray.reset();
  }

  for (auto ray : this->dataPtr->deadZoneRayFans)
  {
    ray->Clear();
    ray.reset();
  }

  for (auto ray : this->dataPtr->points)
  {
    ray->Clear();
    ray.reset();
  }

  this->dataPtr->lidarPoints.clear();
  this->dataPtr->pointsMat.setNull();
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::Init()
{
  BaseLidarVisual::Init();
  this->Create();
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::Create()
{
  // enable GL_PROGRAM_POINT_SIZE so we can set gl_PointSize in vertex shader
  auto engine = Ogre2RenderEngine::Instance();
  std::string renderSystemName =
      engine->OgreRoot()->getRenderSystem()->getFriendlyName();
  if (renderSystemName.find("OpenGL") != std::string::npos)
  {
#ifdef __APPLE__
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);
#else
#ifndef _WIN32
    glEnable(GL_PROGRAM_POINT_SIZE);
#endif
#endif
  }
  this->dataPtr->pointsMat =
      Ogre::MaterialManager::getSingleton().getByName(
      "PointCloudPoint");

  this->ClearPoints();
  this->dataPtr->receivedData = false;
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::ClearPoints()
{
  this->dataPtr->lidarPoints.clear();
  this->ClearVisualData();
  this->dataPtr->receivedData = false;
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::ClearVisualData()
{
  this->dataPtr->noHitRayStrips.clear();
  this->dataPtr->deadZoneRayFans.clear();
  this->dataPtr->rayLines.clear();
  this->dataPtr->rayStrips.clear();
  this->dataPtr->points.clear();
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::SetPoints(const std::vector<double> &_points)
{
  this->dataPtr->lidarPoints = _points;
  this->dataPtr->receivedData = true;
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::Update()
{
  if (this->lidarVisualType == LidarVisualType::LVT_NONE)
  {
    this->ClearVisualData();
    return;
  }

  if (!this->dataPtr->receivedData || this->dataPtr->lidarPoints.size() == 0)
  {
    ignwarn << "New lidar data not received. Exiting update function"
            << std::endl;
    return;
  }

  bool clearVisuals = false;

  if (this->lidarVisualType != this->dataPtr->lidarVisType
        || !this->displayNonHitting)
  {
    clearVisuals = true;
  }

  if (this->displayNonHitting != this->dataPtr->currentDisplayNonHitting)
  {
    clearVisuals = true;
    this->dataPtr->currentDisplayNonHitting = this->displayNonHitting;
  }

  // if visual type is changed, clear all DynamicLines
  if (clearVisuals)
  {
    this->ClearVisualData();
  }
  this->dataPtr->lidarVisType = this->lidarVisualType;

  this->dataPtr->receivedData = false;
  double horizontalAngle = this->minHorizontalAngle;
  double verticalAngle = this->minVerticalAngle;

  if (this->horizontalCount > 1)
  {
    this->horizontalAngleStep =
        (this->maxHorizontalAngle - this->minHorizontalAngle) /
              (this->horizontalCount - 1);
  }

  if (this->verticalCount > 1)
  {
    this->verticalAngleStep =
        (this->maxVerticalAngle - this->minVerticalAngle) /
              (this->verticalCount - 1);
  }

  if (this->dataPtr->lidarPoints.size() !=
                  this->verticalCount * this->horizontalCount)
  {
    ignwarn << "Size of lidar data inconsistent with rays."
            << " Exiting update function."
            << std::endl;
    return;
  }

  // Process each point from received data
  // Every line segment, and every triangle is saved separately,
  // as a pointer to a DynamicLine
  // This initializes and updates only the selected DynamicLine variables
  for (unsigned int j = 0; j < this->verticalCount; ++j)
  {
    horizontalAngle = this->minHorizontalAngle;

    if (this->dataPtr->lidarVisType == LidarVisualType::LVT_RAY_LINES ||
        this->dataPtr->lidarVisType == LidarVisualType::LVT_TRIANGLE_STRIPS)
    {
      if (j+1 > this->dataPtr->rayLines.size())
      {
        std::shared_ptr<Ogre2DynamicRenderable> renderable =
                        std::shared_ptr<Ogre2DynamicRenderable>(
                                    new Ogre2DynamicRenderable(this->Scene()));

        renderable->SetOperationType(MT_LINE_LIST);
        MaterialPtr mat = this->Scene()->Material("Lidar/BlueRay");
        renderable->SetMaterial(mat, false);

        this->ogreNode->attachObject(renderable->OgreObject());
        this->dataPtr->rayLines.push_back(renderable);

        if (this->dataPtr->lidarVisType == LidarVisualType::LVT_TRIANGLE_STRIPS)
        {
          renderable = std::shared_ptr<Ogre2DynamicRenderable>(
                                  new Ogre2DynamicRenderable(this->Scene()));

          renderable->SetOperationType(MT_TRIANGLE_STRIP);
          mat = this->Scene()->Material("Lidar/LightBlueStrips");

          renderable->SetMaterial(mat, false);

          this->ogreNode->attachObject(renderable->OgreObject());
          this->dataPtr->noHitRayStrips.push_back(renderable);

          renderable = std::shared_ptr<Ogre2DynamicRenderable>(
                                  new Ogre2DynamicRenderable(this->Scene()));

          renderable->SetOperationType(MT_TRIANGLE_FAN);
          mat = this->Scene()->Material("Lidar/TransBlack");

          renderable->SetMaterial(mat, false);

          this->ogreNode->attachObject(renderable->OgreObject());
          this->dataPtr->deadZoneRayFans.push_back(renderable);
          this->dataPtr->deadZoneRayFans[j]->AddPoint(
                      ignition::math::Vector3d::Zero);

          renderable = std::shared_ptr<Ogre2DynamicRenderable>(
                                  new Ogre2DynamicRenderable(this->Scene()));

          renderable->SetOperationType(MT_TRIANGLE_STRIP);
          mat = this->Scene()->Material("Lidar/BlueStrips");

          renderable->SetMaterial(mat, false);

          this->ogreNode->attachObject(renderable->OgreObject());
          this->dataPtr->rayStrips.push_back(renderable);
        }
      }
    }

    else if (this->dataPtr->lidarVisType ==
                  LidarVisualType::LVT_POINTS)
    {
      if (j+1 > this->dataPtr->points.size())
      {
        std::shared_ptr<Ogre2DynamicRenderable> renderable =
                        std::shared_ptr<Ogre2DynamicRenderable>(
                                    new Ogre2DynamicRenderable(this->Scene()));

        renderable->SetOperationType(MT_POINTS);

        // use low level programmable material so we can customize point size
        Ogre::Item *item = dynamic_cast<Ogre::Item *>(renderable->OgreObject());
        item->setCastShadows(false);
        item->getSubItem(0)->setMaterial(this->dataPtr->pointsMat);

        this->ogreNode->attachObject(renderable->OgreObject());
        this->dataPtr->points.push_back(renderable);
      }
    }

    if (this->dataPtr->lidarVisType == LidarVisualType::LVT_TRIANGLE_STRIPS)
    {
      this->dataPtr->deadZoneRayFans[j]->SetPoint(0, this->offset.Pos());
    }

    unsigned count = this->horizontalCount;
    // Process each ray in current scan
    for (unsigned int i = 0; i < count; ++i)
    {
      // calculate range of the ray
      double r = this->dataPtr->lidarPoints[ j * this->horizontalCount + i];

      bool inf = (std::isinf(r) || r >= this->maxRange);
      ignition::math::Quaterniond ray(
        ignition::math::Vector3d(0.0, -verticalAngle, horizontalAngle));

      ignition::math::Vector3d axis = this->offset.Rot() * ray *
        ignition::math::Vector3d(1.0, 0.0, 0.0);

      // Check for infinite range, which indicates the ray did not
      // intersect an object.
      double hitRange = inf ? 0 : r;

      // Compute the start point of the ray
      ignition::math::Vector3d startPt =
                  (axis * this->minRange) + this->offset.Pos();

      // Compute the end point of the ray
      ignition::math::Vector3d pt =
                  (axis * hitRange) + this->offset.Pos();

      double noHitRange = inf ? this->maxRange : hitRange;

      // Compute the end point of the no-hit ray
      ignition::math::Vector3d noHitPt =
                  (axis * noHitRange) + this->offset.Pos();

      // Update the lines and strips that represent each simulated ray.

      // For TRIANGLE_STRIPS Lidar Visual to be displayed
      if (this->dataPtr->lidarVisType == LidarVisualType::LVT_TRIANGLE_STRIPS
        || this->dataPtr->lidarVisType == LidarVisualType::LVT_RAY_LINES)
      {
        if (i >= this->dataPtr->rayLines[j]->PointCount()/2)
        {
          if (this->displayNonHitting || !inf)
          {
            this->dataPtr->rayLines[j]->AddPoint(startPt);
            this->dataPtr->rayLines[j]->AddPoint(inf ? noHitPt: pt);
          }

          if (this->dataPtr->lidarVisType ==
                LidarVisualType::LVT_TRIANGLE_STRIPS)
          {
            this->dataPtr->rayStrips[j]->AddPoint(startPt);
            this->dataPtr->rayStrips[j]->AddPoint(inf ? startPt : pt);

            this->dataPtr->noHitRayStrips[j]->AddPoint(startPt);
            this->dataPtr->noHitRayStrips[j]->AddPoint(
                inf ? (this->displayNonHitting? noHitPt: startPt) : pt);
          }
        }
        else
        {
          if (this->displayNonHitting || !inf)
          {
            this->dataPtr->rayLines[j]->SetPoint(i*2, startPt);
            this->dataPtr->rayLines[j]->SetPoint(i*2+1, inf ? noHitPt: pt);
          }

          if (this->dataPtr->lidarVisType ==
                LidarVisualType::LVT_TRIANGLE_STRIPS)
          {
            this->dataPtr->rayStrips[j]->SetPoint(i*2, startPt);
            this->dataPtr->rayStrips[j]->SetPoint(i*2+1, inf? startPt : pt);

            this->dataPtr->noHitRayStrips[j]->SetPoint(i*2, startPt);
            this->dataPtr->noHitRayStrips[j]->SetPoint(i*2+1,
                inf ? (this->displayNonHitting? noHitPt: startPt) : pt);
          }
        }
        if (this->dataPtr->lidarVisType == LidarVisualType::LVT_TRIANGLE_STRIPS)
        {
          // Draw the triangle fan that indicates the dead zone.
          if (i+1 >= this->dataPtr->deadZoneRayFans[j]->PointCount())
            this->dataPtr->deadZoneRayFans[j]->AddPoint(startPt);
          else
            this->dataPtr->deadZoneRayFans[j]->SetPoint(i+1, startPt);
        }
      }

      // For POINTS Lidar Visual to be displayed
      else if (this->dataPtr->lidarVisType ==
                LidarVisualType::LVT_POINTS)
      {
        if (i >= this->dataPtr->points[j]->PointCount())
        {
          if (this->displayNonHitting || !inf)
          {
            this->dataPtr->points[j]->AddPoint(inf ? noHitPt: pt);
          }
        }
        else
        {
          if (this->displayNonHitting || !inf)
          {
            this->dataPtr->points[j]->SetPoint(i, inf ? noHitPt: pt);
          }
        }
      }
      horizontalAngle += this->horizontalAngleStep;
    }

    // Update the DynamicLines pointers after adding points based on type
    if (this->dataPtr->lidarVisType == LidarVisualType::LVT_TRIANGLE_STRIPS
      || this->dataPtr->lidarVisType == LidarVisualType::LVT_RAY_LINES)
    {
      this->dataPtr->rayLines[j]->Update();
      if (this->dataPtr->lidarVisType ==
                      LidarVisualType::LVT_TRIANGLE_STRIPS)
      {
        this->dataPtr->rayStrips[j]->Update();
        this->dataPtr->noHitRayStrips[j]->Update();
        this->dataPtr->deadZoneRayFans[j]->Update();
      }
    }
    else if (this->dataPtr->lidarVisType == LidarVisualType::LVT_POINTS)
    {
      this->dataPtr->points[j]->Update();
    }
    verticalAngle += this->verticalAngleStep;
  }

  if (this->dataPtr->lidarVisType == LidarVisualType::LVT_POINTS &&
      !this->dataPtr->points.empty())
  {
    // point renderables use low level materials
    // get the material and set size uniform variable
    auto pass = this->dataPtr->pointsMat->getTechnique(0)->getPass(0);
    auto vertParams = pass->getVertexProgramParameters();
    vertParams->setNamedConstant("size", static_cast<Ogre::Real>(this->size));

    // support setting color only from diffuse for now
    MaterialPtr mat = this->Scene()->Material("Lidar/BlueRay");
    auto fragParams = pass->getFragmentProgramParameters();
    fragParams->setNamedConstant("color",
        Ogre2Conversions::Convert(mat->Diffuse()));
  }

  // The newly created dynamic lines are having default visibility as true.
  // The visibility needs to be set as per the current value after the new
  // renderables are created.
  this->SetVisible(this->dataPtr->visible);
}

//////////////////////////////////////////////////
unsigned int Ogre2LidarVisual::PointCount() const
{
  return this->dataPtr->lidarPoints.size();
}

//////////////////////////////////////////////////
std::vector<double> Ogre2LidarVisual::Points() const
{
  return this->dataPtr->lidarPoints;
}

//////////////////////////////////////////////////
void Ogre2LidarVisual::SetVisible(bool _visible)
{
  this->dataPtr->visible = _visible;
  this->ogreNode->setVisible(this->dataPtr->visible);
}
