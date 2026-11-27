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

#include <ignition/common/Console.hh>
#include "ignition/rendering/ogre/OgreDynamicLines.hh"
#include "ignition/rendering/ogre/OgreLidarVisual.hh"
#include "ignition/rendering/ogre/OgreScene.hh"
#include "ignition/rendering/ogre/OgreMarker.hh"
#include "ignition/rendering/ogre/OgreGeometry.hh"


class ignition::rendering::OgreLidarVisualPrivate
{
  /// \brief Non Hitting DynamicLines Object to display
  public: std::vector<std::shared_ptr<OgreDynamicLines>> noHitRayStrips;

  /// \brief Hitting DynamicLines Object to display
  public: std::vector<std::shared_ptr<OgreDynamicLines>> rayStrips;

  /// \brief Dead Zone Geometry DynamicLines Object to display
  public: std::vector<std::shared_ptr<OgreDynamicLines>> deadZoneRayFans;

  /// \brief Lidar Ray DynamicLines Object to display
  public: std::vector<std::shared_ptr<OgreDynamicLines>> rayLines;

  /// \brief Lidar Points DynamicLines Object to display
  public: std::vector<std::shared_ptr<OgreDynamicLines>> points;

  /// \brief Lidar visual type
  public: LidarVisualType lidarVisType =
            LidarVisualType::LVT_TRIANGLE_STRIPS;

  /// \brief Current value of DisplayNonHitting parameter
  public: bool currentDisplayNonHitting = true;

  /// \brief The current lidar points data
  public: std::vector<double> lidarPoints;

  /// \brief The colour of rendered points
  public: std::vector<ignition::math::Color> pointColors;

  /// \brief True if new points data is received
  public: bool receivedData = false;

  /// \brief The visibility of the visual
  public: bool visible = true;
};

using namespace ignition;
using namespace rendering;

//////////////////////////////////////////////////
OgreLidarVisual::OgreLidarVisual()
  : dataPtr(new OgreLidarVisualPrivate)
{
}

//////////////////////////////////////////////////
OgreLidarVisual::~OgreLidarVisual()
{
  // no ops
}

//////////////////////////////////////////////////
void OgreLidarVisual::PreRender()
{
  // no ops
}

//////////////////////////////////////////////////
void OgreLidarVisual::Destroy()
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

  this->ClearPoints();
  this->ClearVisualData();
}

//////////////////////////////////////////////////
void OgreLidarVisual::Init()
{
  BaseLidarVisual::Init();
  this->Create();
}

//////////////////////////////////////////////////
void OgreLidarVisual::Create()
{
  this->ClearPoints();
  this->dataPtr->receivedData = false;
}

//////////////////////////////////////////////////
void OgreLidarVisual::ClearPoints()
{
  this->dataPtr->lidarPoints.clear();
  this->ClearVisualData();
  this->dataPtr->receivedData = false;
}

//////////////////////////////////////////////////
void OgreLidarVisual::ClearVisualData()
{
  this->dataPtr->noHitRayStrips.clear();
  this->dataPtr->deadZoneRayFans.clear();
  this->dataPtr->rayLines.clear();
  this->dataPtr->rayStrips.clear();
  this->dataPtr->points.clear();
}

//////////////////////////////////////////////////
void OgreLidarVisual::SetPoints(const std::vector<double> &_points)
{
  this->dataPtr->lidarPoints = _points;
  this->dataPtr->pointColors.clear();
  for (unsigned int i = 0u; i < this->dataPtr->lidarPoints.size(); ++i)
  {
    this->dataPtr->pointColors.push_back(ignition::math::Color::Blue);
  }
  this->dataPtr->receivedData = true;
}

//////////////////////////////////////////////////
void OgreLidarVisual::SetPoints(const std::vector<double> &_points,
                        const std::vector<ignition::math::Color> &_colors)
{
  if (_points.size() != _colors.size())
  {
    ignerr << "Unequal size of point and color vector."
           << "Setting all point colors blue." << std::endl;
    this->SetPoints(_points);
  }
  this->dataPtr->lidarPoints = _points;
  this->dataPtr->pointColors = _colors;
  this->dataPtr->receivedData = true;
}

//////////////////////////////////////////////////
void OgreLidarVisual::Update()
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

  #if (!(OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7)))
  // the Materials are assigned here to avoid repetitive search for materials
  Ogre::MaterialPtr noHitRayStripsMat =
                  Ogre::MaterialManager::getSingleton().getByName(
                                                    "Lidar/LightBlueStrips");
  Ogre::MaterialPtr rayLineMat =
                  Ogre::MaterialManager::getSingleton().getByName(
                                                    "Lidar/BlueRay");
  Ogre::MaterialPtr hitRayStripsMat =
                  Ogre::MaterialManager::getSingleton().getByName(
                                                    "Lidar/BlueStrips");
  Ogre::MaterialPtr deadZoneMat =
                  Ogre::MaterialManager::getSingleton().getByName(
                                                    "Lidar/TransBlack");
  Ogre::MaterialPtr pointsMat =
                  Ogre::MaterialManager::getSingleton().getByName(
                                                    "PointCloudPoint");
  #endif

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
        std::shared_ptr<OgreDynamicLines> line =
                    std::shared_ptr<OgreDynamicLines>(
                          new OgreDynamicLines(MT_LINE_LIST));

        #if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
              line->setMaterial("Lidar/BlueRay");
        #else
            line->setMaterial(rayLineMat);
        #endif
        std::shared_ptr<Ogre::MovableObject> mv =
                std::dynamic_pointer_cast<Ogre::MovableObject>(line);
        this->Node()->attachObject(mv.get());
        this->dataPtr->rayLines.push_back(line);

        if (this->dataPtr->lidarVisType == LidarVisualType::LVT_TRIANGLE_STRIPS)
        {
          line = std::shared_ptr<OgreDynamicLines>(
                    new OgreDynamicLines(MT_TRIANGLE_STRIP));

          #if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
            line->setMaterial("Lidar/LightBlueStrips");
          #else
            line->setMaterial(noHitRayStripsMat);
          #endif
          mv = std::dynamic_pointer_cast<Ogre::MovableObject>(line);
          this->Node()->attachObject(mv.get());
          this->dataPtr->noHitRayStrips.push_back(line);

          line = std::shared_ptr<OgreDynamicLines>(
                      new OgreDynamicLines(MT_TRIANGLE_FAN));

          #if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
            line->setMaterial("Lidar/TransBlack");
          #else
            line->setMaterial(deadZoneMat);
          #endif
          mv = std::dynamic_pointer_cast<Ogre::MovableObject>(line);
          this->Node()->attachObject(mv.get());
          this->dataPtr->deadZoneRayFans.push_back(line);
          this->dataPtr->deadZoneRayFans[j]->AddPoint(
                      ignition::math::Vector3d::Zero);

          line = std::shared_ptr<OgreDynamicLines>(
                      new OgreDynamicLines(MT_TRIANGLE_STRIP));

          #if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
            line->setMaterial("Lidar/BlueStrips");
          #else
            line->setMaterial(hitRayStripsMat);
          #endif
          mv = std::dynamic_pointer_cast<Ogre::MovableObject>(line);
          this->Node()->attachObject(mv.get());
          this->dataPtr->rayStrips.push_back(line);
        }
      }
    }

    else if (this->dataPtr->lidarVisType ==
                  LidarVisualType::LVT_POINTS)
    {
      if (j+1 > this->dataPtr->points.size())
      {
        std::shared_ptr<OgreDynamicLines> line =
                  std::shared_ptr<OgreDynamicLines>(
                              new OgreDynamicLines(MT_POINTS));

        #if (OGRE_VERSION <= ((1 << 16) | (10 << 8) | 7))
            line->setMaterial("PointCloudPoint");
        #else
            line->setMaterial(pointsMat);
        #endif
        std::shared_ptr<Ogre::MovableObject> mv =
                std::dynamic_pointer_cast<Ogre::MovableObject>(line);
        this->Node()->attachObject(mv.get());
        this->dataPtr->points.push_back(line);
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
            this->dataPtr->points[j]->AddPoint(inf ? noHitPt: pt,
                  this->dataPtr->pointColors[j * this->horizontalCount + i]);
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
    // get the PointCloudPoint material
    Ogre::MaterialPtr mat = this->dataPtr->points[0]->getMaterial();
    auto pass = mat->getTechnique(0)->getPass(0);
    auto params = pass->getVertexProgramParameters();
    params->setNamedConstant("size", static_cast<Ogre::Real>(this->size));
  }

  // The newly created dynamic lines are having default visibility as true.
  // The visibility needs to be set as per the current value after the new
  // renderables are created.
  this->SetVisible(this->dataPtr->visible);
}

//////////////////////////////////////////////////
unsigned int OgreLidarVisual::PointCount() const
{
  return this->dataPtr->lidarPoints.size();
}

//////////////////////////////////////////////////
std::vector<double> OgreLidarVisual::Points() const
{
  return this->dataPtr->lidarPoints;
}

//////////////////////////////////////////////////
void OgreLidarVisual::SetVisible(bool _visible)
{
  this->dataPtr->visible = _visible;
  this->ogreNode->setVisible(this->dataPtr->visible);
}
