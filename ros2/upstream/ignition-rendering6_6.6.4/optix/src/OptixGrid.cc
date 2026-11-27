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

#include "gz/rendering/optix/OptixGrid.hh"
#include "gz/rendering/optix/OptixIncludes.hh"
#include "gz/rendering/optix/OptixVisual.hh"

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
// OptixGrid
//////////////////////////////////////////////////
OptixGrid::OptixGrid()
{
}

//////////////////////////////////////////////////
OptixGrid::~OptixGrid()
{
}

//////////////////////////////////////////////////
void OptixGrid::PreRender()
{
  // TODO remove me once grid is implemented.
  // hide the fake grid
  BaseGrid::PreRender();
  if (this->parent)
    this->parent->SetLocalScale(0.0, 0.0, 0.0);
}
