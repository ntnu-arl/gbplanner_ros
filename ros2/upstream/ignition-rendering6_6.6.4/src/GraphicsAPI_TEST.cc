/*
 * Copyright (C) 2022 Open Source Robotics Foundation
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

#include <gtest/gtest.h>

#include <ignition/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "ignition/rendering/GraphicsAPI.hh"

using namespace ignition;
using namespace rendering;

/////////////////////////////////////////////////
TEST(GraphicsAPITest, GraphicsAPI)
{
  GraphicsAPI api = GraphicsAPIUtils::Set("UNKNOWN");
  EXPECT_EQ(GraphicsAPI::UNKNOWN, api);

  api = GraphicsAPIUtils::Set("DIRECT3D11");
  EXPECT_EQ(GraphicsAPI::DIRECT3D11, api);

  api = GraphicsAPIUtils::Set("VULKAN");
  EXPECT_EQ(GraphicsAPI::VULKAN, api);

  api = GraphicsAPIUtils::Set("METAL");
  EXPECT_EQ(GraphicsAPI::METAL, api);

  api = GraphicsAPIUtils::Set("invalid");
  EXPECT_EQ(GraphicsAPI::UNKNOWN, api);

  std::string str = GraphicsAPIUtils::Str(GraphicsAPI::UNKNOWN);
  EXPECT_EQ("UNKNOWN", str);

  str = GraphicsAPIUtils::Str(GraphicsAPI::DIRECT3D11);
  EXPECT_EQ("DIRECT3D11", str);

  str = GraphicsAPIUtils::Str(GraphicsAPI::VULKAN);
  EXPECT_EQ("VULKAN", str);

  str = GraphicsAPIUtils::Str(GraphicsAPI::METAL);
  EXPECT_EQ("METAL", str);

  str = GraphicsAPIUtils::Str(static_cast<GraphicsAPI>(99));
  EXPECT_TRUE(str.empty());
}
