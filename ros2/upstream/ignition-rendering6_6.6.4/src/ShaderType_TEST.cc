/* * Copyright (C) 2022 Open Source Robotics Foundation
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

#include "ignition/rendering/ShaderType.hh"

using namespace ignition;
using namespace rendering;

/////////////////////////////////////////////////
TEST(ShaderType, ShaderUtil)
{
  EXPECT_TRUE(ShaderUtil::IsValid(ShaderType::ST_PIXEL));
  EXPECT_TRUE(ShaderUtil::IsValid(ShaderType::ST_VERTEX));
  EXPECT_TRUE(ShaderUtil::IsValid(ShaderType::ST_NORM_OBJ));
  EXPECT_TRUE(ShaderUtil::IsValid(ShaderType::ST_NORM_TAN));
  EXPECT_FALSE(ShaderUtil::IsValid(ShaderType::ST_UNKNOWN));

  EXPECT_EQ(ShaderType::ST_PIXEL,
      ShaderUtil::Sanitize(ShaderType::ST_PIXEL));
  EXPECT_EQ(ShaderType::ST_UNKNOWN,
      ShaderUtil::Sanitize(static_cast<ShaderType>(99)));

  EXPECT_EQ("pixel", ShaderUtil::Name(ShaderType::ST_PIXEL));
  EXPECT_EQ("vertex", ShaderUtil::Name(ShaderType::ST_VERTEX));
  EXPECT_EQ("normal_map_object_space",
      ShaderUtil::Name(ShaderType::ST_NORM_OBJ));
  EXPECT_EQ("normal_map_tangent_space",
      ShaderUtil::Name(ShaderType::ST_NORM_TAN));
  EXPECT_EQ("UNKNOWN", ShaderUtil::Name(ShaderType::ST_UNKNOWN));

  EXPECT_EQ(ST_PIXEL, ShaderUtil::Enum("pixel"));
  EXPECT_EQ(ST_VERTEX, ShaderUtil::Enum("vertex"));
  EXPECT_EQ(ST_NORM_OBJ, ShaderUtil::Enum("normal_map_object_space"));
  EXPECT_EQ(ST_NORM_TAN, ShaderUtil::Enum("normal_map_tangent_space"));
  EXPECT_EQ(ST_UNKNOWN, ShaderUtil::Enum("UNKNOWN"));
  EXPECT_EQ(ST_UNKNOWN, ShaderUtil::Enum("invalid"));
}
