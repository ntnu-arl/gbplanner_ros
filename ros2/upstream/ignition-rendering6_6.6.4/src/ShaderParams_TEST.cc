/* * Copyright (C) 2017 Open Source Robotics Foundation
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


#include "gz/rendering/ShaderParams.hh"

using namespace gz::rendering;


/////////////////////////////////////////////////
TEST(ShaderParams, InitiallyNotDirty)
{
  ShaderParams params;
  EXPECT_FALSE(params.IsDirty());
}

/////////////////////////////////////////////////
TEST(ShaderParams, NonConstAccessDirties)
{
  ShaderParams params;
  params["some_parameter"];
  EXPECT_TRUE(params.IsDirty());
}

/////////////////////////////////////////////////
TEST(ShaderParams, ClearDirty)
{
  ShaderParams params;
  params["some_parameter"];
  EXPECT_TRUE(params.IsDirty());
  params.ClearDirty();
  EXPECT_FALSE(params.IsDirty());
}

/////////////////////////////////////////////////
TEST(ShaderParams, ConstAccessDoesNotDirty)
{
  ShaderParams params;
  params["some_parameter"];
  params.ClearDirty();
  const ShaderParams &constParams = params;
  constParams["some_parameter"];
  EXPECT_FALSE(params.IsDirty());
}


/////////////////////////////////////////////////
TEST(ShaderParams, ConstRangeForLoopDoesNotDirty)
{
  ShaderParams params;
  params["some_parameter"] = 4.0f;
  params.ClearDirty();

  for (const auto & name_param : params)
  {
    EXPECT_EQ(std::string("some_parameter"), name_param.first);
  }
  EXPECT_FALSE(params.IsDirty());
}

/////////////////////////////////////////////////
TEST(ShaderParams, Iterator)
{
  ShaderParams params;
  params["some_parameter"] = 4.0f;
  params["some_parameter2"] = 4.1f;
  auto iterator = params.begin();
  auto iteratorCopy = iterator;
  EXPECT_TRUE(iterator == params.begin());
  EXPECT_EQ(iteratorCopy->first, iterator->first);
  EXPECT_EQ("some_parameter2", iterator->first);
  EXPECT_EQ(ShaderParam::PARAM_FLOAT, iterator->second.Type());
  float val;
  EXPECT_TRUE(iterator->second.Value(&val));
  EXPECT_FLOAT_EQ(4.1f, val);

  iterator++;
  EXPECT_EQ("some_parameter", iterator->first);
  EXPECT_EQ(ShaderParam::PARAM_FLOAT, iterator->second.Type());
  EXPECT_TRUE(iterator->second.Value(&val));
  EXPECT_FLOAT_EQ(4.0f, val);
}


//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
