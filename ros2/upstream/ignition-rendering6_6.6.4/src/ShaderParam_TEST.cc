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


#include "gz/rendering/ShaderParam.hh"

using namespace gz::rendering;


/////////////////////////////////////////////////
TEST(ShaderParam, InitiallyNone)
{
  ShaderParam p;
  EXPECT_EQ(ShaderParam::PARAM_NONE, p.Type());
}

/////////////////////////////////////////////////
TEST(ShaderParam, FloatType)
{
  ShaderParam p;
  p = 10.0f;
  EXPECT_EQ(ShaderParam::PARAM_FLOAT, p.Type());
  float val;
  EXPECT_TRUE(p.Value(&val));
  EXPECT_FLOAT_EQ(10.0f, val);

  int badRef;
  EXPECT_FALSE(p.Value(&badRef));

  // test copy assignment
  ShaderParam p2;
  p2 = p;
  EXPECT_EQ(ShaderParam::PARAM_FLOAT, p2.Type());
  float val2;
  EXPECT_TRUE(p.Value(&val2));
  EXPECT_FLOAT_EQ(10.0f, val2);

  // test copy constructor
  ShaderParam p3(p);
  EXPECT_EQ(ShaderParam::PARAM_FLOAT, p3.Type());
  float val3;
  EXPECT_TRUE(p.Value(&val3));
  EXPECT_FLOAT_EQ(10.0f, val3);
}

/////////////////////////////////////////////////
TEST(ShaderParam, IntType)
{
  ShaderParam p;
  p = 10;
  EXPECT_EQ(ShaderParam::PARAM_INT, p.Type());
  int val;
  EXPECT_TRUE(p.Value(&val));
  EXPECT_EQ(10, val);

  float badRef = 20.0f;
  EXPECT_FALSE(p.Value(&badRef));

  // test copy assignment
  ShaderParam p2;
  p2 = p;
  EXPECT_EQ(ShaderParam::PARAM_INT, p2.Type());
  int val2;
  EXPECT_TRUE(p.Value(&val2));
  EXPECT_EQ(10, val2);

  // test copy constructor
  ShaderParam p3(p);
  EXPECT_EQ(ShaderParam::PARAM_INT, p3.Type());
  int val3;
  EXPECT_TRUE(p.Value(&val3));
  EXPECT_EQ(10, val3);
}

/////////////////////////////////////////////////
TEST(ShaderParam, FloatBufferType)
{
  float b[10] =
  {
    0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f, 0.8f, 0.9f
  };

  ShaderParam p;
  p.InitializeBuffer(10);
  p.UpdateBuffer(b);

  EXPECT_EQ(ShaderParam::PARAM_FLOAT_BUFFER, p.Type());
  EXPECT_EQ(10, static_cast<int>(p.Count()));

  std::shared_ptr<void> buffer;
  EXPECT_TRUE(p.Buffer(buffer));
  for (int x = 0; x < 10; ++x)
  {
    EXPECT_FLOAT_EQ(static_cast<float>(x) / 10.0f,
        reinterpret_cast<float*>(buffer.get())[x]);
  }

  // test copy assignment
  ShaderParam p2;
  p2 = p;
  EXPECT_EQ(ShaderParam::PARAM_FLOAT_BUFFER, p2.Type());
  EXPECT_EQ(10, static_cast<int>(p2.Count()));

  std::shared_ptr<void> buffer2;
  EXPECT_TRUE(p2.Buffer(buffer2));
  for (int x = 0; x < 10; ++x)
  {
    EXPECT_FLOAT_EQ(static_cast<float>(x) / 10.0f,
        reinterpret_cast<float*>(buffer2.get())[x]);
  }

  // test copy constructor
  ShaderParam p3(p);
  EXPECT_EQ(ShaderParam::PARAM_FLOAT_BUFFER, p3.Type());
  EXPECT_EQ(10, static_cast<int>(p3.Count()));

  std::shared_ptr<void> buffer3;
  EXPECT_TRUE(p3.Buffer(buffer3));
  for (int x = 0; x < 10; ++x)
  {
    EXPECT_FLOAT_EQ(static_cast<float>(x) / 10.0f,
        reinterpret_cast<float*>(buffer3.get())[x]);
  }

  ShaderParam p4;
  p4 = 1;
  std::shared_ptr<void> buffer4;
  EXPECT_FALSE(p4.Buffer(buffer4));
}

/////////////////////////////////////////////////
TEST(ShaderParam, IntBufferType)
{
  int b[10] =
  {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9
  };

  ShaderParam p;
  p.InitializeBuffer(10);
  p.UpdateBuffer(b);

  EXPECT_EQ(ShaderParam::PARAM_INT_BUFFER, p.Type());
  EXPECT_EQ(10, static_cast<int>(p.Count()));

  std::shared_ptr<void> buffer;
  EXPECT_TRUE(p.Buffer(buffer));
  for (int x = 0; x < 10; ++x)
  {
    EXPECT_EQ(x, reinterpret_cast<int*>(buffer.get())[x]);
  }

  // test copy assignment
  ShaderParam p2;
  p2 = p;
  EXPECT_EQ(ShaderParam::PARAM_INT_BUFFER, p2.Type());
  EXPECT_EQ(10, static_cast<int>(p2.Count()));

  std::shared_ptr<void> buffer2;
  EXPECT_TRUE(p2.Buffer(buffer2));
  for (int x = 0; x < 10; ++x)
  {
    EXPECT_EQ(x, reinterpret_cast<int*>(buffer2.get())[x]);
  }

  // test copy constructor
  ShaderParam p3(p);
  EXPECT_EQ(ShaderParam::PARAM_INT_BUFFER, p3.Type());
  EXPECT_EQ(10, static_cast<int>(p3.Count()));

  std::shared_ptr<void> buffer3;
  EXPECT_TRUE(p3.Buffer(buffer3));
  for (int x = 0; x < 10; ++x)
  {
    EXPECT_EQ(x, reinterpret_cast<int*>(buffer3.get())[x]);
  }

  ShaderParam p4;
  p4 = 1;
  std::shared_ptr<void> buffer4;
  EXPECT_FALSE(p4.Buffer(buffer4));
}

/////////////////////////////////////////////////
TEST(ShaderParam, TextureType)
{
  ShaderParam p;
  std::string texName = "some_texture.png";
  ShaderParam::ParamType type = ShaderParam::ParamType::PARAM_TEXTURE;
  unsigned int uvSetIndex = 1u;
  p.SetTexture(texName, type, uvSetIndex);

  std::string texNameRet;
  unsigned int uvSetIndexRet;
  p.Value(texNameRet, uvSetIndexRet);
  EXPECT_EQ(type, p.Type());
  EXPECT_EQ(texName, texNameRet);
  EXPECT_EQ(uvSetIndex, uvSetIndexRet);

  // test copy assignment
  ShaderParam p2;
  p2 = p;
  EXPECT_EQ(type, p2.Type());
  std::string texNameRet2;
  unsigned int uvSetIndexRet2;
  p2.Value(texNameRet2, uvSetIndexRet2);
  EXPECT_EQ(texName, texNameRet2);
  EXPECT_EQ(uvSetIndex, uvSetIndexRet2);

  // test copy constructor
  ShaderParam p3(p);
  EXPECT_EQ(type, p3.Type());
  std::string texNameRet3;
  unsigned int uvSetIndexRet3;
  p3.Value(texNameRet3, uvSetIndexRet3);
  EXPECT_EQ(texName, texNameRet3);
  EXPECT_EQ(uvSetIndex, uvSetIndexRet3);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
