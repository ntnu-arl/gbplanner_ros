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

#include <gtest/gtest.h>

#include "test_config.h"  // NOLINT(build/include)

#include "gz/rendering/PixelFormat.hh"

using namespace gz;
using namespace rendering;

/////////////////////////////////////////////////
TEST(PixelFormatTest, PixelUtil)
{
  PixelFormat format = PF_R8G8B8;
  EXPECT_EQ(3u, PixelUtil::BytesPerPixel(format));
  EXPECT_EQ(1u, PixelUtil::BytesPerChannel(format));
  EXPECT_EQ(3072u, PixelUtil::MemorySize(format, 32, 32));
  EXPECT_EQ("R8G8B8", PixelUtil::Name(format));
  EXPECT_EQ(format, PixelUtil::Enum("R8G8B8"));

  format = PF_FLOAT32_R;
  EXPECT_EQ(4u, PixelUtil::BytesPerPixel(format));
  EXPECT_EQ(4u, PixelUtil::BytesPerChannel(format));
  EXPECT_EQ(4096u, PixelUtil::MemorySize(format, 32, 32));
  EXPECT_EQ("FLOAT32_R", PixelUtil::Name(format));
  EXPECT_EQ(format, PixelUtil::Enum("FLOAT32_R"));

  format = PF_FLOAT32_RGB;
  EXPECT_EQ(12u, PixelUtil::BytesPerPixel(format));
  EXPECT_EQ(4u, PixelUtil::BytesPerChannel(format));
  EXPECT_EQ(12288u, PixelUtil::MemorySize(format, 32, 32));
  EXPECT_EQ("FLOAT32_RGB", PixelUtil::Name(format));
  EXPECT_EQ(format, PixelUtil::Enum("FLOAT32_RGB"));

  format = PF_FLOAT32_RGBA;
  EXPECT_EQ(16u, PixelUtil::BytesPerPixel(format));
  EXPECT_EQ(4u, PixelUtil::BytesPerChannel(format));
  EXPECT_EQ(16384u, PixelUtil::MemorySize(format, 32, 32));
  EXPECT_EQ("FLOAT32_RGBA", PixelUtil::Name(format));
  EXPECT_EQ(format, PixelUtil::Enum("FLOAT32_RGBA"));

  format = PF_L16;
  EXPECT_EQ(2u, PixelUtil::BytesPerPixel(format));
  EXPECT_EQ(2u, PixelUtil::BytesPerChannel(format));
  EXPECT_EQ(2048u, PixelUtil::MemorySize(format, 32, 32));
  EXPECT_EQ("L16", PixelUtil::Name(format));
  EXPECT_EQ(format, PixelUtil::Enum("L16"));

  format = PF_R8G8B8A8;
  EXPECT_EQ(4u, PixelUtil::BytesPerPixel(format));
  EXPECT_EQ(1u, PixelUtil::BytesPerChannel(format));
  EXPECT_EQ(4096u, PixelUtil::MemorySize(format, 32, 32));
}

/////////////////////////////////////////////////
TEST(PixelFormatTest, PixelUtilInvalid)
{
  PixelFormat format = static_cast<PixelFormat>(16);

  EXPECT_EQ(PF_UNKNOWN, PixelUtil::Sanitize(format));
  EXPECT_EQ("UNKNOWN", PixelUtil::Name(format));

  EXPECT_EQ(PF_UNKNOWN, PixelUtil::Enum("invalid"));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
