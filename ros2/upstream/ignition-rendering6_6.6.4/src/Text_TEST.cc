/* * Copyright (C) 2018 Open Source Robotics Foundation
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

#include <gz/common/Console.hh>

#include "test_config.h"  // NOLINT(build/include)
#include "gz/rendering/RenderEngine.hh"
#include "gz/rendering/RenderingIface.hh"
#include "gz/rendering/Text.hh"
#include "gz/rendering/Scene.hh"

using namespace gz;
using namespace rendering;

class TextTest : public testing::Test,
                 public testing::WithParamInterface<const char *>
{
  public: void Text(const std::string &_renderEngine);
};

/////////////////////////////////////////////////
void TextTest::Text(const std::string &_renderEngine)
{
#ifdef __APPLE__                                                                                                                                                                              
  std::cerr << "Skipping test for apple, see issue #847." << std::endl;
  return;
#endif
  if (_renderEngine != "ogre")
  {
    igndbg << "Text not supported yet in rendering engine: "
            << _renderEngine << std::endl;
    return;
  }

  RenderEngine *engine = rendering::engine(_renderEngine);
  if (!engine)
  {
    FAIL() << "Engine '" << _renderEngine
           << "' is not supported" << std::endl;
    return;
  }
  ScenePtr scene = engine->CreateScene("scene");

  TextPtr text = scene->CreateText();
  ASSERT_NE(nullptr, text);

  // check default values
  EXPECT_NE(std::string(), text->FontName());
  EXPECT_EQ(std::string(), text->TextString());
  EXPECT_FLOAT_EQ(0.0, text->SpaceWidth());
  EXPECT_FLOAT_EQ(1.0, text->CharHeight());
  EXPECT_FLOAT_EQ(0.0, text->Baseline());
  EXPECT_EQ(TextHorizontalAlign::LEFT, text->HorizontalAlignment());
  EXPECT_EQ(TextVerticalAlign::BOTTOM, text->VerticalAlignment());
  EXPECT_EQ(math::Color::White, text->Color());
  EXPECT_FALSE(text->ShowOnTop());
  EXPECT_NE(math::AxisAlignedBox(), text->AABB());

  // check setting text properties
  text->SetFontName("Liberation Sans");
  EXPECT_EQ("Liberation Sans", text->FontName());

  text->SetTextString("abc def");
  EXPECT_EQ("abc def", text->TextString());

  text->SetCharHeight(1.8f);
  EXPECT_FLOAT_EQ(1.8f, text->CharHeight());

  text->SetSpaceWidth(1.5f);
  EXPECT_FLOAT_EQ(1.5f, text->SpaceWidth());

  text->SetBaseline(0.5f);
  EXPECT_FLOAT_EQ(0.5f, text->Baseline());

  text->SetTextAlignment(TextHorizontalAlign::CENTER, TextVerticalAlign::TOP);
  EXPECT_EQ(TextHorizontalAlign::CENTER, text->HorizontalAlignment());
  EXPECT_EQ(TextVerticalAlign::TOP, text->VerticalAlignment());

  text->SetShowOnTop(true);
  EXPECT_TRUE(text->ShowOnTop());

  text->SetColor(math::Color(1.0f, 0.2f, 0.3f, 1.0f));
  EXPECT_EQ(math::Color(1.0f, 0.2f, 0.3f, 1.0f), text->Color());

  // create material
  MaterialPtr mat = scene->CreateMaterial();
  mat->SetAmbient(0.6f, 0.7f, 0.8f);
  mat->SetDiffuse(0.3f, 0.8f, 0.2f);
  mat->SetSpecular(0.4f, 0.9f, 1.0f);

  text->SetMaterial(mat);
  MaterialPtr textMat = text->Material();
  ASSERT_NE(nullptr, textMat);
  EXPECT_EQ(math::Color(0.6f, 0.7f, 0.8f), textMat->Ambient());
  EXPECT_EQ(math::Color(0.3f, 0.8f, 0.2f), textMat->Diffuse());
  EXPECT_EQ(math::Color(0.4f, 0.9f, 1.0f), textMat->Specular());

  // color is affected by material but currently only by the diffuse component
  EXPECT_EQ(math::Color(0.3f, 0.8f, 0.2f, 1.0f), text->Color());

  // Clean up
  engine->DestroyScene(scene);
  unloadEngine(engine->Name());
}

/////////////////////////////////////////////////
TEST_P(TextTest, Text)
{
  Text(GetParam());
}

INSTANTIATE_TEST_CASE_P(Text, TextTest,
    RENDER_ENGINE_VALUES,
    PrintToStringParam());

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
