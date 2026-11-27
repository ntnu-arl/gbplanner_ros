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
#include <gz/common/Image.hh>
#include <gz/common/Filesystem.hh>
#include <gz/rendering.hh>

#include "example_config.hh"

void BuildScene(gz::rendering::ScenePtr _scene);

void PresentImage(gz::rendering::ImagePtr _image,
    const std::string &_name);


// Global constants due to laziness
const double width = 512;
const double height = 512;
const int bytesPerPixel = 3;

//! [init shaders variables]
const std::string depth_vertex_shader_file =
  "depth_vertex_shader.glsl";
const std::string depth_fragment_shader_file =
  "depth_fragment_shader.glsl";
const std::string vertex_shader_file = "vertex_shader.glsl";
const std::string fragment_shader_file = "fragment_shader.glsl";
//! [init shaders variables]

const std::string RESOURCE_PATH =
    ignition::common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");


//////////////////////////////////////////////////
int main()
{
  // Initialize gz::rendering
  auto *engine = gz::rendering::engine("ogre");
  if (!engine)
  {
    std::cerr << "Failed to load ogre\n";
    return 1;
  }
  // Create a scene and add stuff to it
  gz::rendering::ScenePtr scene = engine->CreateScene("scene");
  if (!scene)
  {
    std::cerr << "Failed to create scene." << std::endl;
    return 1;
  }
  BuildScene(scene);

  gz::rendering::VisualPtr root = scene->RootVisual();

  // Create a camera
  gz::rendering::CameraPtr camera;
  camera = scene->CreateCamera("example_custom_shaders");
  camera->SetImageWidth(width);
  camera->SetImageHeight(height);
  camera->SetHFOV(1.05);
  camera->SetAntiAliasing(2);
  camera->SetAspectRatio(width / height);
  camera->SetImageFormat(gz::rendering::PF_R8G8B8);
  root->AddChild(camera);

  // Create a camera for depth image
  gz::rendering::CameraPtr depthCamera;
  depthCamera = scene->CreateCamera("example_custom_shaders_depth");
  depthCamera->SetImageWidth(width);
  depthCamera->SetImageHeight(height);
  depthCamera->SetHFOV(1.05);
  depthCamera->SetAntiAliasing(2);
  depthCamera->SetAspectRatio(width / height);
  depthCamera->SetImageFormat(gz::rendering::PF_R8G8B8);
  root->AddChild(depthCamera);

  //! [Get shader path]
  // path to look for vertex and fragment shaders
  std::string depth_vertex_shader_path = gz::common::joinPaths(
      RESOURCE_PATH,  depth_vertex_shader_file);

  std::string depth_fragment_shader_path = gz::common::joinPaths(
      RESOURCE_PATH, depth_fragment_shader_file);
  //! [Get shader path]

  //! [add shader to camera]
  // create shader material
  gz::rendering::MaterialPtr depthMat = scene->CreateMaterial();
  depthMat->SetVertexShader(depth_vertex_shader_path);
  depthMat->SetFragmentShader(depth_fragment_shader_path);

  // apply shader to camera
  depthCamera->SetMaterial(depthMat);
  //! [add shader to camera]

  gz::rendering::ImagePtr image =
    std::make_shared<gz::rendering::Image>(camera->CreateImage());
  gz::rendering::ImagePtr depthImage =
    std::make_shared<gz::rendering::Image>(depthCamera->CreateImage());

  depthCamera->Capture(*depthImage);
  PresentImage(depthImage, "depth.png");
  camera->Capture(*image);
  PresentImage(image, "regular.png");

  return 0;
}

//////////////////////////////////////////////////
void PresentImage(gz::rendering::ImagePtr _image,
    const std::string &_name)
{
  // Present the data
  unsigned char *data = _image->Data<unsigned char>();

  gz::common::Image image;
  image.SetFromData(data, width, height, gz::common::Image::RGB_INT8);

  image.SavePNG(_name);

  std::cout << "Image saved: " << _name << std::endl;
}

//////////////////////////////////////////////////
void BuildScene(gz::rendering::ScenePtr _scene)
{
  // initialize _scene
  _scene->SetAmbientLight(0.3, 0.3, 0.3);
  gz::rendering::VisualPtr root = _scene->RootVisual();

  // create directional light
  gz::rendering::DirectionalLightPtr light0 =
    _scene->CreateDirectionalLight();
  light0->SetDirection(-0.5, 0.5, -1);
  light0->SetDiffuseColor(0.5, 0.5, 0.5);
  light0->SetSpecularColor(0.5, 0.5, 0.5);
  root->AddChild(light0);

  // create white material
  gz::rendering::MaterialPtr grey = _scene->CreateMaterial();
  grey->SetAmbient(0.5, 0.5, 0.5);
  grey->SetDiffuse(0.8, 0.8, 0.8);
  grey->SetReceiveShadows(true);
  grey->SetReflectivity(0);

  // create plane visual
  gz::rendering::VisualPtr plane = _scene->CreateVisual();
  auto geom = _scene->CreatePlane();
  plane->AddGeometry(geom);
  plane->SetLocalScale(5, 8, 1);
  plane->SetLocalPosition(3, 0, -0.5);
  plane->SetMaterial(grey);
  root->AddChild(plane);

  // create shader materials
  // path to look for vertex and fragment shader parameters
  std::string vertex_shader_path = gz::common::joinPaths(
      RESOURCE_PATH, vertex_shader_file);

  std::string fragment_shader_path = gz::common::joinPaths(
      RESOURCE_PATH, fragment_shader_file);

  //! [add shader to visual]
  // create shader material
  gz::rendering::MaterialPtr shader = _scene->CreateMaterial();
  shader->SetVertexShader(vertex_shader_path);
  shader->SetFragmentShader(fragment_shader_path);

  // create box visual and apply shader
  gz::rendering::VisualPtr box = _scene->CreateVisual();
  box->AddGeometry(_scene->CreateBox());
  box->SetOrigin(0.0, 0.5, 0.0);
  box->SetLocalPosition(3, 0, 0);
  box->SetLocalRotation(IGN_PI / 4, 0, IGN_PI / 3);
  box->SetLocalScale(1, 2.5, 1);
  box->SetMaterial(shader);
  //! [add shader to visual]

  root->AddChild(box);
}
