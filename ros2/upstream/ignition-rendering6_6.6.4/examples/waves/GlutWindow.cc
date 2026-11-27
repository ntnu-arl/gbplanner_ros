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

#if __APPLE__
  #include <OpenGL/gl.h>
  #include <OpenGL/OpenGL.h>
  #include <GLUT/glut.h>
#else
  #include <GL/glew.h>
  #include <GL/glut.h>
  #if _WIN32
    #define NOMINMAX
    #include <windows.h>
    #include <Wingdi.h>
    #include <GL/glu.h>
  #else
    #include <GL/gl.h>
  #endif
#endif

#if !defined(__APPLE__) && !defined(_WIN32)
  #include <GL/glx.h>
#endif

#include <mutex>

#include <ignition/common/Console.hh>
#include <ignition/rendering.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Image.hh>
#include <ignition/rendering/Scene.hh>

#include "example_config.hh"

#include "GlutWindow.hh"

#define KEY_ESC 27
#define KEY_TAB  9

const std::string RESOURCE_PATH =
    ignition::common::joinPaths(std::string(PROJECT_BINARY_PATH), "media");

//////////////////////////////////////////////////
unsigned int imgw = 0;
unsigned int imgh = 0;

std::vector<ir::CameraPtr> g_cameras;
ir::CameraPtr g_camera;
ir::CameraPtr g_currCamera;
unsigned int g_cameraIndex = 0;
ir::ImagePtr g_image;
ir::ShaderParamsPtr g_fsParams;
ir::ShaderParamsPtr g_vsParams;

std::chrono::steady_clock::time_point g_startTime;

bool g_initContext = false;

#if __APPLE__
  CGLContextObj g_context;
  CGLContextObj g_glutContext;
#elif _WIN32
  HGLRC g_context = 0;
  HDC g_display = 0;
  HGLRC g_glutContext = 0;
  HDC g_glutDisplay = 0;
#else
  GLXContext g_context;
  Display *g_display;
  GLXDrawable g_drawable;
  GLXContext g_glutContext;
  Display *g_glutDisplay;
  GLXDrawable g_glutDrawable;
#endif

// view control variables
ir::RayQueryPtr g_rayQuery;
ir::OrbitViewController g_viewControl;
ir::RayQueryResult g_target;
struct mouseButton
{
  int button = 0;
  int state = GLUT_UP;
  int x = 0;
  int y = 0;
  int motionX = 0;
  int motionY = 0;
  int dragX = 0;
  int dragY = 0;
  int scroll = 0;
  bool buttonDirty = false;
  bool motionDirty = false;
};
struct mouseButton g_mouse;
std::mutex g_mouseMutex;

//////////////////////////////////////////////////
void mouseCB(int _button, int _state, int _x, int _y)
{
  // ignore unknown mouse button numbers
  if (_button >= 5)
    return;

  std::lock_guard<std::mutex> lock(g_mouseMutex);
  g_mouse.button = _button;
  g_mouse.state = _state;
  g_mouse.x = _x;
  g_mouse.y = _y;
  g_mouse.motionX = _x;
  g_mouse.motionY = _y;
  g_mouse.buttonDirty = true;
}

//////////////////////////////////////////////////
void motionCB(int _x, int _y)
{
  std::lock_guard<std::mutex> lock(g_mouseMutex);
  int deltaX = _x - g_mouse.motionX;
  int deltaY = _y - g_mouse.motionY;
  g_mouse.motionX = _x;
  g_mouse.motionY = _y;

  if (g_mouse.motionDirty)
  {
    g_mouse.dragX += deltaX;
    g_mouse.dragY += deltaY;
  }
  else
  {
    g_mouse.dragX = deltaX;
    g_mouse.dragY = deltaY;
  }
  g_mouse.motionDirty = true;
}

//////////////////////////////////////////////////
void handleMouse()
{
  std::lock_guard<std::mutex> lock(g_mouseMutex);
  // only ogre supports ray query for now so use
  // ogre camera located at camera index = 0.
  ir::CameraPtr rayCamera = g_cameras[0];
  if (!g_rayQuery)
  {
    g_rayQuery = rayCamera->Scene()->CreateRayQuery();
    if (!g_rayQuery)
    {
      ignerr << "Failed to create Ray Query" << std::endl;
      return;
    }
  }
  if (g_mouse.buttonDirty)
  {
    g_mouse.buttonDirty = false;
    double nx =
        2.0 * g_mouse.x / static_cast<double>(rayCamera->ImageWidth()) - 1.0;
    double ny = 1.0 -
        2.0 * g_mouse.y / static_cast<double>(rayCamera->ImageHeight());
    g_rayQuery->SetFromCamera(rayCamera, ignition::math::Vector2d(nx, ny));
    g_target  = g_rayQuery->ClosestPoint();
    if (!g_target)
    {
      // set point to be 10m away if no intersection found
      g_target.point = g_rayQuery->Origin() + g_rayQuery->Direction() * 10;
      return;
    }

    // mouse wheel scroll zoom
    if ((g_mouse.button == 3 || g_mouse.button == 4) &&
        g_mouse.state == GLUT_UP)
    {
      double scroll = (g_mouse.button == 3) ? -1.0 : 1.0;
      double distance = rayCamera->WorldPosition().Distance(
          g_target.point);
      int factor = 1;
      double amount = -(scroll * factor) * (distance / 5.0);
      for (ir::CameraPtr camera : g_cameras)
      {
        g_viewControl.SetCamera(camera);
        g_viewControl.SetTarget(g_target.point);
        g_viewControl.Zoom(amount);
      }
    }
  }

  if (g_mouse.motionDirty)
  {
    g_mouse.motionDirty = false;
    auto drag = ignition::math::Vector2d(g_mouse.dragX, g_mouse.dragY);

    // left mouse button pan
    if (g_mouse.button == GLUT_LEFT_BUTTON && g_mouse.state == GLUT_DOWN)
    {
      for (ir::CameraPtr camera : g_cameras)
      {
        g_viewControl.SetCamera(camera);
        g_viewControl.SetTarget(g_target.point);
        g_viewControl.Pan(drag);
      }
    }
    else if (g_mouse.button == GLUT_MIDDLE_BUTTON && g_mouse.state == GLUT_DOWN)
    {
      for (ir::CameraPtr camera : g_cameras)
      {
        g_viewControl.SetCamera(camera);
        g_viewControl.SetTarget(g_target.point);
        g_viewControl.Orbit(drag);
      }
    }
    // right mouse button zoom
    else if (g_mouse.button == GLUT_RIGHT_BUTTON && g_mouse.state == GLUT_DOWN)
    {
      double hfov = rayCamera->HFOV().Radian();
      double vfov = 2.0f * atan(tan(hfov / 2.0f) /
          rayCamera->AspectRatio());
      double distance = rayCamera->WorldPosition().Distance(
          g_target.point);
      double amount = ((-g_mouse.dragY /
          static_cast<double>(rayCamera->ImageHeight()))
          * distance * tan(vfov/2.0) * 6.0);
      for (ir::CameraPtr camera : g_cameras)
      {
        g_viewControl.SetCamera(camera);
        g_viewControl.SetTarget(g_target.point);
        g_viewControl.Zoom(amount);
      }
    }
  }
}

//////////////////////////////////////////////////
void updateUniforms()
{
  if (!g_vsParams)
    return;

  auto t = std::chrono::steady_clock::now() - g_startTime;
  float seconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(t).count() /
      1000.0;

  // update the time `t` uniform
  float s = fmod(seconds, 100);
  (*g_vsParams)["t"] = s;
}

//////////////////////////////////////////////////
void displayCB()
{
#if __APPLE__
  CGLSetCurrentContext(g_context);
#elif _WIN32
  if (!wglMakeCurrent(g_display, g_context))
  {
    std::cerr << "Not able to wglMakeCurrent" << '\n';
    exit(-1);
  }
#else
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  g_cameras[g_cameraIndex]->Capture(*g_image);
  handleMouse();

#if __APPLE__
  CGLSetCurrentContext(g_glutContext);
#elif _WIN32
  wglMakeCurrent(g_glutDisplay, g_glutContext);
#else
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif

  unsigned char *data = g_image->Data<unsigned char>();

  glClearColor(0.5, 0.5, 0.5, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPixelZoom(1, -1);
  glRasterPos2f(-1, 1);
  glDrawPixels(imgw, imgh, GL_RGB, GL_UNSIGNED_BYTE, data);

  glutSwapBuffers();
  updateUniforms();
}

//////////////////////////////////////////////////
void idleCB()
{
  glutPostRedisplay();
}

//////////////////////////////////////////////////
void keyboardCB(unsigned char _key, int, int)
{
  if (_key == KEY_ESC || _key == 'q' || _key == 'Q')
  {
    exit(0);
  }
}

//////////////////////////////////////////////////
void initCamera(ir::CameraPtr _camera)
{
  g_camera = _camera;
  imgw = g_camera->ImageWidth();
  imgh = g_camera->ImageHeight();
  ir::Image image = g_camera->CreateImage();
  g_image = std::make_shared<ir::Image>(image);
  g_camera->Capture(*g_image);
}

//////////////////////////////////////////////////
void initUniforms()
{
  ir::NodePtr node = g_camera->Parent();
  ir::VisualPtr waves =
      std::dynamic_pointer_cast<ir::Visual>(node->ChildByName("waves"));
  ir::MaterialPtr shader = waves->Material();
  if (!shader)
    return;

  // set vertex shader params
  g_vsParams = shader->VertexShaderParams();

  auto engine = g_camera->Scene()->Engine();
  if (engine->Name() == "ogre2")
  {
    // worldviewproj_matrix is a constant defined by ogre.
    // Here we add a line to add this constant to the params.
    // The specified value is ignored as it will be auto bound to the
    // correct type and value. See available constants:
    // https://github.com/OGRECave/ogre-next/blob/v2-2/OgreMain/src/OgreGpuProgramParams.cpp
    (*g_vsParams)["worldviewproj_matrix"] = 1;
  }

  (*g_vsParams)["Nwaves"] = 3;
  (*g_vsParams)["rescale"] = 0.5f;

  float bumpScale[2] = {25.0f, 25.0f};
  (*g_vsParams)["bumpScale"].InitializeBuffer(2);
  (*g_vsParams)["bumpScale"].UpdateBuffer(bumpScale);

  float bumpSpeed[2] = {0.01f, 0.01f};
  (*g_vsParams)["bumpSpeed"].InitializeBuffer(2);
  (*g_vsParams)["bumpSpeed"].UpdateBuffer(bumpSpeed);

  float amplitude = 3.0f;
  float amplitudeV[3] = {0.6f * amplitude, 0.4f * amplitude, 0.3f * amplitude};
  (*g_vsParams)["amplitude"].InitializeBuffer(3);
  (*g_vsParams)["amplitude"].UpdateBuffer(amplitudeV);

  float frequency = 0.028f;
  float wavenumberV[3] = {frequency, 3.2f * frequency, 1.8f * frequency};
  (*g_vsParams)["wavenumber"].InitializeBuffer(3);
  (*g_vsParams)["wavenumber"].UpdateBuffer(wavenumberV);

  float omegaV[3] = {0.5f, 1.7f, 1.0f};
  (*g_vsParams)["omega"].InitializeBuffer(3);
  (*g_vsParams)["omega"].UpdateBuffer(omegaV);

  float dir0[2] = {-1.0f, 0.0f};
  (*g_vsParams)["dir0"].InitializeBuffer(2);
  (*g_vsParams)["dir0"].UpdateBuffer(dir0);

  float dir1[2] = {-0.7, 0.7};
  (*g_vsParams)["dir1"].InitializeBuffer(2);
  (*g_vsParams)["dir1"].UpdateBuffer(dir1);

  float dir2[2] = {0.7, 0.7};
  (*g_vsParams)["dir2"].InitializeBuffer(2);
  (*g_vsParams)["dir2"].UpdateBuffer(dir2);

  float steepness = 1.0f;
  float steepnessV[3] = {steepness, 1.5f * steepness, 0.8f * steepness};
  (*g_vsParams)["steepness"].InitializeBuffer(3);
  (*g_vsParams)["steepness"].UpdateBuffer(steepnessV);

  float tau = 2.0f;
  (*g_vsParams)["tau"] = tau;

  // camera_position_object_space is a constant defined by ogre.
  (*g_vsParams)["camera_position_object_space"] = 1;

  (*g_vsParams)["t"] = 0.0f;
  g_startTime = std::chrono::steady_clock::now();

  // set fragment shader params
  g_fsParams = shader->FragmentShaderParams();

  float hdrMultiplier = 0.4f;
  (*g_fsParams)["hdrMultiplier"] = hdrMultiplier;

  float fresnelPower = 5.0f;
  (*g_fsParams)["fresnelPower"] = fresnelPower;

  float shallowColor[4] = {0.0f, 0.1f, 0.3f, 1.0f};
  (*g_fsParams)["shallowColor"].InitializeBuffer(4);
  (*g_fsParams)["shallowColor"].UpdateBuffer(shallowColor);

  float deepColor[4] = {0.0f, 0.05f, 0.2f, 1.0f};
  (*g_fsParams)["deepColor"].InitializeBuffer(4);
  (*g_fsParams)["deepColor"].UpdateBuffer(deepColor);

  std::string bumpMapPath = ignition::common::joinPaths(RESOURCE_PATH,
      "wave_normals.dds");
  (*g_fsParams)["bumpMap"].SetTexture(bumpMapPath);

  std::string cubeMapPath = ignition::common::joinPaths(RESOURCE_PATH,
      "skybox_lowres.dds");

  (*g_fsParams)["cubeMap"].SetTexture(cubeMapPath,
      ir::ShaderParam::ParamType::PARAM_TEXTURE_CUBE, 1u);
}

//////////////////////////////////////////////////
void initContext()
{
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Waves");
  glutDisplayFunc(displayCB);
  glutIdleFunc(idleCB);
  glutKeyboardFunc(keyboardCB);

  glutMouseFunc(mouseCB);
  glutMotionFunc(motionCB);
}

//////////////////////////////////////////////////
void printUsage()
{
  std::cout << "===============================" << std::endl;
  std::cout << "  ESC - Exit                   " << std::endl;
  std::cout << "===============================" << std::endl;
}

//////////////////////////////////////////////////
void run(std::vector<ir::CameraPtr> _cameras)
{
  if (_cameras.empty())
  {
    ignerr << "No cameras found. Scene will not be rendered" << std::endl;
    return;
  }

#if __APPLE__
  g_context = CGLGetCurrentContext();
#elif _WIN32
  g_context = wglGetCurrentContext();
  g_display = wglGetCurrentDC();
#else
  g_context = glXGetCurrentContext();
  g_display = glXGetCurrentDisplay();
  g_drawable = glXGetCurrentDrawable();
#endif

  g_cameras = _cameras;
  initCamera(_cameras[0]);
  initUniforms();
  initContext();
  printUsage();

#if __APPLE__
  g_glutContext = CGLGetCurrentContext();
#elif _WIN32
  g_glutContext = wglGetCurrentContext();
  g_glutDisplay = wglGetCurrentDC();
#else
  g_glutDisplay = glXGetCurrentDisplay();
  g_glutDrawable = glXGetCurrentDrawable();
  g_glutContext = glXGetCurrentContext();
#endif

  glutMainLoop();
}
