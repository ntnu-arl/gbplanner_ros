/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#include "GlutWindow.hh"

#define KEY_ESC 27
#define KEY_TAB  9

//////////////////////////////////////////////////
unsigned int imgw = 0;
unsigned int imgh = 0;

std::vector<ir::CameraPtr> g_cameras;
ir::CameraPtr g_camera;
ir::CameraPtr g_currCamera;
unsigned int g_cameraIndex = 0;
ir::ImagePtr g_image;
ir::ShaderParamsPtr g_shaderParams;

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

double g_offset = 0.0;

int g_seed[1] = {0};
float g_resolution[2] = {400, 200};
float g_color[3] = {1.0, 1.0, 1.0};
float g_adjustments[16] = {
  0, 0, 0, 0.0005,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0
};

//////////////////////////////////////////////////
//! [update camera]
void updateCameras()
{
  double angle = g_offset / 2 * M_PI;
  double x = sin(angle) * 3.0 + 3.0;
  double y = cos(angle) * 3.0;
  for (ir::CameraPtr camera : g_cameras)
  {
    camera->SetLocalPosition(x, y, 0.0);
  }

  g_offset += 0.0005;
}
//! [update camera]

//////////////////////////////////////////////////
//! [update uniforms]
void updateUniforms()
{
  if (!g_shaderParams)
    return;
  (*g_shaderParams)["u_seed"].UpdateBuffer(g_seed);
  (*g_shaderParams)["u_resolution"].UpdateBuffer(g_resolution);
  (*g_shaderParams)["u_color"].UpdateBuffer(g_color);
  (*g_shaderParams)["u_adjustments"].UpdateBuffer(g_adjustments);
}
//! [update uniforms]

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
  updateCameras();
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
  else if (_key == 'o')
  {
    g_adjustments[3] -= 0.0001;
  }
  else if (_key == 'p')
  {
    g_adjustments[3] += 0.0001;
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
  ir::VisualPtr sphere =
      std::dynamic_pointer_cast<ir::Visual>(node->ChildByName("box"));
  ir::MaterialPtr shader = sphere->Material();
  if (!shader)
    return;
  g_shaderParams = shader->FragmentShaderParams();

  (*g_shaderParams)["u_seed"].InitializeBuffer(1);
  (*g_shaderParams)["u_resolution"].InitializeBuffer(2);
  (*g_shaderParams)["u_color"].InitializeBuffer(3);
  (*g_shaderParams)["u_adjustments"].InitializeBuffer(16);

  auto engine = g_camera->Scene()->Engine();
  if (engine->Name() == "ogre2")
  {
    // worldviewproj_matrix is a constant defined by ogre.
    // Here we add a line to add this constant to the params.
    // The specified value is ignored as it will be auto bound to the
    // correct type and value. See available constants:
    // https://github.com/OGRECave/ogre-next/blob/v2-2/OgreMain/src/OgreGpuProgramParams.cpp
    auto params = shader->VertexShaderParams();
    (*params)["worldviewproj_matrix"] = 1;
  }
}

//////////////////////////////////////////////////
void initContext()
{
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Custom Shaders Uniforms");
  glutDisplayFunc(displayCB);
  glutIdleFunc(idleCB);
  glutKeyboardFunc(keyboardCB);
}

//////////////////////////////////////////////////
void printUsage()
{
  std::cout << "===============================" << std::endl;
  std::cout << "  ESC - Exit                   " << std::endl;
  std::cout << "                               " << std::endl;
  std::cout << "  o - Decrease Frequency       " << std::endl;
  std::cout << "  p - Increase Frequency       " << std::endl;
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
