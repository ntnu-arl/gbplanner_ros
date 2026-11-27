/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
  #include <GL/gl.h>
  #include <GL/glut.h>
#endif

#if !defined(__APPLE__) && !defined(_WIN32)
  #include <GL/glx.h>
#endif

#include <ctime>
#include <string>
#include <vector>

#include <gz/rendering.hh>

#include "ManualSceneDemo.hh"
#include "DemoWindow.hh"

#define KEY_ESC 27
#define KEY_TAB  9

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
ManualSceneDemoPtr g_demo;
ImagePtr g_image;

unsigned int imgw = 0;
unsigned int imgh = 0;

bool g_initContext = false;

#if __APPLE__
  CGLContextObj g_context;
  CGLContextObj g_glutContext;
#elif _WIN32
#else
  GLXContext g_context;
  Display *g_display;
  GLXDrawable g_drawable;
  GLXContext g_glutContext;
  Display *g_glutDisplay;
  GLXDrawable g_glutDrawable;
#endif

double g_offset = 0.0;
double g_fps = 0.0;

const int g_fpsSize = 10;
std::vector<double> g_fpsQueue(g_fpsSize);
int g_fpsIndex = 0;
int g_fpsCount = 0;
clock_t g_prevTime;

//////////////////////////////////////////////////
void printUsage()
{
  std::cout << "===============================" << std::endl;
  std::cout << "  TAB : Switch render engines  " << std::endl;
  std::cout << "   -  : Previous scene         " << std::endl;
  std::cout << "   +  : Next scene             " << std::endl;
  std::cout << "  0-9 : Select scenes 0-9      " << std::endl;
  std::cout << "  ESC : Exit                   " << std::endl;
  std::cout << "===============================" << std::endl;
}

//////////////////////////////////////////////////
void printTextImpl(const std::string &_text, const int _x, const int _y)
{
  glWindowPos2i(_x, _y);
  const char *ctext = _text.c_str();
  while (*ctext) glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *ctext++);
}

//////////////////////////////////////////////////
void printTextBack(const std::string &_text, const int _x, const int _y)
{
  glColor3f(0, 0, 0);

  for (int i = -2; i < 3; ++i)
  {
    for (int j = -2; j < 3; ++j)
    {
      printTextImpl(_text, _x + i, _y + j);
    }
  }
}

//////////////////////////////////////////////////
void printTextFore(const std::string &_text, const int _x, const int _y)
{
  glColor3f(1, 1, 1);
  printTextImpl(_text, _x, _y);
}

//////////////////////////////////////////////////
void printText(const std::string &_text, const int _x, const int _y)
{
  printTextBack(_text, _x, _y);
  printTextFore(_text, _x, _y);
}

//////////////////////////////////////////////////
void printEngine()
{
  int y = imgh - 20;
  std::string text = "Engine: " +
      g_demo->CurrentCamera()->Scene()->Engine()->Name();
  printText(text, 10, y);
}

//////////////////////////////////////////////////
void updateFPS()
{
  clock_t currTime = clock();
  double elapsedTime = static_cast<double>(currTime - g_prevTime)
      / CLOCKS_PER_SEC;
  g_fpsQueue[g_fpsIndex] = 1 / elapsedTime;
  g_fpsCount = (g_fpsCount >= g_fpsSize) ? g_fpsSize : g_fpsCount + 1;
  g_fpsIndex = (g_fpsIndex + 1) % g_fpsSize;
  g_prevTime = currTime;
}

//////////////////////////////////////////////////
void printFPS()
{
  double total = 0;
  updateFPS();

  for (int i = 0; i < g_fpsCount; ++i)
  {
    total += g_fpsQueue[i];
  }

  int y = imgh - 40;
  double fps = total / g_fpsCount;
  std::string fpsText = "FPS: " + std::to_string(fps);
  printText(fpsText, 10, y);
}

//////////////////////////////////////////////////
void displayCB()
{
#if __APPLE__
  CGLSetCurrentContext(g_context);
#elif _WIN32
#else
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  g_demo->Update();
  CameraPtr camera = g_demo->CurrentCamera();

  camera->Capture(*g_image);

#if __APPLE__
  CGLSetCurrentContext(g_glutContext);
#elif _WIN32
#else
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif

  unsigned char *data = g_image->Data<unsigned char>();

  glClearColor(0.5, 0.5, 0.5, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glPixelZoom(1, -1);
  glRasterPos2f(-1, 1);
  glDrawPixels(imgw, imgh, GL_RGB, GL_UNSIGNED_BYTE, data);

  printEngine();
  printFPS();

  glutSwapBuffers();
}

//////////////////////////////////////////////////
void idleCB()
{
  glutPostRedisplay();
}

//////////////////////////////////////////////////
void keyboardCB(unsigned char _key, int, int)
{
#if __APPLE__
    CGLSetCurrentContext(g_context);
#elif _WIN32
#else
  if (g_display)
  {
    glXMakeCurrent(g_display, g_drawable, g_context);
  }
#endif

  switch (_key)
  {
    case KEY_ESC:
    case 'q':
    case 'Q':
      exit(0);

    case KEY_TAB:
      g_demo->NextCamera();
      g_fpsIndex = 0;
      g_fpsCount = 0;

      // for (int i = 0; i < g_fpsSize; ++i) g_fpsQueue[i] = 0;

      break;

    case '-':
    case '_':
      g_demo->PrevScene();
      break;

    case '=':
    case '+':
      g_demo->NextScene();
      break;

    default:
      break;
  }

  if ('0' <= _key && _key <= '9')
  {
    int index = (_key - '1') % 10;
    g_demo->SelectScene(index);
  }

#if __APPLE__
  CGLSetCurrentContext(g_glutContext);
#elif _WIN32
#else
  glXMakeCurrent(g_glutDisplay, g_glutDrawable, g_glutContext);
#endif
}

//////////////////////////////////////////////////
void initCamera(CameraPtr _camera)
{
  imgw = _camera->ImageWidth();
  imgh = _camera->ImageHeight();
  Image image = _camera->CreateImage();
  g_image = std::make_shared<Image>(image);
  _camera->Capture(*g_image);
}

//////////////////////////////////////////////////
void initContext()
{
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Custom Scene Viewer");
  glutDisplayFunc(displayCB);
  glutIdleFunc(idleCB);
  glutKeyboardFunc(keyboardCB);

#if not (__APPLE__ || _WIN32)
  glewInit();
#endif
}

//////////////////////////////////////////////////
void run(ManualSceneDemoPtr _demo)
{
#if __APPLE__
  g_context = CGLGetCurrentContext();
#endif
#if not (__APPLE__ || _WIN32)
  g_context = glXGetCurrentContext();
  g_display = glXGetCurrentDisplay();
  g_drawable = glXGetCurrentDrawable();
#endif

  g_prevTime = clock();
  g_demo = _demo;
  initCamera(_demo->CurrentCamera());
  initContext();
  printUsage();

#if __APPLE__
  g_glutContext = CGLGetCurrentContext();
#endif
#if not (__APPLE__ || _WIN32)
  g_glutDisplay = glXGetCurrentDisplay();
  g_glutDrawable = glXGetCurrentDrawable();
  g_glutContext = glXGetCurrentContext();
#endif

  glutMainLoop();
}
