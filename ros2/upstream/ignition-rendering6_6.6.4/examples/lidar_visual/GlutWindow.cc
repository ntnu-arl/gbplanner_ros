/*
 * Copyright (C) 2020 Open Source Robotics Foundation
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

#if not defined(__APPLE__) && not defined(_WIN32)
  #include <GL/glx.h>
#endif

#include <iomanip>
#include <mutex>
#include <map>
#include <string>

#include <ignition/math/Matrix4.hh>
#include <ignition/common/Console.hh>
#include <ignition/rendering/Camera.hh>
#include <ignition/rendering/Image.hh>
#include <ignition/rendering/OrbitViewController.hh>
#include <ignition/rendering/RayQuery.hh>
#include <ignition/rendering/Scene.hh>
#include <ignition/rendering.hh>

using namespace ignition;
using namespace rendering;

#include "GlutWindow.hh"
#include "example_config.hh"

#define KEY_ESC 27
#define KEY_TAB  9

//////////////////////////////////////////////////
unsigned int imgw = 0;
unsigned int imgh = 0;

std::vector<ir::CameraPtr> g_cameras;
ir::CameraPtr g_camera;
unsigned int g_cameraIndex = 0;
ir::ImagePtr g_image;

bool g_initContext = false;
bool g_clear = false;
std::vector<double> g_lidarData;
ir::LidarVisualPtr g_lidar;
bool g_lidarVisualUpdateDirty = false;
bool g_showNonHitting = true;
LidarVisualType g_lidarVisType = LidarVisualType::LVT_TRIANGLE_STRIPS;

std::chrono::steady_clock::duration g_time{0};
std::chrono::steady_clock::time_point g_startTime;
double prevUpdateTime;

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
void updateLidarVisual()
{
  g_startTime = std::chrono::steady_clock::now();

  // change detected due to key press
  if (g_lidarVisualUpdateDirty)
  {
    if (g_clear == true)
    {
      g_lidar->ClearPoints();
    }
    else
    {
      g_lidar->SetDisplayNonHitting(g_showNonHitting);
      g_lidar->SetPoints(g_lidarData);
      g_lidar->SetType(g_lidarVisType);
      g_lidar->Update();
    }
    g_lidarVisualUpdateDirty = false;
    g_clear = false;
    g_time = std::chrono::steady_clock::now() - g_startTime;
    prevUpdateTime = std::chrono::duration_cast<std::chrono::microseconds>(
        g_time).count();
  }
}

//////////////////////////////////////////////////
void drawText(int _x, int _y, const std::string &_text)
{
  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0.0, imgw, 0.0, imgh);
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glLoadIdentity();
  glColor3f(1.0f, 1.0f, 1.0f);
  glRasterPos2i(_x, _y);
  void *font = GLUT_BITMAP_9_BY_15;
  for (auto c : _text)
    glutBitmapCharacter(font, c);
  glMatrixMode(GL_MODELVIEW);
  glPopMatrix();
  glMatrixMode(GL_PROJECTION);
  glPopMatrix();
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

  updateLidarVisual();
  g_cameras[g_cameraIndex]->Capture(*g_image);
  handleMouse();

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

  std::stringstream text;
  text << std::setw(30) << "Update time (microseconds): " << prevUpdateTime;
  drawText(10, 10, text.str());

  glutSwapBuffers();
}

//////////////////////////////////////////////////
void keyboardCB(unsigned char _key, int, int)
{
  if (_key == KEY_ESC || _key == 'q' || _key == 'Q')
  {
    exit(0);
  }
  else if (_key == 'h' || _key == 'H')
  {
    g_showNonHitting = !g_showNonHitting;
    g_lidarVisualUpdateDirty = true;
  }
  else if (_key == 'c' || _key == 'C')
  {
    g_clear = true;
    g_lidarVisualUpdateDirty = true;
  }
  else if (_key == '0')
  {
    g_lidarVisType = LidarVisualType::LVT_NONE;
    g_lidarVisualUpdateDirty = true;
    ignmsg << "Set lidar visual type to NONE"
           << std::endl;
  }
  else if (_key == '1')
  {
    g_lidarVisType = LidarVisualType::LVT_RAY_LINES;
    g_lidarVisualUpdateDirty = true;
    ignmsg << "Set lidar visual type to RAY_LINES"
           << std::endl;
  }
  else if (_key == '2')
  {
    g_lidarVisType = LidarVisualType::LVT_POINTS;
    g_lidarVisualUpdateDirty = true;
    ignmsg << "Set lidar visual type to POINTS"
           << std::endl;
  }
  else if (_key == '3')
  {
    g_lidarVisType = LidarVisualType::LVT_TRIANGLE_STRIPS;
    g_lidarVisualUpdateDirty = true;
    ignmsg << "Set lidar visual type to TRIANGLE_STRIPS"
           << std::endl;
  }
  else if (_key == KEY_TAB)
  {
    g_cameraIndex = (g_cameraIndex + 1) % g_cameras.size();
  }
}

//////////////////////////////////////////////////
void idleCB()
{
  glutPostRedisplay();
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
void initLidarVisual(ir::LidarVisualPtr _lidar)
{
  g_lidar = _lidar;
}

//////////////////////////////////////////////////
void initContext()
{
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Lidar Visual Example");
  glutDisplayFunc(displayCB);
  glutIdleFunc(idleCB);
  glutKeyboardFunc(keyboardCB);
  glutMouseFunc(mouseCB);
  glutMotionFunc(motionCB);
}

//////////////////////////////////////////////////
void printUsage()
{
  std::cout << "==========================================" << std::endl;
  std::cout << "  TAB - Switch render engines             " << std::endl;
  std::cout << "  ESC - Exit                              " << std::endl;
  std::cout << "                                          " << std::endl;
  std::cout << "  H: Toggle display for non-hitting rays  " << std::endl;
  std::cout << "  C: Clear Visual                         " << std::endl;
  std::cout << "                                          " << std::endl;
  std::cout << "  0: Do not display visual                " << std::endl;
  std::cout << "  1: Display ray lines visual             " << std::endl;
  std::cout << "  2: Display points visual                " << std::endl;
  std::cout << "  3: Display triangle strips visual       " << std::endl;
  std::cout << "==========================================" << std::endl;
}

//////////////////////////////////////////////////
void run(std::vector<ir::CameraPtr> _cameras,
        std::vector<ir::LidarVisualPtr> _nodes,
        std::vector<double> _pts)
{
  if (_cameras.empty())
  {
    ignerr << "No cameras found. Scene will not be rendered" << std::endl;
    return;
  }

#if __APPLE__
  g_context = CGLGetCurrentContext();
#elif _WIN32
#else
  g_context = glXGetCurrentContext();
  g_display = glXGetCurrentDisplay();
  g_drawable = glXGetCurrentDrawable();
#endif

  g_cameras = _cameras;

  initCamera(_cameras[0]);
  initLidarVisual(_nodes[0]);
  initContext();
  printUsage();
  g_lidarData.clear();
  for (int pt =0; pt < _pts.size(); pt++)
  {
    g_lidarData.push_back(_pts[pt]);
  }

#if __APPLE__
  g_glutContext = CGLGetCurrentContext();
#elif _WIN32
#else
  g_glutDisplay = glXGetCurrentDisplay();
  g_glutDrawable = glXGetCurrentDrawable();
  g_glutContext = glXGetCurrentContext();
#endif

  glutMainLoop();
}
