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

#include <iomanip>
#include <mutex>
#include <map>
#include <string>

#include <gz/math/Matrix4.hh>
#include <gz/common/Console.hh>
#include <gz/common/Mesh.hh>
#include <gz/common/Skeleton.hh>
#include <gz/common/SkeletonAnimation.hh>
#include <gz/rendering/Camera.hh>
#include <gz/rendering/Image.hh>
#include <gz/rendering/OrbitViewController.hh>
#include <gz/rendering/RayQuery.hh>
#include <gz/rendering/Scene.hh>

#include "GlutWindow.hh"
#include "example_config.hh"

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
std::vector<ir::VisualPtr> g_visuals;
std::vector<ir::VisualPtr> g_allVisuals;
ic::SkeletonPtr g_skel;
ic::SkeletonAnimation *g_skelAnim;
unsigned int g_animIdx = 0;
bool g_updateAll = false;
bool g_manualBoneUpdate = false;
bool g_rootBoneWeight = 1.0;
bool g_actorUpdateDirty = true;
std::chrono::steady_clock::duration g_time{0};
std::chrono::steady_clock::time_point g_startTime;
std::chrono::steady_clock::time_point prevUpdateTime;

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
    g_rayQuery->SetFromCamera(rayCamera, gz::math::Vector2d(nx, ny));
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
    auto drag = gz::math::Vector2d(g_mouse.dragX, g_mouse.dragY);

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
void updatePose(double _time)
{
  // manually update the bone pose
  for (auto &v : g_visuals)
  {
  //! [update pose]
    std::map<std::string, ignition::math::Matrix4d> animFrames;
    animFrames = g_skelAnim->PoseAt(_time, true);
    std::map<std::string, ignition::math::Matrix4d> skinFrames;
    for (auto pair : animFrames)
    {
      std::string animNodeName = pair.first;
      auto animTf = pair.second;

      std::string skinName =
          g_skel->NodeNameAnimToSkin(g_animIdx, animNodeName);
      ignition::math::Matrix4d skinTf =
              g_skel->AlignTranslation(g_animIdx, animNodeName)
              * animTf * g_skel->AlignRotation(g_animIdx, animNodeName);

      skinFrames[skinName] = skinTf;
    }

    // set bone transforms
    ir::MeshPtr mesh =
        std::dynamic_pointer_cast<ir::Mesh>(v->GeometryByIndex(0));
    mesh->SetSkeletonLocalTransforms(skinFrames);
  //! [update pose]
  }
}

//////////////////////////////////////////////////
void updateTime(double _time)
{
  // set time to advance animation
  for (auto &v : g_visuals)
  {
  //! [update actor]
    ir::MeshPtr mesh =
        std::dynamic_pointer_cast<ir::Mesh>(v->GeometryByIndex(0));
    mesh->UpdateSkeletonAnimation(
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(_time)));
  //! [update actor]
  }
}

//////////////////////////////////////////////////
void updateActor()
{
  g_time = std::chrono::steady_clock::now() - g_startTime;
  auto seconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(g_time).count() /
      1000.0;

  g_skelAnim = g_skel->Animation(g_animIdx);

  // change detected due to key press
  if (g_actorUpdateDirty)
  {
    // disable all auto animations
    for (auto &v : g_allVisuals)
    {
      ir::MeshPtr mesh =
          std::dynamic_pointer_cast<ir::Mesh>(v->GeometryByIndex(0));
      for (unsigned int i = 0; i < g_skel->AnimationCount(); ++i)
      {
        auto anim = g_skel->Animation(i);

        // disable all
        mesh->SetSkeletonAnimationEnabled(anim->Name(), false, false, 0.0);
      }
      v->SetVisible(false);
    }

    // set visuals to be updated
    g_visuals.clear();
    if (g_updateAll)
    {
      std::copy(g_allVisuals.begin(), g_allVisuals.end(),
          std::back_inserter(g_visuals));
    }
    else
    {
      unsigned int idx = static_cast<unsigned int>(
          sqrt(g_allVisuals.size()) * 0.5);
      g_visuals.push_back(g_allVisuals[idx]);
    }

    // enabled selected animation
    for (auto &v : g_visuals)
    {
      v->SetVisible(true);
      ir::MeshPtr mesh =
          std::dynamic_pointer_cast<ir::Mesh>(v->GeometryByIndex(0));
      if (!g_manualBoneUpdate)
      {
        mesh->SetSkeletonAnimationEnabled(g_skelAnim->Name(), true, true, 1.0);
      }
      // update root bone weight
      std::unordered_map<std::string, float> weights;
      weights[g_skel->RootNode()->Name()] = g_rootBoneWeight;
      mesh->SetSkeletonWeights(weights);
    }


    g_actorUpdateDirty = false;
  }

  // manually update skeleton bone pose
  if (g_manualBoneUpdate)
  {
    updatePose(fmod(seconds, g_skelAnim->Length()));
  }
  // advance time for built in animations
  else
  {
    updateTime(seconds);
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

  updateActor();

  // draw FPS
  auto now = std::chrono::steady_clock::now();
  double t = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - prevUpdateTime).count() / 1000.0;
  prevUpdateTime = now;
  std::string manual = g_manualBoneUpdate ? "true" : "false";
  std::stringstream text;
  text << std::fixed << std::setw(5) << std::setprecision(4) << (1.0/t);
  text << std::setw(30) << "Manual skeleton update: " << manual;
  text << std::setw(30) << "Root bone weight:: " << std::setprecision(2)
       << g_rootBoneWeight;
  drawText(10, 10, text.str());

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
  if (_key == KEY_ESC || _key == 'q' || _key == 'Q')
  {
    exit(0);
  }
  else if (_key == KEY_TAB)
  {
    g_cameraIndex = (g_cameraIndex + 1) % g_cameras.size();
  }
  else if (_key == 'a')
  {
    g_animIdx = (g_animIdx + 1) % 2;
    auto anim = g_skel->Animation(g_animIdx);
    g_actorUpdateDirty = true;
    std::cout << "Playing animation: " << anim->Name() << std::endl;
  }
  else if (_key == 'm')
  {
    g_manualBoneUpdate = !g_manualBoneUpdate;
    g_actorUpdateDirty = true;
    std::cout << "Manual skeleton bone update: " << g_manualBoneUpdate
              << std::endl;
  }
  else if (_key == 't')
  {
    g_updateAll = !g_updateAll;
    g_actorUpdateDirty = true;
    std::cout << "Update all meshes " << g_updateAll << std::endl;
  }
  else if (_key == 'r')
  {
    g_actorUpdateDirty = true;
    g_rootBoneWeight = g_rootBoneWeight ^ 1;
    std::cout << "Setting root bone weight to: " << g_rootBoneWeight
              << std::endl;
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
void initContext()
{
  glutInitDisplayMode(GLUT_DOUBLE);
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(imgw, imgh);
  glutCreateWindow("Actor animation");
  glutDisplayFunc(displayCB);
  glutIdleFunc(idleCB);
  glutKeyboardFunc(keyboardCB);

  glutMouseFunc(mouseCB);
  glutMotionFunc(motionCB);
}

//////////////////////////////////////////////////
void initAnimation()
{
  if (!g_skel || g_skel->AnimationCount() == 0)
  {
    std::cerr << "Failed to load animation." << std::endl;
    return;
  }

  g_skelAnim = g_skel->Animation(g_animIdx);

  g_startTime = std::chrono::steady_clock::now();
}

//////////////////////////////////////////////////
void printUsage()
{
  std::cout << "==========================================" << std::endl;
  std::cout << "  TAB - Switch render engines             " << std::endl;
  std::cout << "  ESC - Exit                              " << std::endl;
  std::cout << "  A   - Switch animation                  " << std::endl;
  std::cout << "  M   - Toggle manual skeleton update     " << std::endl;
  std::cout << "  T   - Toggle animated mesh count        " << std::endl;
  std::cout << "  R   - Toggle root bone weight           " << std::endl;
  std::cout << "        (non-manual skeleton update only) " << std::endl;
  std::cout << "==========================================" << std::endl;
}

//////////////////////////////////////////////////
void run(std::vector<ir::CameraPtr> _cameras,
         const std::vector<ir::VisualPtr> &_visuals,
         ic::SkeletonPtr _skel)
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
  g_allVisuals = _visuals;
  g_skel = _skel;

  initCamera(_cameras[0]);
  initContext();
  initAnimation();
  printUsage();

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
