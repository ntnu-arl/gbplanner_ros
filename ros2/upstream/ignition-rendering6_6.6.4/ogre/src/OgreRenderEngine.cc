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

// Not Apple or Windows
#if !defined(__APPLE__) && !defined(_WIN32)
# include <X11/Xlib.h>
# include <X11/Xutil.h>
# include <GL/glx.h>
#endif

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

# include <sstream>

#include <gz/plugin/Register.hh>

#include <gz/common/Console.hh>
#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>

#include "gz/rendering/RenderEngineManager.hh"
#include "gz/rendering/ogre/OgreIncludes.hh"
#include "gz/rendering/ogre/OgreRenderEngine.hh"
#include "gz/rendering/ogre/OgreRenderTypes.hh"
#include "gz/rendering/ogre/OgreRTShaderSystem.hh"
#include "gz/rendering/ogre/OgreScene.hh"
#include "gz/rendering/ogre/OgreStorage.hh"

class gz::rendering::OgreRenderEnginePrivate
{
#if !defined(__APPLE__) && !defined(_WIN32)
  public: XVisualInfo *dummyVisual = nullptr;
#endif

  /// \brief A list of supported fsaa levels
  public: std::vector<unsigned int> fsaaLevels;
};

using namespace gz;
using namespace rendering;

//////////////////////////////////////////////////
OgreRenderEnginePlugin::OgreRenderEnginePlugin()
{
}

//////////////////////////////////////////////////
std::string OgreRenderEnginePlugin::Name() const
{
  return OgreRenderEngine::Instance()->Name();
}

//////////////////////////////////////////////////
RenderEngine *OgreRenderEnginePlugin::Engine() const
{
  return OgreRenderEngine::Instance();
}

//////////////////////////////////////////////////
OgreRenderEngine::OgreRenderEngine() :
  dataPtr(new OgreRenderEnginePrivate)
{
  this->ogrePaths.push_back(OGRE_RESOURCE_PATH);

  const char *env = std::getenv("OGRE_RESOURCE_PATH");
  if (env)
    this->ogrePaths.push_back(std::string(env));
}

//////////////////////////////////////////////////
OgreRenderEngine::~OgreRenderEngine()
{
}

//////////////////////////////////////////////////
void OgreRenderEngine::Destroy()
{
  BaseRenderEngine::Destroy();

  if (this->scenes)
  {
    this->scenes->RemoveAll();
  }

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
  delete this->ogreOverlaySystem;
  this->ogreOverlaySystem = nullptr;
#endif

  OgreRTShaderSystem::Instance()->Fini();

  if (ogreRoot)
  {
    try
    {
      // TODO(anyone): do we need to catch segfault on delete?
      delete this->ogreRoot;
    }
    catch (...)
    {
    }
    this->ogreRoot = nullptr;
  }

  delete this->ogreLogManager;
  this->ogreLogManager = nullptr;

#if (!defined(__APPLE__) && !defined(_WIN32))
  if (this->dummyDisplay)
  {
    Display *x11Display = static_cast<Display*>(this->dummyDisplay);
    GLXContext x11Context = static_cast<GLXContext>(this->dummyContext);
    glXDestroyContext(x11Display, x11Context);
    XDestroyWindow(x11Display, this->dummyWindowId);
    XCloseDisplay(x11Display);
    this->dummyDisplay = nullptr;
    XFree(this->dataPtr->dummyVisual);
    this->dataPtr->dummyVisual = nullptr;
  }
#endif
}

//////////////////////////////////////////////////
bool OgreRenderEngine::IsEnabled() const
{
  return this->initialized && this->renderPathType != NONE;
}

//////////////////////////////////////////////////
std::string OgreRenderEngine::Name() const
{
  return "ogre";
}

//////////////////////////////////////////////////
OgreRenderEngine::OgreRenderPathType
    OgreRenderEngine::RenderPathType() const
{
  return this->renderPathType;
}

//////////////////////////////////////////////////
void OgreRenderEngine::AddResourcePath(const std::string &_uri)
{
  if (_uri == "__default__" || _uri.empty())
    return;

  std::string path = common::findFilePath(_uri);

  if (path.empty())
  {
    ignerr << "URI doesn't exist[" << _uri << "]\n";
    return;
  }

  this->resourcePaths.push_back(path);

  try
  {
    if (!Ogre::ResourceGroupManager::getSingleton().resourceLocationExists(
          path, "General"))
    {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          path, "FileSystem", "General", true);

      Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup(
          "General");
      // Parse all material files in the path if any exist
      if (common::isDirectory(path))
      {
        std::vector<std::string> paths;

        common::DirIter endIter;
        for (common::DirIter dirIter(path); dirIter != endIter; ++dirIter)
        {
          paths.push_back(*dirIter);
        }
        std::sort(paths.begin(), paths.end());

        // Iterate over all the models in the current ign-rendering path
        for (auto dIter = paths.begin(); dIter != paths.end(); ++dIter)
        {
          std::string fullPath = *dIter;
          std::string matExtension = fullPath.substr(fullPath.size()-9);
          if (matExtension == ".material")
          {
            Ogre::DataStreamPtr stream =
              Ogre::ResourceGroupManager::getSingleton().openResource(
                  fullPath, "General");

            // There is a material file under there somewhere, read the thing in
            try
            {
              Ogre::MaterialManager::getSingleton().parseScript(
                  stream, "General");
              Ogre::MaterialPtr matPtr =
                Ogre::MaterialManager::getSingleton().getByName(
                    fullPath);

              bool matPtrNotNull;
#if OGRE_VERSION_LT_1_11_0
              matPtrNotNull = !matPtr.isNull();
#else
              matPtrNotNull = matPtr != nullptr;
#endif
              if (matPtrNotNull)
              {
                // is this necessary to do here? Someday try it without
                matPtr->compile();
                matPtr->load();
              }
            }
            catch(Ogre::Exception& e)
            {
              ignerr << "Unable to parse material file[" << fullPath << "]\n";
            }
            stream->close();
          }
        }
      }
    }
  }
  catch(Ogre::Exception &/*_e*/)
  {
    ignerr << "Unable to load Ogre Resources.\nMake sure the"
        "resources path in the world file is set correctly." << std::endl;
  }
}

//////////////////////////////////////////////////
Ogre::Root *OgreRenderEngine::OgreRoot() const
{
  return this->ogreRoot;
}

//////////////////////////////////////////////////
ScenePtr OgreRenderEngine::CreateSceneImpl(unsigned int _id,
    const std::string &_name)
{
  auto scene = OgreScenePtr(new OgreScene(_id, _name));
  this->scenes->Add(scene);
  return scene;
}

//////////////////////////////////////////////////
SceneStorePtr OgreRenderEngine::Scenes() const
{
  return this->scenes;
}

//////////////////////////////////////////////////
bool OgreRenderEngine::LoadImpl(
    const std::map<std::string, std::string> &_params)
{
  // parse params
  auto it = _params.find("useCurrentGLContext");
  if (it != _params.end())
    std::istringstream(it->second) >> this->useCurrentGLContext;

  try
  {
    this->LoadAttempt();
    return true;
  }
  catch (Ogre::Exception &ex)
  {
    ignerr << ex.what() << std::endl;
    return false;
  }
  catch (...)
  {
    ignerr << "Failed to load render-engine" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
bool OgreRenderEngine::InitImpl()
{
  try
  {
    this->InitAttempt();
    return true;
  }
  catch (...)
  {
    ignerr << "Failed to initialize render-engine" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
void OgreRenderEngine::LoadAttempt()
{
  this->CreateLogger();
  if (!this->useCurrentGLContext)
    this->CreateContext();
  this->CreateRoot();
  this->CreateOverlay();
  this->LoadPlugins();
  this->CreateRenderSystem();
  this->ogreRoot->initialise(false);
  this->CreateResources();
  this->CreateRenderWindow();
  this->CheckCapabilities();
}

//////////////////////////////////////////////////
void OgreRenderEngine::CreateLogger()
{
  // create log file path
  std::string logPath;
  common::env(IGN_HOMEDIR, logPath);
  logPath = common::joinPaths(logPath, ".ignition", "rendering");
  common::createDirectories(logPath);
  logPath = common::joinPaths(logPath, "ogre.log");

  // create actual log
  this->ogreLogManager = new Ogre::LogManager();
  this->ogreLogManager->createLog(logPath, true, false, false);
}

//////////////////////////////////////////////////
void OgreRenderEngine::CreateContext()
{
#if (!defined(__APPLE__) && !defined(_WIN32))
  // create X11 display
  this->dummyDisplay = XOpenDisplay(0);
  Display *x11Display = static_cast<Display*>(this->dummyDisplay);

  if (!this->dummyDisplay)
  {
    ignerr << "Unable to open display: " << XDisplayName(0) << std::endl;
    return;
  }

  // create X11 visual
  int screenId = DefaultScreen(x11Display);

  int attributeList[] = { GLX_RGBA, GLX_DOUBLEBUFFER, GLX_DEPTH_SIZE, 16,
      GLX_STENCIL_SIZE, 8, None };

  this->dataPtr->dummyVisual =
      glXChooseVisual(x11Display, screenId, attributeList);

  if (!this->dataPtr->dummyVisual)
  {
    ignerr << "Unable to create glx visual" << std::endl;
    return;
  }

  // create X11 context
  this->dummyWindowId = XCreateSimpleWindow(x11Display,
      RootWindow(this->dummyDisplay, screenId), 0, 0, 1, 1, 0, 0, 0);

  this->dummyContext = glXCreateContext(x11Display, this->dataPtr->dummyVisual,
                                        nullptr, 1);

  GLXContext x11Context = static_cast<GLXContext>(this->dummyContext);

  if (!this->dummyContext)
  {
    ignerr << "Unable to create glx context" << std::endl;
    return;
  }

  // select X11 context
  glXMakeCurrent(x11Display, this->dummyWindowId, x11Context);
#endif
}

//////////////////////////////////////////////////
void OgreRenderEngine::CreateRoot()
{
  try
  {
    this->ogreRoot = new Ogre::Root("", "", "");
  }
  catch (Ogre::Exception &ex)
  {
    ignerr << "Unable to create Ogre root" << std::endl;
  }
}

//////////////////////////////////////////////////
void OgreRenderEngine::CreateOverlay()
{
#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
    // OgreOverlay is a component on its own in ogre 1.9 so must manually
    // initialize it. Must be created after this->dataPtr->root,
    // but before this->dataPtr->root is initialized.
  this->ogreOverlaySystem = new Ogre::OverlaySystem();
#endif
}

//////////////////////////////////////////////////
void OgreRenderEngine::LoadPlugins()
{
  for (auto iter = this->ogrePaths.begin();
       iter != this->ogrePaths.end(); ++iter)
  {
    std::string path(*iter);
    if (!common::isDirectory(path))
      continue;

    std::vector<std::string> plugins;
    std::vector<std::string>::iterator piter;

#ifdef __APPLE__
    std::string extension = ".dylib";
#elif _WIN32
    std::string extension = ".dll";
#else
    std::string extension = ".so";
#endif

    plugins.push_back(path+"/RenderSystem_GL");
    plugins.push_back(path+"/Plugin_ParticleFX");
    plugins.push_back(path+"/Plugin_BSPSceneManager");
    plugins.push_back(path+"/Plugin_OctreeSceneManager");

#ifdef HAVE_OCULUS
    plugins.push_back(path+"/Plugin_CgProgramManager");
#endif

    for (piter = plugins.begin(); piter != plugins.end(); ++piter)
    {
      try
      {
        // Load the plugin into OGRE
        this->ogreRoot->loadPlugin(*piter+extension);
      }
      catch(Ogre::Exception &e)
      {
        try
        {
          // Load the debug plugin into OGRE
          this->ogreRoot->loadPlugin(*piter+"_d"+extension);
        }
        catch(Ogre::Exception &ed)
        {
          if ((*piter).find("RenderSystem") != std::string::npos)
          {
            ignerr << "Unable to load Ogre Plugin[" << *piter
                  << "]. Rendering will not be possible."
                  << "Make sure you have installed OGRE properly.\n";
          }
        }
      }
    }
  }
}

//////////////////////////////////////////////////
void OgreRenderEngine::CreateRenderSystem()
{
  Ogre::RenderSystem *renderSys;
  const Ogre::RenderSystemList *rsList;

  // Set parameters of render system (window size, etc.)
#if  OGRE_VERSION_MAJOR == 1 && OGRE_VERSION_MINOR == 6
  rsList = this->ogreRoot->getAvailableRenderers();
#else
  rsList = &(this->ogreRoot->getAvailableRenderers());
#endif

  int c = 0;

  renderSys = nullptr;

  do
  {
    if (c == static_cast<int>(rsList->size()))
      break;

    renderSys = rsList->at(c);
    c++;
  }
  // cpplint has a false positive when extending a while call to multiple lines
  // (it thinks the while loop is empty), so we must put the whole while
  // statement on one line and add NOLINT at the end so that cpplint doesn't
  // complain about the line being too long
  while (renderSys && renderSys->getName().compare("OpenGL Rendering Subsystem") != 0); // NOLINT

  if (renderSys == nullptr)
  {
    ignerr << "unable to find OpenGL rendering system. OGRE is probably "
            "installed incorrectly. Double check the OGRE cmake output, "
            "and make sure OpenGL is enabled." << std::endl;
  }

  // We operate in windowed mode
  renderSys->setConfigOption("Full Screen", "No");

  /// We used to allow the user to set the RTT mode to PBuffer, FBO, or Copy.
  ///   Copy is slow, and there doesn't seem to be a good reason to use it
  ///   PBuffer limits the size of the renderable area of the RTT to the
  ///           size of the first window created.
  ///   FBO seem to be the only good option
  renderSys->setConfigOption("RTT Preferred Mode", "FBO");

  // get all supported fsaa values
  Ogre::ConfigOptionMap configMap = renderSys->getConfigOptions();
  auto fsaaOoption = configMap.find("FSAA");

  if (fsaaOoption != configMap.end())
  {
    auto values = (*fsaaOoption).second.possibleValues;
    for (auto const &str : values)
    {
      int value = 0;
      try
      {
        value = std::stoi(str);
      }
      catch(...)
      {
        continue;
      }
      this->dataPtr->fsaaLevels.push_back(value);
    }
  }
  std::sort(this->dataPtr->fsaaLevels.begin(), this->dataPtr->fsaaLevels.end());

  // check if target fsaa is supported
  unsigned int fsaa = 0;
  unsigned int targetFSAA = 4;
  auto const it = std::find(this->dataPtr->fsaaLevels.begin(),
      this->dataPtr->fsaaLevels.end(), targetFSAA);
  if (it != this->dataPtr->fsaaLevels.end())
    fsaa = targetFSAA;

  renderSys->setConfigOption("FSAA", std::to_string(fsaa));

  this->ogreRoot->setRenderSystem(renderSys);
}

//////////////////////////////////////////////////
void OgreRenderEngine::CreateResources()
{
  std::vector< std::pair<std::string, std::string> > archNames;

  // TODO(anyone) support loading resources from user specified paths
  std::list<std::string> paths;
  const char *env = std::getenv("IGN_RENDERING_RESOURCE_PATH");
  std::string resourcePath = (env) ? std::string(env) :
      IGN_RENDERING_RESOURCE_PATH;
  // install path
  std::string mediaPath = common::joinPaths(resourcePath, "ogre", "media");
  paths.push_back(mediaPath);
  // src path
  mediaPath = common::joinPaths(resourcePath, "ogre", "src", "media");
  paths.push_back(mediaPath);

  for (auto const &p : paths)
  {
    if (!common::isDirectory(p))
      continue;

    archNames.push_back(
        std::make_pair(p, "General"));
    // archNames.push_back(
    //     std::make_pair(prefix + "/skyx", "SkyX"));
    archNames.push_back(
        std::make_pair(p + "/materials/programs", "General"));
    archNames.push_back(
        std::make_pair(p + "/materials/scripts", "General"));
    // archNames.push_back(
    //     std::make_pair(prefix + "/materials/textures", "General"));
    // archNames.push_back(
    //     std::make_pair(prefix + "/media/models", "General"));
    archNames.push_back(
        std::make_pair(p + "/fonts", "Fonts"));
  }

  for (auto aiter = archNames.begin(); aiter != archNames.end(); ++aiter)
  {
    try
    {
      Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
          aiter->first, "FileSystem", aiter->second);
    }
    catch(Ogre::Exception &/*_e*/)
    {
      ignerr << "Unable to load Ogre Resources. Make sure the resources "
          "path in the world file is set correctly." << std::endl;
    }
  }
}

//////////////////////////////////////////////////
void OgreRenderEngine::CreateRenderWindow()
{
  // create dummy window
  auto res = this->CreateRenderWindow(std::to_string(this->dummyWindowId), 1, 1,
      1, 0);
  if (res.empty())
  {
    ignerr << "Failed to create dummy render window." << std::endl;
  }
}

//////////////////////////////////////////////////
std::string OgreRenderEngine::CreateRenderWindow(const std::string &_handle,
    const unsigned int _width, const unsigned int _height,
    const double _ratio, const unsigned int _antiAliasing)
{
  Ogre::StringVector paramsVector;
  Ogre::NameValuePairList params;
  Ogre::RenderWindow *window = nullptr;

  if (!this->useCurrentGLContext)
  {
    // Mac and Windows *must* use externalWindow handle.
#if defined(__APPLE__) || defined(_MSC_VER)
    params["externalWindowHandle"] = _handle;
#else
    params["parentWindowHandle"] = _handle;
#endif
  }

  params["FSAA"] = std::to_string(_antiAliasing);
  params["stereoMode"] = "Frame Sequential";

  // TODO(anyone): determine api without qt

  // Set the macAPI for Ogre based on the Qt implementation
  params["macAPI"] = "cocoa";
  params["macAPICocoaUseNSView"] = "true";

  // Hide window if dimensions are less than or equal to one.
  params["border"] = "none";

  // Needed for retina displays
  params["contentScalingFactor"] = std::to_string(_ratio);

  if (this->useCurrentGLContext)
  {
    params["externalGLControl"] = "true";
    params["currentGLContext"] = "true";
  }

  std::ostringstream stream;
  int attempts = 0;
  while (window == nullptr && attempts < 10)
  {
    stream.str("");
    stream << "OgreWindow(" << attempts << ")" << "_" << _handle;
    try
    {
      window = this->ogreRoot->createRenderWindow(
          stream.str(), _width, _height, false, &params);
    }
    catch(Ogre::Exception &_e)
    {
      ignerr << "Unable to create the rendering window. Attempt [" << attempts
             << "]. Exception [" << _e.what() << "]" << std::endl;
      window = nullptr;
    }
    attempts++;
  }

  if (attempts >= 10)
  {
    ignerr << "Unable to create the rendering window after [" << attempts
           << "] attempts." << std::endl;
    return std::string();
  }

  if (nullptr != window)
  {
    window->setActive(true);
    window->setVisible(true);
    window->setAutoUpdated(false);

    // Windows needs to reposition the render window to 0,0.
    window->reposition(0, 0);
  }
  return stream.str();
}

//////////////////////////////////////////////////
void OgreRenderEngine::CheckCapabilities()
{
  if (nullptr == this->ogreRoot ||nullptr == this->ogreRoot->getRenderSystem())
  {
    ignerr << "No ogreRoot or render system" << std::endl;
    return;
  }

  const Ogre::RenderSystemCapabilities *capabilities;
  Ogre::RenderSystemCapabilities::ShaderProfiles profiles;
  Ogre::RenderSystemCapabilities::ShaderProfiles::const_iterator iter;

  capabilities = this->ogreRoot->getRenderSystem()->getCapabilities();
  if (nullptr == capabilities)
  {
    ignerr << "Failed to get capabilities" << std::endl;
    return;
  }

  profiles = capabilities->getSupportedShaderProfiles();

  bool hasFragmentPrograms =
    capabilities->hasCapability(Ogre::RSC_FRAGMENT_PROGRAM);

  bool hasVertexPrograms =
    capabilities->hasCapability(Ogre::RSC_VERTEX_PROGRAM);

#if OGRE_VERSION_LT_1_11_0
  bool hasFBO =
    capabilities->hasCapability(Ogre::RSC_FBO);
#else
  // All targetted APIs by Ogre support this feature.
  // https://ogrecave.github.io/ogre/api/1.10/deprecated.html#_deprecated000139
  bool hasFBO = true;
#endif

  bool hasGLSL =
    std::find(profiles.begin(), profiles.end(), "glsl") != profiles.end();

  if (!hasFragmentPrograms || !hasVertexPrograms)
  {
    ignwarn << "Vertex and fragment shaders are missing. "
           << "Fixed function rendering will be used.\n";
  }

  if (!hasGLSL)
  {
    ignwarn << "GLSL is missing."
           << "Fixed function rendering will be used.\n";
  }

  // cppcheck-suppress knownConditionTrueFalse
  if (!hasFBO)
  {
    ignwarn << "Frame Buffer Objects (FBO) is missing. "
           << "Rendering will be disabled.\n";
  }

  this->renderPathType = OgreRenderEngine::NONE;

  if (hasFBO && hasGLSL && hasVertexPrograms && hasFragmentPrograms)
  {
    this->renderPathType = OgreRenderEngine::FORWARD;
  }
  else if (hasFBO)
  {
    this->renderPathType = OgreRenderEngine::VERTEX;
  }
}

//////////////////////////////////////////////////
void OgreRenderEngine::InitAttempt()
{
  if (this->renderPathType == NONE)
  {
    ignwarn << "Cannot initialize render engine since "
           << "render path type is NONE. Ignore this warning if"
           << "rendering has been turned off on purpose.\n";
    return;
  }

  this->initialized = false;

  Ogre::ColourValue ambient;

  /// Create a dummy rendering context.
  /// This will allow ign-rendering to run headless. And it also allows OGRE to
  /// initialize properly

  // Set default mipmap level (NB some APIs ignore this)
  Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

  // init the resources
  Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

  Ogre::MaterialManager::getSingleton().setDefaultTextureFiltering(
      Ogre::TFO_ANISOTROPIC);

  OgreRTShaderSystem::Instance()->Init();

  this->scenes = OgreSceneStorePtr(new OgreSceneStore);
}

/////////////////////////////////////////////////
std::vector<unsigned int> OgreRenderEngine::FSAALevels() const
{
  return this->dataPtr->fsaaLevels;
}

#if (OGRE_VERSION >= ((1 << 16) | (9 << 8) | 0))
/////////////////////////////////////////////////
Ogre::OverlaySystem *OgreRenderEngine::OverlaySystem() const
{
  return this->ogreOverlaySystem;
}
#endif

// Register this plugin
IGNITION_ADD_PLUGIN(OgreRenderEnginePlugin,
                    rendering::RenderEnginePlugin)
