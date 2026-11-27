/*
 * Copyright (C) 2021 Rhys Mainwaring
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

#include "ThreadRenderer.h"

#include <QGuiApplication>
#include <QQuickView>
#include <QThread>

#include <exception>
#include <iostream>

int main(int _argc, char** _argv)
{
  try
  {
    // use single-threaded scene graph rendering
    qputenv("QSG_RENDER_LOOP", "basic");

    // requested surface format
    QSurfaceFormat format = RenderThread::CreateSurfaceFormat();
    QSurfaceFormat::setDefaultFormat(format);
    RenderThread::Print(format);

    qmlRegisterType<ThreadRenderer>("IgnitionRendering", 1, 0,
        "ThreadRenderer");

    QGuiApplication app(_argc, _argv);

    int execReturn = 0;
    {
      QQuickView view;

      // Rendering in a thread introduces a slightly more complicated cleanup
      // so we ensure that no cleanup of graphics resources happen until the
      // application is shutting down.
      view.setPersistentOpenGLContext(true);
      view.setPersistentSceneGraph(true);

      view.setResizeMode(QQuickView::SizeRootObjectToView);
      view.setSource(QUrl("qrc:/Main.qml"));
      view.show();

      execReturn = app.exec();
    }

    // As the render threads make use of our QGuiApplication object
    // to clean up gracefully, wait for them to finish before
    // QGuiApp is taken off the heap.
    for (QThread *t : qAsConst(ThreadRenderer::threads))
    {
      t->wait();
      delete t;
      t = nullptr;
    }

    return execReturn;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
  }
  catch (...)
  {
    std::cerr << "Unknown exception"  << '\n';
  }
  return -1;
}
