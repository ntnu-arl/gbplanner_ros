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

// Elements of this example are derived from the textureinthread example from the Qt Toolkit
// https://code.qt.io/cgit/qt/qtdeclarative.git/tree/examples/quick/scenegraph/textureinthread?h=5.15

/****************************************************************************
**
** Copyright (C) 2017 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef IGNITION_RENDERING_EXAMPLES_SIMPLE_DEMO_QML_THREAD_RENDERER_HH_
#define IGNITION_RENDERING_EXAMPLES_SIMPLE_DEMO_QML_THREAD_RENDERER_HH_

#include "IgnitionRenderer.hh"

#include <QQuickItem>
#include <QMutex>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QQuickWindow>
#include <QSGSimpleTextureNode>
#include <QThread>

//--------------------------------------------------------------------------
/*
 * The render thread shares a context with the scene graph and will
 * render into two separate FBOs, one to use for display and one
 * to use for rendering
 */
class RenderThread : public QThread
{
    Q_OBJECT
public:
    RenderThread(const QSize &_size, QQuickItem *_renderWindowItem);

    QOffscreenSurface *surface = nullptr;
    QOpenGLContext *context = nullptr;

    static void Print(const QSurfaceFormat &_format);
    static QSurfaceFormat CreateSurfaceFormat();

    void InitialiseOnMainThread();

public slots:
    void RenderNext();
    void ShutDown();

signals:
    void TextureReady(int _id, const QSize &_size);

private:
    IgnitionRenderer *renderer = nullptr;
    QSize size;

    /// \brief reference to the render window item
    QQuickItem *renderWindowItem = nullptr;
};

//--------------------------------------------------------------------------
class TextureNode : public QObject, public QSGSimpleTextureNode
{
    Q_OBJECT

public:
    TextureNode(QQuickWindow *_window);
    ~TextureNode() override;

signals:
    void TextureInUse();
    void PendingNewTexture();

public slots:

    // This function gets called on the FBO rendering thread and will store the
    // texture id and size and schedule an update on the window.
    void NewTexture(int _id, const QSize &_size);

    // Before the scene graph starts to render, we update to the pending texture
    void PrepareNode();

private:

    int id;
    QSize size;

    QMutex mutex;

    QSGTexture *texture = nullptr;
    QQuickWindow *window = nullptr;
};

//--------------------------------------------------------------------------
class ThreadRenderer : public QQuickItem
{
    Q_OBJECT
    // QML_NAMED_ELEMENT(Renderer)

public:
    ThreadRenderer();

    static QList<QThread *> threads;

public slots:
    void Ready();

protected:
    QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
    RenderThread *renderThread = nullptr;
};

#endif // IGNITION_RENDERING_EXAMPLES_SIMPLE_DEMO_THREAD_RENDERER_HH_
