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

#include "ThreadRenderer.h"
#include "IgnitionRenderer.hh"

#include <QGuiApplication>
#include <QMutex>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QQuickWindow>
#include <QThread>

QList<QThread *> ThreadRenderer::threads;

//-----------------------------------------------------------------------
RenderThread::RenderThread(const QSize &_size, QQuickItem *_renderWindowItem)
    : size(_size), renderWindowItem(_renderWindowItem)
{
    ThreadRenderer::threads << this;
}

//-----------------------------------------------------------------------
void RenderThread::Print(const QSurfaceFormat &_format)
{
    auto formatOptionsToString = [] (QSurfaceFormat::FormatOptions _value) -> std::string
    {
        std::string options;

        if (_value & QSurfaceFormat::StereoBuffers)
        {
            options.append("StereoBuffers");
        }
        if (_value & QSurfaceFormat::DebugContext)
        {
            options.empty() ? options.append("") : options.append(", ");
            options.append("DebugContext");
        }
        if (_value & QSurfaceFormat::DeprecatedFunctions)
        {
            options.empty() ? options.append("") : options.append(", ");
            options.append("DeprecatedFunctions");
        }
        if (_value & QSurfaceFormat::ResetNotification)
        {
            options.empty() ? options.append("") : options.append(", ");
            options.append("ResetNotification");
        }

        return options;
    };

    auto openGLContextProfileToString = [] (QSurfaceFormat::OpenGLContextProfile _value) -> std::string
    {
        switch (_value)
        {
        case QSurfaceFormat::NoProfile:
            return "NoProfile";
        case QSurfaceFormat::CoreProfile:
            return "CoreProfile";
        case QSurfaceFormat::CompatibilityProfile:
            return "CompatibilityProfile";
        default:
            return "Invalid OpenGLContextProfile";
        } 
    };

    auto renderableTypeToString = [] (QSurfaceFormat::RenderableType _value) -> std::string
    {
        switch (_value)
        {
        case QSurfaceFormat::DefaultRenderableType:
            return "DefaultRenderableType";
        case QSurfaceFormat::OpenGL:
            return "OpenGL";
        case QSurfaceFormat::OpenGLES:
            return "OpenGLES";
        case QSurfaceFormat::OpenVG:
            return "OpenVG";
        default:
            return "Invalid RenderableType";
        } 
    };

    auto swapBehaviorToString = [] (QSurfaceFormat::SwapBehavior _value) -> std::string
    {
        switch (_value)
        {
        case QSurfaceFormat::DefaultSwapBehavior:
            return "DefaultSwapBehavior";
        case QSurfaceFormat::SingleBuffer:
            return "SingleBuffer";
        case QSurfaceFormat::DoubleBuffer:
            return "DoubleBuffer";
        default:
            return "Invalid SwapBehavior";
        } 
    };

    // surface format info
    ignmsg << "version: "
        << _format.version().first << "."
        << _format.version().second << "\n";
    ignmsg << "profile: "
        << openGLContextProfileToString(_format.profile()) << "\n";
    ignmsg << "options: "
        << formatOptionsToString(_format.options()) << "\n";
    ignmsg << "renderableType: "
        << renderableTypeToString(_format.renderableType()) << "\n";
    ignmsg << "hasAlpha: " << _format.hasAlpha() << "\n";
    ignmsg << "redBufferSize: " << _format.redBufferSize() << "\n";
    ignmsg << "greenBufferSize: " << _format.greenBufferSize() << "\n";
    ignmsg << "blueBufferSize: " << _format.blueBufferSize() << "\n";
    ignmsg << "alphaBufferSize: " << _format.alphaBufferSize() << "\n";
    ignmsg << "depthBufferSize: " << _format.depthBufferSize() << "\n";
    ignmsg << "stencilBufferSize: " << _format.stencilBufferSize() << "\n";
    ignmsg << "samples: " << _format.samples() << "\n";
    ignmsg << "swapBehavior: "
        << swapBehaviorToString(_format.swapBehavior()) << "\n";
    ignmsg << "swapInterval: " << _format.swapInterval() << "\n";
    ignmsg << "\n";
}

//-----------------------------------------------------------------------
QSurfaceFormat RenderThread::CreateSurfaceFormat()
{
    // QSurfaceFormat format;
    QSurfaceFormat format(QSurfaceFormat::DeprecatedFunctions);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    format.setMajorVersion(4);
    format.setMinorVersion(1);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);

    return format;    
}

//--------------------------------------------------------------------------
// called when the render node emits textureInUse
void RenderThread::InitialiseOnMainThread()
{
    this->context->makeCurrent(this->surface);
    Print(this->context->format());

    // create renderer
    this->renderer = new IgnitionRenderer();
    this->renderer->InitialiseOnMainThread();

    this->context->doneCurrent();
}

//--------------------------------------------------------------------------
// called when the render node emits textureInUse
void RenderThread::RenderNext()
{
    this->context->makeCurrent(this->surface);

    if (!this->renderer->Initialised())
    {
        this->renderer->Initialise();
    }

    // check if engine has been successfully initialized
    if (!this->renderer->Initialised())
    {
        ignerr << "Unable to initialize renderer" << std::endl;
        return;
    }

    this->renderer->Render();

    emit this->TextureReady(this->renderer->TextureId(), this->renderer->TextureSize());

    this->context->doneCurrent();
}

//--------------------------------------------------------------------------
void RenderThread::ShutDown()
{
    this->context->makeCurrent(this->surface);

    delete this->renderer;
    this->renderer = nullptr;

    this->context->doneCurrent();

    delete this->context;
    this->context = nullptr;

    // schedule this to be deleted only after we're done cleaning up
    this->surface->deleteLater();

    // Stop event processing, move the thread to GUI and make sure it is deleted.
    exit();
    moveToThread(QGuiApplication::instance()->thread());
}

//--------------------------------------------------------------------------
//--------------------------------------------------------------------------
TextureNode::TextureNode(QQuickWindow *_window)
    : id(0)
    , size(0, 0)
    , texture(nullptr)
    , window(_window)
{
    // Our texture node must have a texture, so use the default 0 texture.
    // createTextureFromNativeObject()
    this->texture = this->window->createTextureFromId(0, QSize(1, 1));
    this->setTexture(this->texture);
    this->setFiltering(QSGTexture::Linear);
}

//--------------------------------------------------------------------------
TextureNode::~TextureNode()
{
    delete this->texture;
    this->texture = nullptr;
}

//--------------------------------------------------------------------------
// called when RenderThread emits signal textureReady
void TextureNode::NewTexture(int _id, const QSize &_size)
{
    this->mutex.lock();
    this->id = _id;
    this->size = _size;
    this->mutex.unlock();

    // We cannot call QQuickWindow::update directly here, as this is only allowed
    // from the rendering thread or GUI thread.
    emit this->PendingNewTexture();
}

//--------------------------------------------------------------------------
// called when the window emits beforeRendering
void TextureNode::PrepareNode()
{
    this->mutex.lock();
    int newId = this->id;
    QSize size = this->size;
    this->id = 0;
    this->mutex.unlock();
    if (newId)
    {
        delete this->texture;
        this->texture = nullptr;
        // note: include QQuickWindow::TextureHasAlphaChannel if the rendered content
        // has alpha.
        // createTextureFromNativeObject
        this->texture = this->window->createTextureFromId(newId, size);
        this->setTexture(this->texture);

        this->markDirty(DirtyMaterial);

        // This will notify the rendering thread that the texture is now being rendered
        // and it can start rendering to the other one.
        emit this->TextureInUse();
    }
}

//--------------------------------------------------------------------------
ThreadRenderer::ThreadRenderer()
    : renderThread(nullptr)
{
    setFlag(ItemHasContents, true);
    this->renderThread = new RenderThread(QSize(512, 512), this);
}

//--------------------------------------------------------------------------
void ThreadRenderer::Ready()
{
    // Run on the Main (GUI = QML) thread
    this->renderThread->surface = new QOffscreenSurface();
    this->renderThread->surface->setFormat(this->renderThread->context->format());
    this->renderThread->surface->create();

    // carry out any initialisation before moving to thread 
    this->renderThread->InitialiseOnMainThread();

    // Move to Render thread
    this->renderThread->context->moveToThread(this->renderThread);
    this->renderThread->moveToThread(this->renderThread);

    connect(window(), &QQuickWindow::sceneGraphInvalidated,
        this->renderThread, &RenderThread::ShutDown, Qt::QueuedConnection);

    // Running on Render thread
    this->renderThread->start();
    update();
}

//-----------------------------------------------------------------------
QSGNode *ThreadRenderer::updatePaintNode(QSGNode *_oldNode, UpdatePaintNodeData *)
{
    TextureNode *node = static_cast<TextureNode *>(_oldNode);

    if (!this->renderThread->context)
    {
        QOpenGLContext *current = window()->openglContext();
        // Some GL implementations requres that the currently bound context is
        // made non-current before we set up sharing, so we doneCurrent here
        // and makeCurrent down below while setting up our own context.
        current->doneCurrent();

        this->renderThread->context = new QOpenGLContext();

        // set the surface format (this is managed globally in Main.cpp)
        // auto surfaceFormat = RenderThread::createSurfaceFormat();
        // m_renderThread->context->setFormat(surfaceFormat);
        this->renderThread->context->setFormat(current->format());

        this->renderThread->context->setShareContext(current);
        this->renderThread->context->create();

        // QMetaObject::invokeMethod(this, "Ready");
        this->Ready();

        current->makeCurrent(window());

        return nullptr;
    }

    if (!node)
    {
        node = new TextureNode(window());

        /* Set up connections to get the production of FBO textures in sync with vsync on the
         * rendering thread.
         *
         * When a new texture is ready on the rendering thread, we use a direct connection to
         * the texture node to let it know a new texture can be used. The node will then
         * emit pendingNewTexture which we bind to QQuickWindow::update to schedule a redraw.
         *
         * When the scene graph starts rendering the next frame, the prepareNode() function
         * is used to update the node with the new texture. Once it completes, it emits
         * textureInUse() which we connect to the FBO rendering thread's renderNext() to have
         * it start producing content into its current "back buffer".
         *
         * This FBO rendering pipeline is throttled by vsync on the scene graph rendering thread.
         */
        connect(this->renderThread, &RenderThread::TextureReady,
            node, &TextureNode::NewTexture, Qt::DirectConnection);
        connect(node, &TextureNode::PendingNewTexture,
            window(), &QQuickWindow::update, Qt::QueuedConnection);
        connect(this->window(), &QQuickWindow::beforeRendering,
            node, &TextureNode::PrepareNode, Qt::DirectConnection);
        connect(node, &TextureNode::TextureInUse,
            this->renderThread, &RenderThread::RenderNext, Qt::QueuedConnection);

        // Get the production of FBO textures started..
        QMetaObject::invokeMethod(this->renderThread, "RenderNext", Qt::QueuedConnection);
    }

    node->setRect(boundingRect());

    return node;
}

//--------------------------------------------------------------------------
