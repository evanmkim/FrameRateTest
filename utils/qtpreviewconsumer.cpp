/*
 * Copyright (c) 2016, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "qtpreviewconsumer.h"
#include "Error.h"

#include <string.h>
#include <math.h>
#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>

//#include "EGLGlobal.h"
//#include "GLContext.h"
//#include "Window.h"
//#include "Thread.h"
//#include "Options.h"

//#include <Argus/Argus.h>
//#include <Argus/Ext/FaceDetect.h>
//#include <EGLStream/EGLStream.h>

//#include <unistd.h>
//#include <stdlib.h>




namespace ArgusSamples
{
#define PREVIEW_CONSUMER_PRINT(...) printf("PREVIEW CONSUMER: " __VA_ARGS__)

QtPreviewConsumerThread::QtPreviewConsumerThread(EGLDisplay display, EGLStreamKHR stream)
    : m_display(display)
    , m_streams(1, stream)
    , m_textures(1, 0)
    , m_program(0)
    , m_textureUniform(-1)
    , m_layout(LAYOUT_TILED)
    , m_syncStreams(true)
    , m_lineWidth(0)
{
    memset(m_lineColor, 0, sizeof(m_lineColor));


}

QtPreviewConsumerThread::QtPreviewConsumerThread(EGLDisplay display,
                                             const std::vector<EGLStreamKHR>& streams,
                                             RenderLayout layout,
                                             bool syncStreams)
    : m_display(display)
    , m_streams(streams)
    , m_textures(streams.size(), 0)
    , m_program(0)
    , m_textureUniform(-1)
    , m_layout(layout)
    , m_syncStreams(syncStreams)
    , m_lineWidth(0)
{
    memset(m_lineColor, 0, sizeof(m_lineColor));
}

QtPreviewConsumerThread::~QtPreviewConsumerThread()
{
}

bool QtPreviewConsumerThread::threadInitialize()
{




    return true;
}

bool QtPreviewConsumerThread::threadExecute()
{
    EGLint state = EGL_STREAM_STATE_CONNECTING_KHR;

    // Wait until the Argus producers are connected.
    PREVIEW_CONSUMER_PRINT("Waiting until producer(s) connect...\n");
    for (std::vector<EGLStreamKHR>::iterator s = m_streams.begin(); s != m_streams.end(); s++)
    {
        while (true)
        {
            if (!eglQueryStreamKHR(m_display, *s, EGL_STREAM_STATE_KHR, &state))
                ORIGINATE_ERROR("Failed to query stream state (possible producer failure).");
            if (state == EGL_STREAM_STATE_NEW_FRAME_AVAILABLE_KHR)
                break;
            usleep(1000);
        }
    }
    PREVIEW_CONSUMER_PRINT("Producer(s) connected; continuing.\n");

    // Render as long as every stream is connected and the thread is still active.
    uint32_t frame = 0;
    bool done = false;
    while (!done && !m_doShutdown)
    {


        bool newFrameAvailable = false;
        for (uint32_t i = 0; i < m_streams.size(); i++)
        {
            if (!eglQueryStreamKHR(m_display, m_streams[i], EGL_STREAM_STATE_KHR, &state) ||
                state == EGL_STREAM_STATE_DISCONNECTED_KHR)
            {
                done = true;
                break;
            }
            else if (state == EGL_STREAM_STATE_NEW_FRAME_AVAILABLE_KHR)
            {
                newFrameAvailable = true;
                if (!eglStreamConsumerAcquireKHR(m_display, m_streams[i]))
                {
                    done = true;
                    break;
                }
            }

            if (!done)
            {
                if (newFrameAvailable)
                {


                    // Frame numbers will be different for each stream when they aren't synced,
                    // so for now we don't bother printing the "acquired frame" message.
                    /// @todo: Ideally we'd have an option to render a frame id text overlay on
                    ///        top of each stream window.
                    if (m_syncStreams)
                        PREVIEW_CONSUMER_PRINT("Acquired frame %d. Rendering.\n", ++frame);


                    UniqueObj<EGLStream::MetadataContainer> metadataContainer(EGLStream::MetadataContainer::create(m_display, m_streams[i]));
                    EGLStream::IArgusCaptureMetadata *iArgusCaptureMetadata =interface_cast<EGLStream::IArgusCaptureMetadata>(metadataContainer);

                    if (iArgusCaptureMetadata)
                               CaptureMetadata *argusCaptureMetadata = iArgusCaptureMetadata->getMetadata();


                    renderStreams();
                    PROPAGATE_ERROR(m_context.swapBuffers());
                }
                else
                {
                    usleep(1000);
                }
            }


        }

    }
    PREVIEW_CONSUMER_PRINT("No more frames. Cleaning up.\n");

    PROPAGATE_ERROR(requestShutdown());

    return true;
}

void QtPreviewConsumerThread::renderStreams()
{



    glClearColor(m_lineColor[0], m_lineColor[1], m_lineColor[2], 1.0f);

    if (m_streams.size() == 1)
    {
        glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    }
    else
    {
        if (m_layout == LAYOUT_HORIZONTAL)
        {
            int32_t streamWidth = m_windowSize.width() / m_streams.size();
            for (uint32_t i = 0; i < m_streams.size(); i++)
            {
                glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_textures[i]);
                glViewport(i * streamWidth, 0, streamWidth, m_windowSize.height());
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                if (m_lineWidth && i > 0)
                {
                    glEnable(GL_SCISSOR_TEST);
                    glScissor(i * streamWidth - m_lineWidth / 2, 0,
                              m_lineWidth, m_windowSize.height());
                    glClear(GL_COLOR_BUFFER_BIT);
                    glDisable(GL_SCISSOR_TEST);
                }
            }
        }
        else if (m_layout == LAYOUT_VERTICAL)
        {
            int32_t streamHeight = m_windowSize.height() / m_streams.size();
            for (uint32_t i = 0; i < m_streams.size(); i++)
            {
                glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_textures[i]);
                glViewport(0, m_windowSize.height() - streamHeight * (i+1),
                           m_windowSize.width(), streamHeight);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                if (m_lineWidth && i > 0)
                {
                    glEnable(GL_SCISSOR_TEST);
                    glScissor(0, m_windowSize.height() - streamHeight * i - m_lineWidth / 2,
                              m_windowSize.width(), m_lineWidth);
                    glClear(GL_COLOR_BUFFER_BIT);
                    glDisable(GL_SCISSOR_TEST);
                }
            }
        }
        else if (m_layout == LAYOUT_TILED)
        {
            uint32_t tileCount = ceil(sqrt(m_streams.size()));
            int32_t tileWidth = m_windowSize.width() / tileCount;
            int32_t tileHeight = m_windowSize.height() / tileCount;
            for (uint32_t i = 0; i < m_streams.size(); i++)
            {
                uint32_t row = (tileCount - 1) - (i / tileCount);
                uint32_t col = i % tileCount;
                glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_textures[i]);
                glViewport(col * tileWidth, row * tileHeight, tileWidth, tileHeight);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
            }
            if (m_lineWidth)
            {
                glEnable(GL_SCISSOR_TEST);
                for (uint32_t i = 1; i < tileCount; i++)
                {
                    glScissor(0, m_windowSize.height() - tileHeight * i - m_lineWidth / 2,
                              m_windowSize.width(), m_lineWidth);
                    glClear(GL_COLOR_BUFFER_BIT);
                    glScissor(tileWidth * i - m_lineWidth / 2, 0,
                              m_lineWidth, m_windowSize.height());
                    glClear(GL_COLOR_BUFFER_BIT);
                }
                glDisable(GL_SCISSOR_TEST);
            }
        }
        else if (m_layout == LAYOUT_SPLIT_HORIZONTAL)
        {
            int32_t streamHeight = m_windowSize.height() / m_streams.size();
            glEnable(GL_SCISSOR_TEST);
            for (uint32_t i = 0; i < m_streams.size(); i++)
            {
                glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_textures[i]);
                glScissor(0, m_windowSize.height() - streamHeight * (i+1),
                          m_windowSize.width(), streamHeight);
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                if (m_lineWidth && i > 0)
                {
                    glScissor(0, m_windowSize.height() - streamHeight * i - m_lineWidth / 2,
                              m_windowSize.width(), m_lineWidth);
                    glClear(GL_COLOR_BUFFER_BIT);
                }
            }
            glDisable(GL_SCISSOR_TEST);
        }
        else if (m_layout == LAYOUT_SPLIT_VERTICAL)
        {
            int32_t streamWidth = m_windowSize.width() / m_streams.size();
            glEnable(GL_SCISSOR_TEST);
            for (uint32_t i = 0; i < m_streams.size(); i++)
            {
                glBindTexture(GL_TEXTURE_EXTERNAL_OES, m_textures[i]);
                glScissor(streamWidth * i, 0, streamWidth, m_windowSize.height());
                glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
                if (m_lineWidth && i > 0)
                {
                    glScissor(streamWidth * i - m_lineWidth / 2, 0,
                              m_lineWidth, m_windowSize.height());
                    glClear(GL_COLOR_BUFFER_BIT);
                }
            }
            glDisable(GL_SCISSOR_TEST);
        }
    }

    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

bool QtPreviewConsumerThread::threadShutdown()
{
    glDeleteProgram(m_program);
    glDeleteTextures(m_textures.size(), &m_textures[0]);
    m_context.cleanup();

    PREVIEW_CONSUMER_PRINT("Done.\n");

    return true;
}

void QtPreviewConsumerThread::setLineWidth(uint32_t width)
{
    m_lineWidth = width;
}

void QtPreviewConsumerThread::setLineColor(float r, float g, float b)
{
    m_lineColor[0] = r;
    m_lineColor[1] = g;
    m_lineColor[2] = b;
}

} // namespace ArgusSamples

