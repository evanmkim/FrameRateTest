#-------------------------------------------------
#
# Project created by QtCreator 2019-06-20T12:09:05
#
#-------------------------------------------------

QT       += core gui
QT       += multimedia multimediawidgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ExposureUI
TEMPLATE = app

CONFIG += c++11

CONFIG += console

INCLUDEPATH += /usr/local/include \
               /usr/local/include/opencv \
               /usr/local/include/opencv2


LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_videoio.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_imgcodecs.so \
        /usr/lib/aarch64-linux-gnu/tegra/libargus.so \
        /usr/lib/aarch64-linux-gnu/libQt5Multimedia.so.5.5 \
        /usr/lib/aarch64-linux-gnu/libv4l2.so \
        /usr/lib/aarch64-linux-gnu/libX11.so \
        /usr/lib/aarch64-linux-gnu/libX11.so\
        /usr/lib/aarch64-linux-gnu/libXext.so \
        /usr/lib/aarch64-linux-gnu/libjpeg.so \
        /usr/lib/aarch64-linux-gnu/tegra-egl/libEGL.so \
        /usr/lib/aarch64-linux-gnu/tegra-egl/libGLESv2.so.2 \
        /usr/lib/aarch64-linux-gnu/tegra-egl/libEGL.so \
        /usr/lib/aarch64-linux-gnu/libexpat.so \
        /usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so \
        /usr/lib/aarch64-linux-gnu/tegra/libnvbuf_utils.so.1.0.0


LIBS += /usr/local/lib/libopencv_highgui.so \
        /usr/local/lib/libopencv_core.so    \
        /usr/local/lib/libopencv_videoio.so \
        /usr/local/lib/libopencv_features2d.so \
        /usr/local/lib/libopencv_imgproc.so \
        /usr/local/lib/libopencv_imgcodecs.so.3.4

SOURCES += main.cpp\
        mainwindow.cpp \
        utils/glx/glxWindow.cpp \
        utils/gtk/gtkWindow.cpp \
        utils/EGLGlobal.cpp \
        utils/GLContext.cpp \
        utils/Observed.cpp \
        utils/Options.cpp \
        utils/PreviewConsumer.cpp \
        utils/Thread.cpp \
        utils/WindowBase.cpp \
    utils/qtpreviewconsumer.cpp \
    asmOpenCV.cpp \
    maincamera.cpp \
    circular_buffer.cpp

HEADERS  += mainwindow.h \
            Argus/Ext/BayerAverageMap.h \
            Argus/Ext/BayerSharpnessMap.h \
            Argus/Ext/DebugCaptureSession.h \
            Argus/Ext/DeFog.h \
            Argus/Ext/FaceDetect.h \
            Argus/Ext/InternalFrameCount.h \
            Argus/Ext/PwlWdrSensorMode.h \
            Argus/Ext/SensorPrivateMetadata.h \
            Argus/Argus.h \
            Argus/CameraDevice.h \
            Argus/CameraProvider.h \
            Argus/CaptureMetadata.h \
            Argus/CaptureSession.h \
            Argus/Event.h \
            Argus/EventProvider.h \
            Argus/EventQueue.h \
            Argus/Request.h \
            Argus/Settings.h \
            Argus/Stream.h \
            Argus/Types.h \
            Argus/UUID.h \
            EGLStream/NV/ImageNativeBuffer.h \
            EGLStream/ArgusCaptureMetadata.h \
            EGLStream/EGLStream.h \
            EGLStream/Frame.h \
            EGLStream/FrameConsumer.h \
            EGLStream/Image.h \
            EGLStream/MetadataContainer.h \
            utils/glx/glxWindow.h \
            utils/gtk/gtkWindow.h \
            utils/Courier16x24.h \
            utils/EGLGlobal.h \
            utils/Error.h \
            utils/GLContext.h \
            utils/InitOnce.h \
            utils/IObserver.h \
            utils/Observed.h \
            utils/Options.h \
            utils/Ordered.h \
            utils/PreviewConsumer.h \
            utils/Thread.h \
            utils/UniquePointer.h \
            utils/Validator.h \
            utils/Value.h \
            utils/Window.h \
            utils/WindowBase.h \
    nvbuf_utils.h \
    cvmatconsumerthread.h \
    utils/qtpreviewconsumer.h \
    mattoqimage.h \
    asmOpenCV.h \
    utils/gpiotriggermode.h \
    maincamera.h \
    circular_buffer.h

FORMS    += mainwindow.ui
