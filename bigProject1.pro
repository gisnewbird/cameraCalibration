#-------------------------------------------------
#
# Project created by QtCreator 2018-12-04T19:50:49
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
#QMAKE_CXXFLAGS += -fopenmp
#LIBS += -lgomp -lpthread
TARGET = bigProject1
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        mainwindow.cpp \
    cameracalibration.cpp

HEADERS += \
        mainwindow.h \
    cameracalibration.h

FORMS += \
        mainwindow.ui
#INCLUDEPATH += /usr/local/include \
#                /usr/local/include/opencv \
#                /usr/local/include/opencv2
INCLUDEPATH += D:\\sourceLibrary\\myopencv\\install\\include
INCLUDEPATH += D:\\sourceLibrary\\myopencv\\install\\include\\opencv
INCLUDEPATH += D:\\sourceLibrary\\myopencv\\install\\include\\opencv2
LIBS += D:\sourceLibrary\myopencv\install\x86\vc14\lib\opencv_*.lib
#LIBS += /usr/local/lib/libopencv_*.so

#INCLUDEPATH += D:\sourceLibrary\openMVG\include
#INCLUDEPATH += D:\sourceLibrary\openMVG\include\openMVG
#INCLUDEPATH += D:\sourceLibrary\openMVG\include\openMVG_dependencies
#LIBS += D:\sourceLibrary\openMVG\lib\openMVG_*.lib
#LIBS += D:\sourceLibrary\openMVG\lib\openMVG_features.lib
INCLUDEPATH += D:\sourceLibrary\myEigen\include\eigen3
INCLUDEPATH += D:\sourceLibrary\myVTK8.1\include\vtk-8.1
LIBS += D:\sourceLibrary\myVTK8.1\lib\vtk*.lib
INCLUDEPATH += D:\sourceLibrary\myPCL\3rdParty\Boost\include\boost-1_64
#LIBS += D:\sourceLibrary\myPCL\3rdParty\Boost\lib\libboost_*.lib
#LIBS += D:\sourceLibrary\myPCL\3rdParty\Boost\lib\libboost_thread-vc140-mt-gd-1_64.lib
#LIBS += D:\sourceLibrary\myPCL\3rdParty\Boost\lib\libboost_date_time-vc140-mt-gd-1_64.lib
#LIBS += D:\sourceLibrary\myPCL\3rdParty\Boost\lib\libboost_system-vc140-mt-gd-1_64.lib
#LIBS += D:\sourceLibrary\myPCL\3rdParty\Boost\lib\libboost_chrono-vc140-mt-gd-1_64.lib

RESOURCES += \
    icon.qrc

#QMAKE_CFLAGS_RELEASE = MT
#QMAKE_CXXFLAGS_RELEASE = MT
msvc:QMAKE_CXXFLAGS += -source-charset:utf-8
