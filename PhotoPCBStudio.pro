#-------------------------------------------------
#
# Project created by QtCreator 2026-02-10T12:11:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = PhotoPCBStudio
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        src/main.cpp \
        src/mainwindow.cpp \
        src/imageprocessor.cpp \
        src/edgesharpener.cpp \
        src/gaussian_blur.cpp \
        src/dp_simplify.cpp \
        src/ledlayoutengine.cpp \
        src/layergenerator.cpp

HEADERS += \
        src/mainwindow.h \
        src/imageprocessor.h \
        src/edgesharpener.h \
        src/gaussian_blur.h \
        src/dp_simplify.h \
        src/ledlayoutengine.h \
        src/layergenerator.h \
        src/ledstrip.h

FORMS += \
        src/mainwindow.ui
