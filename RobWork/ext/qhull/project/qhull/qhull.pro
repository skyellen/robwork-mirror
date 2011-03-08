# -------------------------------------------------
# qhull.pro -- Qt project file for qhull.exe
# -------------------------------------------------

TARGET = qhull
DESTDIR = ../..
TEMPLATE = app
CONFIG += console warn_on
CONFIG -= app_bundle
LIBS += -L../..
QMAKE_CFLAGS_WARN_ON += -Werror -Wall -Wcast-qual -Wextra -Wshadow -Wwrite-strings
QMAKE_CFLAGS_WARN_ON += -Wno-sign-conversion # Many size_t vs. int warnings
#QMAKE_CFLAGS_WARN_ON += -Wconversion # No workaround for bit-field conversions
build_pass:CONFIG(debug, debug|release):{
   LIBS += libqhulld
   PRE_TARGETDEPS += ../../libqhulld.a
   OBJECTS_DIR = ../../tmp/qhull/Debug
}else:build_pass:CONFIG(release, debug|release):{
   LIBS += libqhull
   PRE_TARGETDEPS += ../../libqhull.a
   OBJECTS_DIR = ../../tmp/qhull/Release
}
QT -= gui
MOC_DIR = ../../tmp/moc
RCC_DIR = ../../tmp/rcc
INCLUDEPATH = ../../cpp;../../cpp/road;../../tmp
VPATH = ../..
SOURCES += src/unix.c
HEADERS += src/libqhull.h
