QT += core gui widgets sql serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = force_control_assistant_tool
TEMPLATE = app
#CONFIG += c++11
#DEFINES += _GLIBCXX_USE_CXX11_ABI=0
#QMAKE_CXXFLAGS += -std=c++11

DESTDIR += ./bin

include(./peripherals/peripherals.pri)
include(./utility/utility.pri)
INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/peripherals/include/
INCLUDEPATH += $$PWD/utility/include/
INCLUDEPATH += $$PWD/../aral/aral_export/include/
INCLUDEPATH += /usr/arcs/include


HEADERS  += $$PWD/include/mainwindow.h \
            $$PWD/include/robot_control.h \


SOURCES += $$PWD/src/main.cpp \
           $$PWD/src/mainwindow.cpp \
           $$PWD/src/robot_control.cpp \
           $$PWD/src/dynamics_check.cpp \

FORMS += mainwindow.ui

#********linking library **********************************
LIBS += -L$$PWD/../aral/aral_export/lib/  -laral_d
LIBS += -L/usr/arcs/lib -laubo_driver
LIBS += -lpthread



