QT += core gui widgets sql serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = force_control_assistant_tool
TEMPLATE = app
CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11

DESTDIR = $$PWD/build

INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/toml/include
INCLUDEPATH += $$PWD/../aral/aral_export/include/
INCLUDEPATH += /usr/arcs/include

HEADERS  += $$PWD/include/mainwindow.h \
            $$PWD/include/robot_control.h \

SOURCES += $$PWD/src/main.cpp \
           $$PWD/src/mainwindow.cpp \
           $$PWD/src/robot_control.cpp \

FORMS += mainwindow.ui

#********linking library **********************************
LIBS += -L$$PWD/../aral/aral_export/lib/  -laral_d
LIBS += -L/usr/arcs/lib -laubo_driver
LIBS += -lpthread

#********INSTALL **********************************
config.path = $$PWD/build
config.files += $$PWD/config.toml
config.files += $$PWD/aubo_i5.urdf
INSTALLS += config




