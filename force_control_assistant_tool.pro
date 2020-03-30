QT += core gui widgets sql serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = force_control_assistant_tool
TEMPLATE = app
CONFIG += c++11
DEFINES += _GLIBCXX_USE_CXX11_ABI=0
QMAKE_CXXFLAGS += -std=c++11

DESTDIR += ./bin

#********linking library **********************************
unix{
    #32bit system
    contains(QT_ARCH, i386){

    }
    #64bit system
    contains(QT_ARCH, x86_64){
#        LIBS += -L$$PWD/lib/lib64/ -lauborobotcontroller
        LIBS += /home/lg/Projects/AUBO_ROBOTICS/lib/lib64/libaubo_robotics.a
    }
}
LIBS += -lpthread

include(./peripherals/peripherals.pri)
include(./utility/utility.pri)
INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/peripherals/include/
INCLUDEPATH += $$PWD/utility/include/

INCLUDEPATH += /home/lg/Projects/AUBO_ROBOTICS/aubo_robotics/interface/include/


HEADERS  += $$PWD/include/mainwindow.h \
            $$PWD/include/robot_control.h \

SOURCES += $$PWD/src/main.cpp \
           $$PWD/src/mainwindow.cpp \
           $$PWD/src/robot_control.cpp

FORMS += mainwindow.ui


