QT       += core gui widgets sql serialport

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
#        LIBS += $$PWD/lib/lib64/libotgLib.a
    }
}
LIBS += -lpthread

include(./peripherals/peripherals.pri)
include(./utility/utility.pri)
INCLUDEPATH += $$PWD/include
INCLUDEPATH += $$PWD/peripherals/include/


HEADERS  += $$PWD/include/mainwindow.h \
            $$PWD/include/FTSensorDataProcess.h \
            include/robotcontrol.h \

SOURCES += $$PWD/src/main.cpp \
           $$PWD/src/mainwindow.cpp \
           $$PWD/src/FTSensorDataProcess.cpp \
            src/robotcontrol.cpp

FORMS += mainwindow.ui


