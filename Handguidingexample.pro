QT       += core gui sql
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Handguidingexample
TEMPLATE = app
CONFIG += c++11
DEFINES += _GLIBCXX_USE_CXX11_ABI=0

#********linking library **********************************
unix{
    #32bit system
    contains(QT_ARCH, i386){
        LIBS += -L$$PWD/lib/lib32/ -lauborobotcontroller
#        LIBS += -L$$PWD/lib/lib32/libauborobotcontroller.a
        LIBS += $$PWD/lib/lib32/libotgLib.a
    }
    #64bit system
    contains(QT_ARCH, x86_64){
        LIBS += -L$$PWD/lib/lib64/ -lauborobotcontroller
        LIBS += $$PWD/lib/lib64/libotgLib.a
    }
}
LIBS += -lmemcached -lpthread

include(./ftsensor/ftsensor.pri)
include(./utility/utility.pri)
INCLUDEPATH += $$PWD/include


HEADERS  += $$PWD/include/mainwindow.h \
            $$PWD/include/FTSensorDataProcess.h \
            include/robotcontrol.h \

SOURCES += $$PWD/src/main.cpp \
           $$PWD/src/mainwindow.cpp \
           $$PWD/src/FTSensorDataProcess.cpp \
            src/robotcontrol.cpp

FORMS += mainwindow.ui


