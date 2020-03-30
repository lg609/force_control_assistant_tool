INCLUDEPATH += $$PWD/include

INCLUDEPATH += /home/lg/Projects/AUBO_ROBOTICS/aubo_robotics/RobotLibraryInterface/include/


HEADERS += \
    $$PWD/include/rq_sensor_state.h \
    $$PWD/include/rq_sensor_socket.h \
    $$PWD/include/rq_thread.h \
    $$PWD/include/rq_sensor_com.h \
    $$PWD/include/rq_int.h \
    $$PWD/include/mutex.h   \
    $$PWD/include/database.h \

SOURCES += \
    $$PWD/src/rq_sensor_state.c \
    $$PWD/src/rq_sensor_socket.c \
    $$PWD/src/rq_thread.c \
    $$PWD/src/rq_sensor_com.c \
    $$PWD/src/mutex.c \
    $$PWD/src/database.cpp \
