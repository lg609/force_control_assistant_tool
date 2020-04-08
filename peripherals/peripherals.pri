INCLUDEPATH += $$PWD/include

HEADERS += \
    $$PWD/include/ft_sensor.h \
    $$PWD/include/kunwei_sensor.h \
#    $$PWD/include/optoforce_sensor.h \
#    $$PWD/include/robotiq_sensor.h \
#    $$PWD/include/ati_sensor.h \
#    $$PWD/include/serial_port.h \

SOURCES += \
    $$PWD/src/ft_sensor.cpp \
    $$PWD/src/kunwei_sensor.cpp \
    $$PWD/src/sensor_data_process.cpp \
    $$PWD/src/serial_port.cpp \
#    $$PWD/src/optoforce_sensor.cpp \
#    $$PWD/src/robotiq_sensor.cpp \
#    $$PWD/src/ati_sensor.cpp \

