QT += core gui widgets

CONFIG += c++11

TARGET = 3DofRobot

SOURCES += \
    main.cpp \
    MainWindow/mainwindow.cpp \
    RobotArm/robotarm.cpp \
    RobotArm/numerical.cpp \
    Logging/logger.cpp \
    Settings/customsettings.cpp \
    Dynamixel/dynamixel.cpp \

HEADERS += \
    MainWindow/mainwindow.h \
    RobotArm/robotarm.h \
    RobotArm/numerical.h \
    Logging/logger.h \
    Settings/customsettings.h \
    Dynamixel/dynamixel.h \

FORMS += \
    mainwindow.ui


LIBS += -L$$PWD/Dynamixel/ -ldxl_x32_cpp

INCLUDEPATH += $$PWD/Dynamixel/dynamixel_sdk
DEPENDPATH += $$PWD/Dynamixel/dynamixel_sdk
