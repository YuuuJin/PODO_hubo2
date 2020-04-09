#-------------------------------------------------
#
# Project created by QtCreator 2016-02-17T10:05:39
#
#-------------------------------------------------

QT       += core gui sql network opengl multimedia serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG(debug, debug|release) {
    DESTDIR = ../../exe
} else {
    DESTDIR = ../../exe
}


TARGET = PODOGUI
TEMPLATE = app


###
INCLUDEPATH += \
    ../../share/Headers/RBModel

LIBS += \
    -L../../share/Libs/ -lRBModel
###


SOURCES += \
    main.cpp\
    GUIMainWindow.cpp \
    LAN/RBTCPClient.cpp \
    LAN/RBTCPServer.cpp \
    BasicFiles/RBDataBase.cpp \
    BasicFiles/LANDialog.cpp \
    BasicFiles/PODOALDialog.cpp \
    BasicFiles/JointDialog.cpp \
    BasicFiles/SensorDialog.cpp \
    BasicFiles/ModelDialog.cpp \
    BasicFiles/SettingDialog.cpp \
    TutorialDialog.cpp \
    walkingdialog.cpp \
    joystickdialog.cpp \
    JoyStick/RBJoystick.cpp \
    OriginalDemoDialog.cpp \
    pyeongchangdialog.cpp \
    mpcwalkingdialog.cpp

HEADERS  += \
    GUIMainWindow.h \
    CommonHeader.h \
    LAN/RBLANCommon.h \
    LAN/RBLog.h \
    LAN/RBTCPClient.h \
    LAN/RBTCPServer.h \
    BasicFiles/RBDataBase.h \
    BasicFiles/RBDataType.h \
    BasicFiles/RBLog.h \
    BasicFiles/LANDialog.h \
    BasicFiles/PODOALDialog.h \
    BasicFiles/JointDialog.h \
    BasicFiles/SensorDialog.h \
    BasicFiles/ModelDialog.h \
    BasicFiles/SettingDialog.h \
    TutorialDialog.h \
    walkingdialog.h \
    joystickdialog.h \
    JoyStick/joystickvariable.h \
    JoyStick/joystickclass.h \
    OriginalDemoDialog.h \
    pyeongchangdialog.h \
    mpcwalkingdialog.h


FORMS    += \
    GUIMainWindow.ui \
    BasicFiles/LANDialog.ui \
    BasicFiles/PODOALDialog.ui \
    BasicFiles/JointDialog.ui \
    BasicFiles/SensorDialog.ui \
    BasicFiles/ModelDialog.ui \
    BasicFiles/SettingDialog.ui \
    TutorialDialog.ui \
    walkingdialog.ui \
    joystickdialog.ui \
    OriginalDemoDialog.ui \
    pyeongchangdialog.ui \
    mpcwalkingdialog.ui
