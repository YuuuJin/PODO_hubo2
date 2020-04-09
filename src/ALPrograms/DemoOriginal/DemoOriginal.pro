TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle



QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt


INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy \
    ../../../share/Headers

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L../../../share/Libs       -lKINE_DRC_HUBO4 \
    -L../../../share/Libs       -lKINE_DRC_HUBO2 \
    -L../../../share/Libs	-lik_math4 \


SOURCES += \
    main.cpp \
    BasicFiles/TaskMotion.cpp \
    BasicFiles/BasicTrajectory.cpp \
    ManualCAN.cpp

#    BasicFiles/taskpos.cpp \
#    BasicFiles/taskori.cpp \
#    BasicFiles/taskmotion.cpp \


HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    BasicFiles/TaskGeneral.h \
    BasicFiles/TaskMotion.h \
    ManualCAN.h



#    BasicFiles/taskGeneral.h \
#    BasicFiles/taskpos.h \
#    BasicFiles/taskori.h \
#    BasicFiles/taskmotion.h \


