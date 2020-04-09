TEMPLATE = app
CONFIG  += console
CONFIG  -= app_bundle

QT       += core network



QMAKE_CXXFLAGS += -I/usr/include/xenomai/cobalt
QMAKE_CXXFLAGS += -I/usr/include/xenomai
QMAKE_CXXFLAGS += -D_GNU_SOURCE -D_REENTRANT -D__COBALT__
#QMAKE_CXXFLAGS += -std=c++11 -pthread

QMAKE_LFLAGS += /usr/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=/usr/lib/dynlist.ld
QMAKE_LFLAGS += -L/usr/lib -lcobalt -lpthread -lrt

INCLUDEPATH += \
    /usr/include/xenomai/cobalt \
    /usr/include/xenomai \
    /usr/include/xenomai/alchemy \
    ../../../share/Headers \
    /usr/include/eigen3 \
    /home/rainbow/fcl/include/fcl \
    /home/rainbow/fcl/test

LIBS    += \
    -lalchemy \
    -lcopperplate \
    -lcobalt \
    -lpthread \
    -lrt \
    -lpcan \
    -L/usr/local/lib/ -lrbdl \
#    /home/rainbow/fcl/build/lib/libfcl.so \
#    /home/rainbow/fcl/build/lib/libfcl.so.0.6 \
    /home/rainbow/fcl/build/lib/libfcl.so.0.6.0 \
    /home/rainbow/fcl/build/lib/libgtest.a \
    /home/rainbow/fcl/build/lib/libgtest_main.a \
    /home/rainbow/fcl/build/lib/libtest_fcl_utility.a \
    -L"/home/rainbow/fcl/build/lib" \
    -L"/usr/local/lib" \
     /usr/local/lib/libccd.so.2 \
    -pthread \
    -lccd

SOURCES += \
    main.cpp \
    Array.cc \
    QuadProg++.cc


HEADERS += \
    BasicFiles/BasicSetting.h \
    BasicFiles/BasicJoint.h \
    QuadProg++.hh \
    Array.hh \
    js_qp.h


