QT       += core gui vsoa

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++11

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    DWplan/DWPlanner.cpp \
    DWplan/dw.cpp \
    PathPlan/TargetManager.cpp \
    PathPlan/UAVStrategyManager.cpp \
    PathPlan/pathplanner.cpp \
    UAVStrategy/Strategy1.cpp \
    UAVStrategy/Strategy2.cpp \
    UAVStrategy/Strategy3.cpp \
    UAVStrategy/Strategy4.cpp \
    gridmap.cpp \
    maddpg_agent.cpp \
    main.cpp \
    mainwindow.cpp \
    vsoamanager.cpp

HEADERS += \
    DWplan/DWPlanner.h \
    DWplan/dw.h \
    PathPlan/TargetManager.h \
    PathPlan/UAVStrategyManager.h \
    PathPlan/pathplanner.h \
    UAVStrategy/Strategy1.h \
    UAVStrategy/Strategy2.h \
    UAVStrategy/Strategy3.h \
    UAVStrategy/Strategy4.h \
    gridmap.h \
    maddpg_agent.h \
    mainwindow.h \
    vsoamanager.h

FORMS += \
    mainwindow.ui

# To use Qwt in your project, enable the USE_QWT=1.
#USE_QWT = 1
!isEmpty(USE_QWT) {
    LIBS += -L$$[QT_INSTALL_PREFIX]\lib -lqwt
    INCLUDEPATH += $$[QT_INSTALL_PREFIX]\include\Qwt
}

# Default rules for deployment.
sylixos: target.path = /apps/$${TARGET}
else: qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    agentmodel.qrc

DISTFILES +=
