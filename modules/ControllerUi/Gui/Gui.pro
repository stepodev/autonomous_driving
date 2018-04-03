#-------------------------------------------------
#
# Project created by QtCreator 2018-03-31T10:51:40
#
#-------------------------------------------------

QT       += core gui widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Gui
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
        main.cpp \
        gui.cpp

HEADERS += \
        gui.h

FORMS += \
        gui.ui

DISTFILES += \
    Gui.pri

#get UdpServer stuff
unix:!macx: LIBS += -L$$PWD/../../platooning_ws/devel/lib/ -ludpserver
unix:!macx: LIBS += -L$$PWD/../../platooning_ws/devel/lib/ -lmessagetypes

INCLUDEPATH += $$PWD/../../platooning_ws/devel/include
DEPENDPATH += $$PWD/../../platooning_ws/devel/include
INCLUDEPATH += $$PWD/../../platooning_ws/devel/include
DEPENDPATH += $$PWD/../../platooning_ws/devel/include

INCLUDEPATH += $$PWD/../../platooning_ws/src/platooning/include
DEPENDPATH += $$PWD/../../platooning_ws/src/platooning/include
INCLUDEPATH += $$PWD/../../platooning_ws/src/platooning/include
DEPENDPATH += $$PWD/../../platooning_ws/src/platooning/include

INCLUDEPATH += /opt/ros/lunar/include
DEPENDPATH +=/opt/ros/lunar/include

#BOOST
INCLUDEPATH += /usr/include/boost

unix:!macx: LIBS += -L/usr/include/boost -lboost_system
unix:!macx: LIBS += -L/usr/include/boost  -lboost_chrono
unix:!macx: LIBS += -L/usr/include/boost  -lboost_thread


