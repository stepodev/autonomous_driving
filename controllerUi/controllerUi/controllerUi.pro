#-------------------------------------------------
#
# Project created by QtCreator 2018-01-28T14:32:17
#
#-------------------------------------------------

QT       += core gui widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = controllerUi
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
        controllerui.cpp

HEADERS += \
    controllerui.hpp

FORMS += \
        controllerui.ui

#get UdpServer stuff
unix:!macx: LIBS += -L$$PWD/../../catkin_ws/devel/lib/ -ludpserver

INCLUDEPATH += $$PWD/../../catkin_ws/src/platooning/include
DEPENDPATH += $$PWD/../../catkin_ws/src/platooning/include
INCLUDEPATH += $$PWD/../../catkin_ws/devel/include/platooning
DEPENDPATH += $$PWD/../../catkin_ws/devel/include/platooning


#BOOST shit
INCLUDEPATH += /usr/include/boost

unix:!macx: LIBS += -L/usr/include/boost -lboost_system
unix:!macx: LIBS += -L/usr/include/boost  -lboost_chrono
unix:!macx: LIBS += -L/usr/include/boost  -lboost_thread
