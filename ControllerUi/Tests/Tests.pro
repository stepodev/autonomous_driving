QT += testlib
QT += gui
QT += core
QT += widgets

CONFIG += qt warn_on depend_includepath testcase

TEMPLATE = app

SOURCES +=  tst_gui_actions.cpp

include(../Gui/Gui.pri)

#get UdpServer stuff
unix:!macx: LIBS += -L$$PWD/../../catkin_ws/devel/lib/ -ludpserver
INCLUDEPATH += $$PWD/../../catkin_ws/devel/include
DEPENDPATH += $$PWD/../../catkin_ws/devel/include

INCLUDEPATH += $$PWD/../../catkin_ws/src/platooning/include
DEPENDPATH += $$PWD/../../catkin_ws/src/platooning/include
INCLUDEPATH += $$PWD/../../catkin_ws/devel/include/platooning
DEPENDPATH += $$PWD/../../catkin_ws/devel/include/platooning


INCLUDEPATH += /opt/ros/lunar/include
DEPENDPATH +=/opt/ros/lunar/include

#BOOST
INCLUDEPATH += /usr/include/boost

unix:!macx: LIBS += -L/usr/include/boost -lboost_system
unix:!macx: LIBS += -L/usr/include/boost  -lboost_chrono
unix:!macx: LIBS += -L/usr/include/boost  -lboost_thread

unix:!macx: LIBS += -L$$PWD/../../catkin_ws/devel/lib/ -lmessagetypes

INCLUDEPATH += $$PWD/../../catkin_ws/devel/include
DEPENDPATH += $$PWD/../../catkin_ws/devel/include
