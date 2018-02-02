#include "controllerui.hpp"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ControllerUi w;
    w.show();

    //adds vehicles to listen to from args
    w.add_slave_vehicle(2);

    return a.exec();
}
