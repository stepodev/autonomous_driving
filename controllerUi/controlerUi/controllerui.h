#ifndef CONTROLLERUI_H
#define CONTROLLERUI_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QDebug>
#include <boost/property_tree/ptree.hpp> //json parsing and generating
#include <boost/property_tree/json_parser.hpp> //json parsing and generating

#include "UdpServer.hpp"
#include "MessageTypes.hpp"

namespace Ui {
class ControllerUi;
}

class ControllerUi : public QMainWindow
{
    Q_OBJECT

public:
    explicit ControllerUi(QWidget *parent = 0);
    ~ControllerUi();

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);

private slots:
    void on_startPlatooning_clicked();

    void on_toggleRemote_clicked();

private:
    Ui::ControllerUi *ui;
    bool remoteEnabled = false;
    std::unique_ptr<UdpServer> server_ptr_;

    void receive_message( std::pair<std::string, int32_t> msgpair ) ;
};

#endif // CONTROLLERUI_H
