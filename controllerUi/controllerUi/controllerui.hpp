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

    void add_slave_vehicle( int32_t vehicle_id);

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);

private slots:
    void on_startPlatooning_clicked();

    void on_toggleRemote_clicked();

private:
    Ui::ControllerUi *ui;
    bool remoteEnabled_ = false;
    std::unique_ptr<UdpServer> server_ptr_;
    std::list<int32_t> slave_vehicle_ids_;
    boost::thread keypollthread_;

    float remote_lat_angle = 0;
    float remote_speed = 0;

    void receive_message( std::pair<std::string, int32_t> msgpair ) ;
    void keypresspoll();
};

#endif // CONTROLLERUI_H
