#ifndef CONTROLLERUI_H
#define CONTROLLERUI_H

#include <QMainWindow>
#include <QKeyEvent>
#include <QDebug>
#include <QMessageBox>
#include <boost/property_tree/ptree.hpp> //json parsing and generating
#include <boost/property_tree/json_parser.hpp> //json parsing and generating
#include <boost/algorithm/string/join.hpp>
#include <iostream>
#include <chrono>
#include <ctime>

#include "platooning/UdpServer.hpp"
#include "platooning/MessageTypes.hpp"

namespace Ui {
class ControllerUi;
}

class ControllerUi : public QMainWindow
{
    Q_OBJECT

public:
    explicit ControllerUi(QWidget *parent = 0);
    ~ControllerUi();

    void add_slave_vehicle( uint32_t vehicle_id);

protected:
    void keyPressEvent(QKeyEvent *);
    void keyReleaseEvent(QKeyEvent *);

private slots:
    void on_togglePlatooning_v1_clicked();
    void on_toggleRemote_v1_clicked();

    void on_togglePlatooning_v2_clicked();
    void on_toggleRemote_v2_clicked();

private:
    Ui::ControllerUi *ui;
    bool remoteEnabled_v1_ = false;
    bool remoteEnabled_v2_ = false;
    bool platooningEnabled_v1_ = false;
    bool platooningEnabled_v2_ = false;

    std::unique_ptr<UdpServer> server_ptr_;
    boost::function<void (boost::shared_ptr<std::pair<std::string, uint32_t>>)> recv_udp_msg_cb;
    std::list<int32_t> slave_vehicle_ids_;
    boost::thread keypollthread_;

    float remote_lat_angle_v1_ = 0;
    float remote_speed_v1_ = 0;
    float remote_lat_angle_v2_ = 0;
    float remote_speed_v2_ = 0;

    void receive_message( boost::shared_ptr<std::pair<std::string, uint32_t>> msgpair ) ;
    void keypresspoll();
    void set_platooningbutton_v1( const bool& enabled );
    void set_platooningbutton_v2( const bool& enabled );
    void set_remotecontrolbutton_v1( const bool& enabled );
    void set_remotecontrolbutton_v2( const bool& enabled );
};

#endif // CONTROLLERUI_H
