#include "controllerui.hpp"
#include "ui_controllerui.h"

ControllerUi::ControllerUi(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ControllerUi)
{
    ui->setupUi(this);
    QWidget::grabKeyboard(); //only this widget get keypresses

    //start server
    boost::function<void (std::pair<std::string, int32_t>)> cbfun( boost::bind( boost::mem_fn(&ControllerUi::receive_message), this, _1 ) );

    server_ptr_ = std::unique_ptr<UdpServer>( new UdpServer( cbfun
                                                  , udp::endpoint(udp::v4(),10000)
                                                  , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));

}

ControllerUi::~ControllerUi()
{
    delete ui;
}

void ControllerUi::on_startPlatooning_clicked()
{
    server_ptr_->start_send("go",REMOTE_CUSTOM);
}

void ControllerUi::add_slave_vehicle( int32_t vehicle_id) {
    slave_vehicle_ids.emplace_back(vehicle_id);
}

void ControllerUi::receive_message( std::pair<std::string, int32_t> msgpair )
{

    namespace pt = boost::property_tree;

    std::stringstream ss(msgpair->second);

    pt::ptree root;
    boost::property_tree::read_json( ss, root );

    inf32_t vehicle_id = 0;

    if(msgpair.second == REMOTE_LOG ) {
        try {
          ui->info_vehicle_id = root.get_value<int32_t>("vehicle_id");
        } catch( std::exception) {} //do we care?
    }

    if(msgpair.second == REMOTE_LOG && slave_vehicle_ids.find( vehicle_id ) ) {
        try {
            if(root.get_value<bool>("leading_vehicle")) {
                ui->info_lvfv = "LEADER";
            }
        } catch( std::exception) {} //do we care?

        try {
            if(root.get_value<bool>("following_vehicle")) {
                ui->info_lvfv = "FOLLOWER";
            }
        } catch( std::exception) {} //do we care?

        try {
          ui->info_ipd = root.get_value<float>("inner_platoon_distance");
        } catch( std::exception) {} //do we care?

        try {
          ui->info_actual_distance = root.get_value<float>("actual_distance");
        } catch( std::exception) {} //do we care?

        try {
          ui->info_ps = root.get_value<float>("platoon_speed");
        } catch( std::exception) {} //do we care?

        try {
          ui->info_actual_speed = root.get_value<float>("speed");
        } catch( std::exception) {} //do we care?

        try {
          ui->info_platooningstate = root.get_value<std::string>("platooning_state");
        } catch( std::exception) {} //do we care?

        try {
          ui->info_platoonsize = root.get_value<int32_t>("platoon_size");
        } catch( std::exception) {} //do we care?

        try {
          ui->info_platoonmembers = root.get_value<std::string>("platoon_members");
        } catch( std::exception) {} //do we care?
    }
}

void ControllerUi::on_toggleRemote_clicked()
{
    if( !remoteEnabled ) {
        remoteEnabled = true;
        ui->toggleRemote->setStyleSheet("background-color: red");
        ui->cursorDown->setFlat(false);
        ui->cursorUp->setFlat(false);
        ui->cursorLeft->setFlat(false);
        ui->cursorRight->setFlat(false);
    } else {
        remoteEnabled = false;
        ui->toggleRemote->setStyleSheet("background-color: lightgrey");
        ui->cursorDown->setFlat(true);
        ui->cursorUp->setFlat(true);
        ui->cursorLeft->setFlat(true);
        ui->cursorRight->setFlat(true);
    }
}

void ControllerUi::keyPressEvent(QKeyEvent *event)
{
    if(!remoteEnabled) {
        return;
    }

    switch(event->key() )
    {
    case Qt::Key_Left:
        ui->cursorLeft->setDown(true);
        break;
    case Qt::Key_Right:
        ui->cursorRight->setDown(true);
        break;
    case Qt::Key_Up:
        ui->cursorUp->setDown(true);
        break;
    case Qt::Key_Down:
        ui->cursorDown->setDown(true);
        break;
    default:
        break;
    }
}

void ControllerUi::keyReleaseEvent(QKeyEvent *event)
{
    if(!remoteEnabled) {
        return;
    }

    switch(event->key() )
    {
    case Qt::Key_Left:
        ui->cursorLeft->setDown(false);
        break;
    case Qt::Key_Right:
        ui->cursorRight->setDown(false);
        break;
    case Qt::Key_Up:
        ui->cursorUp->setDown(false);
        break;
    case Qt::Key_Down:
        ui->cursorDown->setDown(false);
        break;
    default:
        break;
    }
}
