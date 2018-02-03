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

    server_ptr_->set_filter_own_broadcasts(false);
}

ControllerUi::~ControllerUi()
{
    delete ui;
}

void ControllerUi::on_startPlatooning_clicked()
{
    platooning::platooningToggle msg;
    msg.enable_platooning = remoteEnabled_;
    server_ptr_->start_send(platooning::encode_message(msg),REMOTE_CONTROLTOGGLE);
}

void ControllerUi::add_slave_vehicle( int32_t vehicle_id) {
    slave_vehicle_ids_.emplace_back(vehicle_id);
}

void ControllerUi::receive_message( std::pair<std::string, int32_t> msgpair )
{
    try {
        if( msgpair.first.empty()) {
            return;
        }

        switch( msgpair.second ) {

            case REMOTE_USERINTERFACE:
            platooning::userInterface msg;
            platooning::decode_json(msgpair.first, msg);

            try { ui->info_actual_distance->setText( std::to_string(msg.actual_distance).c_str() );} catch( std::exception &ex ) {}
            try { ui->info_actual_speed->setText( std::to_string(msg.speed).c_str() );} catch( std::exception &ex ) {}
            try { ui->info_ipd->setText( std::to_string(msg.inner_platoon_distance).c_str() );} catch( std::exception &ex ) {}
            try { ui->info_lvfv->setText( msg.following_vehicle ? "FV" : "" );} catch( std::exception &ex ) {}
            try { ui->info_lvfv->setText( msg.leading_vehicle ? "FV" : "" );} catch( std::exception &ex ) {}
            try { ui->info_platooningstate->setText( msg.platooning_state.c_str() );} catch( std::exception &ex ) {}
            try { ui->info_platoonmembers->setText( boost::algorithm::join(msg.platoon_members),',').c_str();} catch( std::exception &ex ) {}


            break;
        default:
            break;
        }

        std::find(slave_vehicle_ids_.begin(), slave_vehicle_ids_.end(), remotevehicle_id) != slave_vehicle_ids_.end();


    } catch( std::exception &ex ) {
        std::cerr << "receive_message crash with " << ex.what() << std::endl;
    }
}

void ControllerUi::on_toggleRemote_clicked()
{
    try {
        if( !remoteEnabled_ ) {
            remoteEnabled_ = true;
            ui->toggleRemote->setStyleSheet("background-color: red");
            ui->cursorDown->setFlat(false);
            ui->cursorUp->setFlat(false);
            ui->cursorLeft->setFlat(false);
            ui->cursorRight->setFlat(false);
            ui->info_remote_lat->setDisabled(false);
            ui->info_remote_speed->setDisabled(false);
            keypollthread_ = boost::thread([this] {
                while(remoteEnabled_) {
                    keypresspoll();
                }
            });
        } else {
            remoteEnabled_ = false;
            ui->toggleRemote->setStyleSheet("background-color: lightgrey");
            ui->cursorDown->setFlat(true);
            ui->cursorUp->setFlat(true);
            ui->cursorLeft->setFlat(true);
            ui->cursorRight->setFlat(true);
            ui->info_remote_lat->setText("");
            ui->info_remote_speed->setText("");
            ui->info_remote_lat->setDisabled(true);
            ui->info_remote_speed->setDisabled(true);
        }

        platooning::remotecontrolToggle msg;
        msg.enable_remotecontrol = remoteEnabled_;
        server_ptr_->start_send(platooning::encode_message(msg), REMOTE_CONTROLTOGGLE);

    } catch( std::exception &ex) {
        std::cerr << "toggleremote crash with " << ex.what() << std::endl;
    }

}

void ControllerUi::keypresspoll() {
    try {
        boost::property_tree::ptree root;
        std::stringstream os;

        while(remoteEnabled_) {
            if(ui->cursorUp->isDown() && remote_speed < 2.8) {
                remote_speed += 0.1f;

                ui->info_remote_speed->insert( std::to_string( remote_speed).c_str());
            }

            if(ui->cursorDown->isDown() && remote_speed >= 0) {
                remote_speed -= 0.1f;

                if( remote_speed < 0 ) {
                    remote_speed = 0;
                }

                ui->info_remote_speed->insert( std::to_string( remote_speed).c_str());
            }

            if(ui->cursorLeft->isDown() && remote_lat_angle > -70 ) {
                remote_lat_angle -= 3;
                ui->info_remote_lat->insert( std::to_string( remote_lat_angle).c_str());
            }

            if(ui->cursorRight->isDown() && remote_lat_angle < 70 ) {
                remote_lat_angle += 3;
                ui->info_remote_lat->insert( std::to_string( remote_lat_angle).c_str());
            }

            platooning::remotecontrolInput msg;

            msg.remote_speed = remote_speed;
            msg.remote_angle = remote_lat_angle;
            msg.emergency_stop = false;
            server_ptr_->start_send(platooning::encode_message(msg),REMOTE_CONTROLINPUT);
            boost::this_thread::sleep_for(boost::chrono::milliseconds(1500));
        }
    } catch( std::exception &ex) {
        std::cerr << "keypresspoll crash with " << ex.what() << std::endl;
    }
}

void ControllerUi::keyPressEvent(QKeyEvent *event)
{
    if(!remoteEnabled_) {
        return;
    }

    if(event->key() == Qt::Key_Left ) {
        ui->cursorLeft->setDown(true);
    }

    if(event->key() == Qt::Key_Right ) {
        ui->cursorRight->setDown(true);
    }

    if(event->key() == Qt::Key_Up ) {
        ui->cursorUp->setDown(true);
    }

    if(event->key() == Qt::Key_Down ) {
        ui->cursorDown->setDown(true);
    }
}

void ControllerUi::keyReleaseEvent(QKeyEvent *event)
{
    if(!remoteEnabled_) {
        return;
    }

    if(event->key() == Qt::Key_Left ) {
        ui->cursorLeft->setDown(false);
    }

    if(event->key() == Qt::Key_Right ) {
        ui->cursorRight->setDown(false);
    }

    if(event->key() == Qt::Key_Up ) {
        ui->cursorUp->setDown(false);
    }

    if(event->key() == Qt::Key_Down ) {
        ui->cursorDown->setDown(false);
    }
}
