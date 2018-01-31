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

void ControllerUi::receive_message( std::pair<std::string, int32_t> msgpair )
{
    /*
    namespace pt = boost::property_tree;

    std::stringstream ss(msgpair->second);

    pt::ptree root;
    boost::property_tree::read_json( ss, root );

    if(msgpair.second == REMOTE_LOG ) {
        //parse stuff
    }
    */
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
