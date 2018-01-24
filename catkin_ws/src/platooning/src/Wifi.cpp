//
// Created by stepo on 12/16/17.
//


/**
 * @file /platooning/src/wifi.cpp
 *
 * @brief Implements Listeners and Senders for traffic towards the network
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include "Wifi.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Wifi::Wifi() = default;


/*****************************************************************************
** Destructors
*****************************************************************************/

  Wifi::~Wifi() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  */
  void Wifi::onInit() {

    name_ = "wifi";

    //subscribers of protocol nodelet
    sub_platoonProtocolOut_ = nh_.subscribe("platoonProtocolOut", 10,
                                            &Wifi::hndl_platoonProtocolOut, this);

    pub_platoonProtocolIn_ = nh_.advertise<platoonProtocolIn>("platoonProtocolIn", 10);

    io_thread = boost::thread([this]() { this->io_service_.run(); });
    std::cout << "iothread" << std::endl;

    try {
      server_ = std::unique_ptr<UdpServer>(new UdpServer(io_service_,pub_platoonProtocolIn_));
    } catch (std::exception &e) {
      NODELET_FATAL( std::string("[WIFI] udpserver init failed\n" + std::string(e.what())).c_str());
    }

    NODELET_INFO("[WIFI] init done");


  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Wifi::hndl_platoonProtocolOut(platooning::platoonProtocolOut msg) {

    server_->start_send(msg);

  }


/*****************************************************************************
** HELPER CLASS
*****************************************************************************/


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Wifi, nodelet::Nodelet);
// %EndTag(FULLTEXT)%