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
      //bind to local 10000 port, broadcast to 10000 port
      boost::function<void (std::shared_ptr<std::vector<char>>)> cbfun( boost::bind( &Wifi::hndl_wifi_receive, this, _1 ) );

      server_ = std::unique_ptr<UdpServer>(new UdpServer(io_service_
                                                 , cbfun
                                                 , udp::endpoint(udp::v4(),10000)
                                                 , udp::endpoint(ip::address_v4::broadcast(),10000)));
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

    server_->start_send(std::move(msg));

  }

  void Wifi::hndl_wifi_receive(std::shared_ptr<std::vector<char>> msg)  {

    char message_type_array[sizeof(int32_t)];
    memcpy(message_type_array, msg->data(), sizeof(int32_t));

    int32_t message_type;
    sscanf(message_type_array, "%i", &message_type);

    //hopefully the whole string without the message
    std::string str( msg->begin()+sizeof(int32_t),msg->end());

    platooning::platoonProtocolIn outmsg;

    switch (message_type) {
      case LV_BROADCAST:
      case FV_HEARTBEAT:
      case LV_REQUEST:
      case FV_REQUEST:
      case ACCEPT_RESPONSE:
      case REJECT_RESPONSE:
      case LEAVE_PLATOON:
        outmsg.payload = str;
        pub_platoonProtocolIn_.publish(outmsg);
        break;

      default:
        break;

    }
  }


/*****************************************************************************
** HELPER CLASS
*****************************************************************************/


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Wifi, nodelet::Nodelet);
// %EndTag(FULLTEXT)%