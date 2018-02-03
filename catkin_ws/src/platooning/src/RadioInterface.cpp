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
#include "RadioInterface.hpp"

namespace platooning {


/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  RadioInterface::RadioInterface() = default;


/*****************************************************************************
** Destructors
*****************************************************************************/

  RadioInterface::~RadioInterface() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  */
  void RadioInterface::onInit() {

    name_ = "radiointerface";

    //subscribers of protocol nodelet
    sub_platoonProtocolOut_ = nh_.subscribe("out/platoonProtocol", 100,
                                            &RadioInterface::hndl_platoonProtocolOut, this);

    pub_platoonProtocolIn_ = nh_.advertise<platoonProtocol>("in/platoonProtocol", 100);

    try {
      //bind to local 10000 port, broadcast to 10000 port
      boost::function<void (std::pair<std::string, int32_t>)> cbfun( boost::bind( boost::mem_fn(
          &RadioInterface::hndl_radio_receive), this, _1 ) );

      server_ptr_ = std::unique_ptr<UdpServer>( new UdpServer( cbfun
                                                 , udp::endpoint(udp::v4(),10000)
                                                 , udp::endpoint(ip::address_v4::broadcast(),10000)));
    } catch (std::exception &e) {
      NODELET_FATAL( std::string("[WIFI] udpserver init failed\n" + std::string(e.what())).c_str());
    }

    NODELET_INFO(std::string("[" + name_ + "] init done").c_str());
  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /**
   * @brief Sends messages into the network
   * @param platoonProtocol msg to be sent
   */
  void RadioInterface::hndl_platoonProtocolOut(platooning::platoonProtocol msg) {
    server_ptr_->start_send(msg.payload, msg.message_type);
  }

  /**
   * @brief handles received messages from the network
   * @param message_pair with message_type and payload
   */
  void RadioInterface::hndl_radio_receive(std::pair<std::string, int32_t> message_pair)  {
    std::cout << "handling radiointerface receive" << std::endl;

    boost::shared_ptr<platooning::platoonProtocol> outmsg
        = boost::shared_ptr<platooning::platoonProtocol>( new platooning::platoonProtocol);

    switch (message_pair.second) {
      case LV_BROADCAST:
      case FV_HEARTBEAT:
      case FV_REQUEST:
      case LV_ACCEPT:
      case LV_REJECT:
      case FV_LEAVE:
        outmsg->payload = message_pair.first;
        pub_platoonProtocolIn_.publish(outmsg);
        break;

      case REMOTE_CMD:
        break;
      case REMOTE_CUSTOM:
        break;
      case REMOTE_LOG:
        //TODO: do something useful
        break;
      default:
        NODELET_FATAL(std::string("[" + name_ + "[ messagetype not recognized").c_str());
        break;

    }
  }


/*****************************************************************************
** HELPER CLASS
*****************************************************************************/


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::RadioInterface, nodelet::Nodelet);
// %EndTag(FULLTEXT)%