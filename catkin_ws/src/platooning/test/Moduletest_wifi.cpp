//
// Created by stepo on 12/16/17.
//


/**
 * @file /platooning/src/platooning.cpp
 *
 * @brief Nodelet implementation of RemoteContol
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <thread>
#include <platooning/registerTestcases.h>
#include <boost/thread/thread.hpp>
#include "Moduletest_wifi.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Moduletest_wifi::Moduletest_wifi() = default;


/*****************************************************************************
** Destructors
*****************************************************************************/

  Moduletest_wifi::~Moduletest_wifi() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void Moduletest_wifi::onInit() {

    name_ = "Moduletest_wifi";

    sub_runTestCmd = nh_.subscribe("runTestCommand", 10,
                                   &Moduletest_wifi::hndl_runTestCmd, this);

    sub_platoonProtocolIn = nh_.subscribe("platoonProtocolIn", 10,
                                   &Moduletest_wifi::hndl_platoonProtocolIn, this);

    pub_testResult = nh_.advertise<platooning::testResult>("testResult", 10);

    std::list<std::string> testcases_to_register = {
        "moduleTest_wifi_send_udp_recv_protocolIn"
        , "moduleTest_wifi_send_protocolOut_recv_udp"
    };

    register_testcases(testcases_to_register);

    NODELET_INFO("[moduletest_wifi] init done");
  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * @brief Handles Wifi published messages to our graph
   */

  void Moduletest_wifi::hndl_platoonProtocolIn(platooning::platoonProtocolIn msg) {

    boost::shared_ptr<testResult> outmsg = boost::shared_ptr<testResult>( new testResult);

    outmsg->success = true;

    if (msg.payload != current_test_) {
      outmsg->comment = std::string("[moduletest_wifi] payload mismatch.\nought:\"")
                       + current_test_ + "\"\nwas   \"" + msg.payload + "\"";
      NODELET_WARN( outmsg->comment.c_str());
      outmsg->success = false;
    } else {
      NODELET_INFO("[moduletest_wifi] GREATU SUCCESSU");
    }

    pub_testResult.publish(outmsg);

    server_ = nullptr;
  }

  void Moduletest_wifi::handl_udp_recvd( std::pair<std::string, int32_t> msg_pair ) {

    std::cout << "modiletest wifi handl udp recvd " << std::endl;

  }

  /**
 * @brief tests wifi to receive an udp datagram and convert and publish that as a message
 */
  void Moduletest_wifi::test_send_udp_recv_protocolIn() {

    NODELET_INFO("[Moduletest wifi] test_send_udp_recv_protocolIn");

    try {
      boost::function<void (std::pair<std::string, int32_t>)> cbfun( boost::bind( boost::mem_fn(&Moduletest_wifi::handl_udp_recvd), this, _1 ) );

      server_ = std::unique_ptr<UdpServer>(
          new UdpServer(
              cbfun
            , udp::endpoint(udp::v4(),10001)
            , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));
    } catch (std::exception &e) {
      NODELET_FATAL( std::string("[moduletest_wifi] udpserver init failed\n" + std::string(e.what())).c_str());
    }

    try {
      platooning::platoonProtocolOut outmsg;
      outmsg.message_type = LV_REQUEST;
      outmsg.payload = "moduleTest_wifi_send_udp_recv_protocolIn";

      server_->start_send(outmsg.payload, outmsg.message_type);
    } catch ( std::exception &e) {
      NODELET_FATAL( std::string("[moduletest_wifi] udpserver start_send failed\n" + std::string(e.what())).c_str());
    }
  }

  /**
 * @brief tests wifi to receive a message and send as udp datagram
 */
  void Moduletest_wifi::test_send_protocolOut_recv_udp() {

    NODELET_INFO("[Moduletest wifi] test_send_protocolOut_recv_udp");

    try {
      boost::function<void (std::pair<std::string, int32_t>)> cbfun( boost::bind( boost::mem_fn(&Moduletest_wifi::handl_udp_recvd), this, _1 ) );

      server_ = std::unique_ptr<UdpServer>( new UdpServer(
            cbfun
          , udp::endpoint(udp::v4(),10001)
          , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));
    } catch (std::exception &e) {
      NODELET_FATAL( std::string("[moduletest_wifi] udpserver init failed\n" + std::string(e.what())).c_str());
    }

    ros::Publisher pub_platoonProtocolOut = nh_.advertise<platooning::platoonProtocolOut>("platoonProtocolOut", 10);

    if( pub_platoonProtocolOut.getNumSubscribers() == 0 ) {
      NODELET_FATAL("Moduletest wifi] test_send_protocolOut_recv_udp: no subscribers to platoonProtocolOut");
      boost::shared_ptr<platooning::testResult> msg = boost::shared_ptr<platooning::testResult>( new platooning::testResult);
      msg->success = false;
      msg->comment = "[Moduletest wifi] test_send_protocolOut_recv_udp: no subscribers to platoonProtocolOut";
      pub_testResult.publish(msg);
      return;
    }

    boost::shared_ptr<platooning::platoonProtocolOut> msg = boost::shared_ptr<platooning::platoonProtocolOut>( new platooning::platoonProtocolOut);

    msg->payload = "moduleTest_wifi_send_protocolOut_recv_udp";
    msg->message_type = LEAVE_PLATOON;

    pub_platoonProtocolOut.publish(msg);

  }

  void Moduletest_wifi::hndl_runTestCmd(platooning::runTestCommand msg) {


    if (msg.testToRun == "moduleTest_wifi_send_udp_recv_protocolIn") {
      current_test_ = "moduleTest_wifi_send_udp_recv_protocolIn";
      test_send_udp_recv_protocolIn();
    }


    if (msg.testToRun == "moduleTest_wifi_send_protocolOut_recv_udp") {
      current_test_ = "moduleTest_wifi_send_protocolOut_recv_udp";
      test_send_protocolOut_recv_udp();
    }

  }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_wifi, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
