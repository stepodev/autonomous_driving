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
#include "Moduletest_radiointerface.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Moduletest_radiointerface::Moduletest_radiointerface()
      : Moduletest( { "moduletest_radiointerface_send_udp_recv_protocolIn"
                        , "moduletest_radiointerface_send_protocolOut_recv_udp"}) {};


/*****************************************************************************
** Destructors
*****************************************************************************/

  Moduletest_radiointerface::~Moduletest_radiointerface() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void Moduletest_radiointerface::onInit() {

    name_ = "Moduletest_radiointerface";

    sub_runTestCmd = nh_.subscribe("runTestCommand", 10,
                                   &Moduletest_radiointerface::hndl_runTestCmd, this);

    sub_platoonProtocolIn = nh_.subscribe("in/platoonProtocol", 10,
                                   &Moduletest_radiointerface::hndl_platoonProtocolIn, this);

    pub_testResult = nh_.advertise<platooning::testResult>("testResult", 10);

    std::list<std::string> testcases_to_register = {

    };

    NODELET_INFO(std::string("[" + name_ + "] init done").c_str());
  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * @brief Handles RadioInterface published messages to our graph
   */

  void Moduletest_radiointerface::hndl_platoonProtocolIn(platooning::platoonProtocol msg) {

    NODELET_INFO(std::string("[" + name_ + "] moduletest_radiointerface] recvd in/protocol").c_str());

    if( current_test_ != "moduletest_radiointerface_send_udp_recv_protocolIn") {
      return;
    }

    std::cout << "Moduletest_radiointerface handl in/platoonProtocol " << std::endl;

    boost::shared_ptr<testResult> outmsg = boost::shared_ptr<testResult>( new testResult);

    outmsg->success = true;

    if (msg.payload != current_test_) {
      outmsg->comment = std::string("[moduletest_radiointerface] payload mismatch.\nought:\"")
                       + current_test_ + "\"\nwas   \"" + msg.payload + "\"";
      NODELET_WARN( outmsg->comment.c_str());
      outmsg->success = false;
    } else {
      NODELET_INFO(std::string("[" + name_ + "] GREATU SUCCESSU " + current_test_).c_str());
    }

    pub_testResult.publish(outmsg);

    server_ = nullptr;
  }

  void Moduletest_radiointerface::handl_udp_recvd( std::pair<std::string, int32_t> msg_pair ) {

    if( current_test_ != "moduletest_radiointerface_send_protocolOut_recv_udp") {
      std::cout << "HELLOE" << std::endl;
      return;
    }

    std::cout << "Moduletest_radiointerface handl udp recvd " << std::endl;

    boost::shared_ptr<testResult> outmsg = boost::shared_ptr<testResult>( new testResult);

    outmsg->success = true;

    if (msg_pair.first != current_test_) {
      outmsg->comment = std::string("[moduletest_radiointerface] payload mismatch.\nought:\"")
                        + current_test_ + "\"\nwas   \"" + msg_pair.first + "\"";
      NODELET_WARN( outmsg->comment.c_str());
      outmsg->success = false;
    }

    if (msg_pair.first != current_test_) {
      outmsg->comment += std::string("\n[moduletest_radiointerface] message_type mismatch.\nought:\"")
                        + std::to_string(FV_LEAVE)
                        + "\"\nwas   \"" + std::to_string(msg_pair.second) + "\"";
      NODELET_WARN( outmsg->comment.c_str());
      outmsg->success = false;
    }

    pub_testResult.publish(outmsg);

    pub_platoonProtocolOut.shutdown();

    server_ = nullptr;

    if( outmsg->success ) {
      NODELET_INFO(std::string("[" + name_ + "] GREATU SUCCESSU " + current_test_).c_str());
    }

  }

  /**
 * @brief tests radiointerface to receive an udp datagram and convert and publish that as a message
 */
  void Moduletest_radiointerface::test_send_udp_recv_protocolIn() {

    NODELET_INFO(std::string("[" + name_ + "] test_send_udp_recv_protocolIn").c_str());

    try {
      boost::function<void (std::pair<std::string, int32_t>)> cbfun( boost::bind( boost::mem_fn(&Moduletest_radiointerface::handl_udp_recvd), this, _1 ) );

      server_ = std::unique_ptr<UdpServer>(
          new UdpServer(
              cbfun
            , udp::endpoint(udp::v4(),10001)
            , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));
      server_->set_filter_own_broadcasts(false);
    } catch (std::exception &e) {
      NODELET_FATAL( std::string("[" + name_ + "] udpserver init failed\n" + std::string(e.what())).c_str());
    }

    try {
      platooning::platoonProtocol outmsg;
      outmsg.message_type = FV_REQUEST;
      outmsg.payload = current_test_;

      server_->start_send(outmsg.payload, outmsg.message_type);
    } catch ( std::exception &e) {
      NODELET_FATAL( std::string("[" + name_ + "] udpserver start_send failed\n" + std::string(e.what())).c_str());
    }
  }

  /**
 * @brief tests radiointerface to receive a message and send as udp datagram
 */
  void Moduletest_radiointerface::test_send_protocolOut_recv_udp() {

    NODELET_INFO(std::string("[" + name_ + "] test_send_protocolOut_recv_udp").c_str());

    try {
      boost::function<void (std::pair<std::string, int32_t>)> cbfun( boost::bind( boost::mem_fn(&Moduletest_radiointerface::handl_udp_recvd), this, _1 ) );

      server_ = std::unique_ptr<UdpServer>( new UdpServer(
            cbfun
          , udp::endpoint(udp::v4(),10000)
          , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));
      server_->set_filter_own_broadcasts(false);
    } catch (std::exception &e) {
      NODELET_FATAL( std::string("[" + name_ + "]udpserver init failed\n" + std::string(e.what())).c_str());
    }

    NODELET_INFO(std::string("[" + name_ + "] preparing message").c_str());

    pub_platoonProtocolOut = nh_.advertise<platooning::platoonProtocol>("out/platoonProtocol", 10);

    if( pub_platoonProtocolOut.getNumSubscribers() == 0 ) {
      NODELET_FATAL(std::string("[" + name_ + "] test_send_protocolOut_recv_udp: no subscribers to platoonProtocolOut").c_str());
      boost::shared_ptr<platooning::testResult> msg = boost::shared_ptr<platooning::testResult>( new platooning::testResult);
      msg->success = false;
      msg->comment = "[" + name_ + " ] test_send_protocolOut_recv_udp: no subscribers to platoonProtocolOut";
      pub_testResult.publish(msg);
      return;
    }

    boost::shared_ptr<platooning::platoonProtocol> msg = boost::shared_ptr<platooning::platoonProtocol>( new platooning::platoonProtocol);

    msg->payload = current_test_;
    msg->message_type = FV_LEAVE;

    pub_platoonProtocolOut.publish(msg);

    NODELET_INFO(std::string("[" + name_ + "] test_send_protocolOut_recv_udp published").c_str());

  }

  void Moduletest_radiointerface::hndl_runTestCmd(platooning::runTestCommand msg) {


    if (msg.testToRun == "moduletest_radiointerface_send_udp_recv_protocolIn") {
      current_test_ = "moduletest_radiointerface_send_udp_recv_protocolIn";
      test_send_udp_recv_protocolIn();
    }


    if (msg.testToRun == "moduletest_radiointerface_send_protocolOut_recv_udp") {
      current_test_ = "moduletest_radiointerface_send_protocolOut_recv_udp";
      test_send_protocolOut_recv_udp();
    }

  }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_radiointerface, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
