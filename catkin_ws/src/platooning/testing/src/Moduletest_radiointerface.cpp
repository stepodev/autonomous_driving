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
#include <boost/thread/thread.hpp>
#include "platooning/Moduletest_radiointerface.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Moduletest_radiointerface::Moduletest_radiointerface() = default;


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

    name_ = Moduletest::name_ = "Moduletest_radiointerface";

    register_testcases(boost::bind(&Moduletest_radiointerface::test_send_udp_recv_protocolIn, this));
    register_testcases(boost::bind(&Moduletest_radiointerface::test_send_protocolOut_recv_udp, this));

    NODELET_INFO(std::string("[" + name_ + "] init done").c_str());

    start_tests();
  };


/*****************************************************************************
** Testcases
*****************************************************************************/

  /**
* @brief tests radiointerface to receive an udp datagram and convert and publish that as a message
*/
  void Moduletest_radiointerface::test_send_udp_recv_protocolIn() {

    set_current_test("test_send_udp_recv_protocolIn");

    NODELET_INFO(std::string("[" + name_ + "] test_send_udp_recv_protocolIn").c_str());

    //prepare subscriber
    sub_map_.clear();
    sub_map_.emplace(topics::IN_PLATOONING_MSG, ros::Subscriber());
    sub_map_[topics::IN_PLATOONING_MSG] = nh_.subscribe(topics::IN_PLATOONING_MSG, 1,
                                                        &Moduletest_radiointerface::hndl_recv_in_protocol,
                                                        this);

    while(sub_map_[topics::IN_PLATOONING_MSG].getNumPublishers() == 0 ) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    //setup server to send udp from
    try {
      boost::function<void (std::pair<std::string, uint32_t>)> cbfun( boost::bind( boost::mem_fn(&Moduletest_radiointerface::hndl_recv_udp_dummy), this, _1 ) );

      server_ = std::unique_ptr<UdpServer>(
          new UdpServer(
              cbfun
              , udp::endpoint(udp::v4(),10001)
              , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));
      server_->set_filter_own_broadcasts(false);
    } catch (std::exception &e) {
      TestResult res;
      res.success = false;
      res.comment = std::string("udpserver init failed ") + e.what();

      finalize_test(res);
    }

    //send msg through our server
    try {
      platooning::platoonProtocol outmsg;
      outmsg.message_type = FV_REQUEST;
      outmsg.payload = get_current_test();

      server_->start_send(outmsg.payload, outmsg.message_type);

    } catch ( std::exception &e) {
      TestResult res;
      res.success = false;
      res.comment = std::string("udpsudpserver start_send failed ") + e.what();

      finalize_test(res);
    }
  }

  void Moduletest_radiointerface::hndl_recv_udp_dummy(std::pair<std::string, uint32_t> msg) {
    NODELET_WARN( std::string("[" + name_ + "][hndl_recv_udp_dummy] unexpectedly received msg\n"
                             + "type " + std::to_string(msg.second) + " payload\n" + msg.first).c_str() );
  }

  void Moduletest_radiointerface::hndl_recv_in_protocol(platooning::platoonProtocol msg) {

    TestResult res;

    if( msg.payload == get_current_test() && msg.message_type == FV_REQUEST) {
      res.success = true;
      res.comment = "";
    }

    if (msg.payload != get_current_test()) {
      res.comment = std::string("[ test_send_udp_recv_protocolIn ] payload mismatch.\nought:\"")
                       + get_current_test() + "\"\nwas   \"" + msg.payload + "\"";
      res.success = false;
    }

    if (msg.message_type != FV_REQUEST) {
      res.comment += std::string("\n[ test_send_udp_recv_protocolIn ] message type mismatch. ought:\"")
                    + std::to_string(FV_REQUEST) + "\"\nwas\"" + std::to_string(msg.message_type) + "\"";
      res.success = false;
    }

    finalize_test(res);
  }

  /**
  * @brief tests radiointerface to receive a message and send as udp datagram
  */

  void Moduletest_radiointerface::test_send_protocolOut_recv_udp() {

    set_current_test("test_send_protocolOut_recv_udp");
    set_timeout( boost::posix_time::seconds(5));

   //prepare server
    try {
      boost::function<void (std::pair<std::string, uint32_t>)> cbfun( boost::bind( boost::mem_fn(&Moduletest_radiointerface::handl_test_udp_recvd), this, _1 ) );

      server_ = std::unique_ptr<UdpServer>( new UdpServer(
          cbfun
          , udp::endpoint(udp::v4(),10000)
          , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));
      server_->set_filter_own_broadcasts(false);
    } catch (std::exception &e) {
      TestResult res;
      res.success = false;
      res.comment = std::string("udpserver init failed ") + e.what();

      finalize_test(res);
      return;
    }

    //prepare publisher
    pub_map_.clear();
    pub_map_.emplace( topics::OUT_PLATOONING_MSG, ros::Publisher());
    pub_map_[topics::OUT_PLATOONING_MSG] = nh_.advertise<platooning::platoonProtocol>(topics::OUT_PLATOONING_MSG, 10);

    while(pub_map_[topics::OUT_PLATOONING_MSG].getNumSubscribers() == 0 ) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if( pub_map_[topics::OUT_PLATOONING_MSG].getNumSubscribers() == 0 ) {
      TestResult res;
      res.success = false;
      res.comment = "test_send_protocolOut_recv_udp: no subscribers to topics::OUT_PLATOONING_MSG";
      finalize_test(res);
      return;
    }

    boost::shared_ptr<platooning::platoonProtocol> msg = boost::shared_ptr<platooning::platoonProtocol>( new platooning::platoonProtocol);

    msg->payload = get_current_test();
    msg->message_type = FV_LEAVE;

    pub_map_[topics::OUT_PLATOONING_MSG].publish(msg);
  }

  void Moduletest_radiointerface::handl_test_udp_recvd( std::pair<std::string, uint32_t> msg_pair ) {

    TestResult res;

    if( msg_pair.first == get_current_test() && msg_pair.second == FV_LEAVE) {
      res.success = true;
    } else if (msg_pair.first != get_current_test()) {
      res.comment = std::string(" payload mismatch.\nought:\"")
                        + get_current_test() + "\"\nwas   \"" + msg_pair.first + "\"";

      res.success = false;
    } else if (msg_pair.second != FV_LEAVE) {
      res.comment += std::string("\nmessage_type mismatch.\nought:\"")
                        + std::to_string(FV_LEAVE)
                        + "\"\nwas   \"" + std::to_string(msg_pair.second) + "\"";
      res.success = false;
    } else {
      res.success = false;
      res.comment = "handl_test_udp_recvd: unknown failure";
    }
    finalize_test(res);
  }
} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_radiointerface, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
