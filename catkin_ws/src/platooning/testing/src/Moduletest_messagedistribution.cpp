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

#include "platooning/Moduletest_messagedistribution.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Moduletest_messagedistribution::Moduletest_messagedistribution() = default;


/*****************************************************************************
** Destructors
*****************************************************************************/

  Moduletest_messagedistribution::~Moduletest_messagedistribution() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void Moduletest_messagedistribution::onInit() {

    name_ = "Moduletest_messagedistribution";

    register_testcases(boost::bind(&Moduletest_messagedistribution::pub_in_platoonMsg_recv_fv_request, this));

    NODELET_INFO(std::string("[" + name_ + "] init done").c_str());

    start_tests();
  };


/*****************************************************************************
** Testcases
*****************************************************************************/

  void Moduletest_messagedistribution::pub_in_platoonMsg_recv_fv_request() {

    set_current_test("pub_in_platoonMsg_recv_fv_request");
    NODELET_INFO(std::string("[" + name_ + "] started testcase " + get_current_test()).c_str());

    //mockup publishers
    pub_map_.clear();
    pub_map_.emplace(topics::IN_PLATOONING_MSG, ros::Publisher());
    pub_map_[topics::IN_PLATOONING_MSG] = nh_.advertise<platooning::platoonProtocol>(topics::IN_PLATOONING_MSG, 1);

    //mockup subscribers
    sub_map_.clear();
    sub_map_.emplace(topics::IN_FV_REQUEST, ros::Subscriber());
    sub_map_[topics::IN_FV_LEAVE] = nh_.subscribe(topics::IN_FV_REQUEST, 1,
                                                  &Moduletest_messagedistribution::hndl_pub_in_platoonMsg_recv_fv_request,
                                                  this);

    boost::shared_ptr<platoonProtocol> inmsg = boost::shared_ptr<platoonProtocol>(new platoonProtocol);

    fv_request msg;
    msg.src_vehicle = 3;

    inmsg->message_type = FV_REQUEST;
    inmsg->payload = encode_message(msg);

    pub_map_[topics::IN_PLATOONING_MSG].publish(inmsg);

  }

  void Moduletest_messagedistribution::hndl_pub_in_platoonMsg_recv_fv_request(platooning::fv_request msg) {

    TestResult res;
    res.success = true;

    if (msg.src_vehicle != 3) {
      res.success = false;
      res.comment = std::string("pub_in_platoonMsg_recv_fv_request:") + "src_vehicle:3 == "
                    + std::to_string(msg.src_vehicle)
                    + "=" + std::to_string(msg.src_vehicle == 3);
    }

    if( msg.src_vehicle == 3) {
      res.success = true;
    }

    finalize_test(res);

  }

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_messagedistribution, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
