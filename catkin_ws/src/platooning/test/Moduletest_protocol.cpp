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

#include "Moduletest_protocol.hpp"
#include <platooning/registerTestcases.h>

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

  Moduletest_protocol::Moduletest_protocol() = default;



/*****************************************************************************
** Destructors
*****************************************************************************/

  Moduletest_protocol::~Moduletest_protocol() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

  /**
  * Set-up necessary publishers/subscribers
  * @return true, if successful
  */
  void Moduletest_protocol::onInit() {

    name_ = "Moduletest_protocol";

    //subscribers of protocol nodelet
    sub_platooningAction = nh_.subscribe("platooningAction", 10,
                                       &Moduletest_protocol::hndl_platooningAction, this);

    std::cout << "subbing to runtestcommand" << std::endl;
    sub_runTestCmd = nh_.subscribe("runTestCommand", 10,
                                   &Moduletest_protocol::hndl_runTestCmd, this);

    //publisher of forced driving vector
    pub_platoonProtocolIn = nh_.advertise< platooning::platoonProtocolIn >("platoonProtocolIn", 10);

    pub_testResult = nh_.advertise<platooning::testResult>("testResult",10);

    std::list<std::string> testcases_to_register = {
        "moduleTest_platooning_leaderrequest"
    };

    register_testcases( testcases_to_register );

    NODELET_INFO("Moduletest_protocol init done");
  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Moduletest_protocol::hndl_platooningAction(platooning::platooningAction msg) {

    platooning::testResult resmsg;
    resmsg.success = true;

    if( msg.actionType != LV_REQUEST ) {
      resmsg.success = false;
    }

    if( msg.vehicleId != 2 ) {
      resmsg.success = false;
    }

    if( msg.platoonId != 3 ) {
      resmsg.success = false;
    }

    std::stringstream resstr;
    resstr << "actionType:" + (msg.actionType == LV_REQUEST)
        << " vehicleId:" << (msg.vehicleId == 2)
        << " platoonId:" << (msg.platoonId == 3);

    //resmsg.comment = resstr.str();

    std::cout << "i wrotes a thing" << std::endl;

    pub_testResult.publish(resmsg);

  }

  void Moduletest_protocol::hndl_runTestCmd(platooning::runTestCommand msg) {

    std::cout << "Protocol: ya talkin to me?" << std::endl;

    if( msg.testToRun != "moduleTest_platooning_leaderrequest") {
      return;
    }

    pt::ptree root;

    root.put("MessageType",(unsigned int) LV_REQUEST );
    root.put("vehicle_id",(unsigned int) 2 );
    root.put("platoon_id",(unsigned int) 3 );
    root.put("ipd",(unsigned int) 4 );
    root.put("ps",(unsigned int) 5 );

    if( !boost::property_tree::json_parser::verify_json(root,1)) {
      NODELET_ERROR("moduletest_protocol invalid json");
    }

    std::stringstream os;

    pt::write_json(os,root,false);

    std::cout << os.str();

    platoonProtocolIn inmsg;

    inmsg.payload = os.str();

    pub_platoonProtocolIn.publish(inmsg);

  }


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_protocol, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
