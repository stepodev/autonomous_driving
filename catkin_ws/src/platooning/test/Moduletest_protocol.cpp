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

  Moduletest_protocol::Moduletest_protocol()
  : Moduletest( {"moduleTest_protocol_fv_request"} ){}



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

    std::cout << "[moduletest Protocol]: init?" << std::endl;

    name_ = "Moduletest_protocol";

    //subscribers of protocol nodelet
    sub_fv_request = nh_.subscribe("fv_request", 10,
                                       &Moduletest_protocol::hndl_fv_request, this);

    std::cout << "subbing to runtestcommand" << std::endl;
    sub_runTestCmd = nh_.subscribe("runTestCommand", 10,
                                   &Moduletest_protocol::hndl_runTestCmd, this);

    pub_platoonProtocolIn = nh_.advertise< platooning::platoonProtocolIn >("platoonProtocolIn", 10);

    pub_testResult = nh_.advertise<platooning::testResult>("testResult",10);

    NODELET_INFO("Moduletest_protocol init done");
  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Moduletest_protocol::hndl_fv_request(platooning::fv_request msg) {

    std::cout << "[moduletest Protocol]: handlin fv_request?" << std::endl;

    boost::shared_ptr<platooning::testResult> resmsg
        = boost::shared_ptr<platooning::testResult>( new platooning::testResult);
        resmsg->success = true;

    if( msg.src_vehicle != 2 ) {
      resmsg->success = false;
      std::stringstream resstr;
      resstr << "src_vehicle:2 == " << msg.src_vehicle << "=" << (msg.src_vehicle == 2);

      resmsg->comment = resstr.str();
    }

    if( resmsg->success ) {
      NODELET_ERROR( (std::string("[") + name_ + "] error with " + resmsg->comment).c_str());
    }

    pub_testResult.publish(resmsg);

  }

  void Moduletest_protocol::hndl_runTestCmd(platooning::runTestCommand msg) {

    std::cout << "Protocol: ya talkin to me?" << std::endl;

    if( msg.testToRun != "moduleTest_protocol_fv_request") {
      return;
    }

    pt::ptree root;

    root.put("src_vehicle",(unsigned int) 2 );
    root.put("platoon_id",(unsigned int) 3 );
    root.put("ipd",(float) 4 );
    root.put("ps",(float) 5 );

    if( !boost::property_tree::json_parser::verify_json(root,1)) {
      NODELET_ERROR("moduletest_protocol invalid json");
    }

    std::stringstream os;

    pt::write_json(os,root,false);

    std::cout << os.str();

    platoonProtocolIn inmsg;

    inmsg.message_type = FV_REQUEST;
    inmsg.payload = os.str();

    pub_platoonProtocolIn.publish(inmsg);

  }


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_protocol, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
