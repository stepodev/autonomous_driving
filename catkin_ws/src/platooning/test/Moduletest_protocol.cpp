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

#include "Moduletest_protocol.hpp"

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

    //subscribers of protocol nodelet
    sub_platooningAction = nh_.subscribe("platooningAction", 10,
                                       &Moduletest_protocol::hndl_platooningAction, this);

    //publisher of forced driving vector
    pub_platoonProtocolIn = nh_.advertise< platooning::platoonProtocolIn >("platoonProtocolIn", 10);

    genTest1();

  };


/*****************************************************************************
** Handlers
*****************************************************************************/

  /*
   * handling an event and publishing something
   */
  void Moduletest_protocol::hndl_platooningAction(platooning::platooningAction msg) {

    std::ofstream myfile;
    myfile.open ("test1.txt");

    myfile << "actionType:" << (msg.actionType == LEADER_REQUEST)
        << " vehicleId:" << (msg.vehicleId == 2)
        << " platoonId:" << (msg.platoonId == 3)
        << std::endl;

    myfile.close();

    std::cout << "i wrotes a thing" << std::endl;

  }

  void Moduletest_protocol::genTest1() {

    pt::ptree root;

    root.put("MessageType",(unsigned int) LEADER_REQUEST );
    root.put("vehicle_id",(unsigned int) 2 );
    root.put("platoon_id",(unsigned int) 3 );
    root.put("ipd",(unsigned int) 4 );
    root.put("ps",(unsigned int) 5 );

    if( !boost::property_tree::json_parser::verify_json(root,1)) {
      NODELET_ERROR("moduletest_protocol invalid json");
    }

    std::stringstream os;

    pt::write_json(os,root,false);

    platoonProtocolIn msg;

    msg.payload = os.str();

      while (true)
      {
          sleep(1);
          pub_platoonProtocolIn.publish(msg);
          std::cout << "moduletest wrote a thing" << std::endl;

      }



  }


} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_protocol, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
