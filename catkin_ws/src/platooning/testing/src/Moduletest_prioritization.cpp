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

#include "platooning/Moduletest_prioritization.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Moduletest_prioritization::Moduletest_prioritization() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_prioritization::~Moduletest_prioritization() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_prioritization::onInit() {

	name_ = "Moduletest_prioritization";

	register_testcases(boost::bind(&Moduletest_prioritization::pub_platooningState_recv_targetSpeed, this));

	NODELET_INFO(std::string("[" + name_ + "] init done").c_str());

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_prioritization::pub_platooningState_recv_targetSpeed() {

	set_current_test("pub_platooningState_recv_oughtData");
	NODELET_INFO(std::string("[" + name_ + "] started testcase " + get_current_test()).c_str());

	//mockup publishers
	pub_map_.clear();
	pub_map_.emplace(topics::PLATOONINGSTATE, ros::Publisher());
	pub_map_[topics::PLATOONINGSTATE] = nh_.advertise<templateMsg>(topics::PLATOONINGSTATE, 1);

	//mockup subscribers
	sub_map_.clear();
	sub_map_.emplace(topics::TARGET_SPEED, ros::Subscriber());
	sub_map_[topics::TARGET_SPEED] = nh_.subscribe(topics::TARGET_SPEED, 1,
	                                            &Moduletest_prioritization::hndl_recv_targetSpeed,
	                                            this);

	boost::shared_ptr<platooningState> inmsg = boost::shared_ptr<platooningState>(new platooningState);

	inmsg->ipd = 3;
	inmsg->ps = 4;

	pub_map_[topics::PLATOONINGSTATE].publish(inmsg);

}

void Moduletest_prioritization::hndl_recv_targetSpeed(platooning::targetSpeed msg) {

	TestResult res;
	res.success = true;

	if ( msg.target_speed != 4) {
		res.success = false;
		res.comment = "this is a useful comment what happened and why";
	}

	if (!res.success) {
		NODELET_ERROR((std::string("[") + name_ + "] error with " + res.comment).c_str());
	}

	finalize_test(res);

}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_prioritization, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
