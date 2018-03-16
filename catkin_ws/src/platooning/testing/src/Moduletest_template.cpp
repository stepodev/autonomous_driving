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

#include "platooning/Moduletest_template.hpp"

// %Tag(FULLTEXT)%
namespace platooning {

/*****************************************************************************
** Constructors
*****************************************************************************/


/**
 * @brief Template Nodelet
 */

Moduletest_template::Moduletest_template() = default;

/*****************************************************************************
** Destructors
*****************************************************************************/

Moduletest_template::~Moduletest_template() = default;


/*****************************************************************************
** Initializers
*****************************************************************************/

/**
* Set-up necessary publishers/subscribers
* @return true, if successful
*/
void Moduletest_template::onInit() {

	name_ = "Moduletest_template";

	register_testcases(boost::bind(&Moduletest_template::pub_templatemsg_recv_othermsg, this));

	NODELET_INFO("[%s ] init done", name_.c_str());

	start_tests();
}

/*****************************************************************************
** Testcases
*****************************************************************************/

void Moduletest_template::pub_templatemsg_recv_othermsg() {

	set_current_test("pub_templatemsg_recv_othermsg");
	NODELET_INFO(std::string("[" + name_ + "] started testcase " + get_current_test()).c_str());

	//mockup publishers
	pub_map_.clear();
	pub_map_.emplace(topics::TEMPLATETOPIC, ros::Publisher());
	pub_map_[topics::TEMPLATETOPIC] = nh_.advertise<templateMsg>(topics::TEMPLATETOPIC, 1);

	//mockup subscribers
	sub_map_.clear();
	sub_map_.emplace(topics::TEMPLATETOPIC, ros::Subscriber());
	sub_map_[topics::TEMPLATETOPIC] = nh_.subscribe(topics::TEMPLATETOPIC, 1,
	                                                &Moduletest_template::hndl_recv_othermsg,
	                                                this);

	boost::shared_ptr<templateMsg> inmsg = boost::shared_ptr<templateMsg>(new templateMsg);

	inmsg->templatebool = true;

	//wait for nodelet to subscribe to topic
	while(pub_map_[topics::TEMPLATETOPIC].getNumSubscribers() < 1 ) {
		boost::this_thread::sleep_for( boost::chrono::milliseconds(200));
	}

	pub_map_[topics::TEMPLATETOPIC].publish(inmsg);

}

void Moduletest_template::hndl_recv_othermsg(const platooning::templateMsg &msg) {

	TestResult res;
	res.success = true;

	if (msg.templatebool != true) {
		res.success = false;
		res.comment = "this is a useful comment what happened and why";
	}

	if (!res.success) {
		NODELET_ERROR("[%s] error with %s ", name_.c_str(), res.comment.c_str());
	}

	finalize_test(res);

}

} // namespace platooning

PLUGINLIB_EXPORT_CLASS(platooning::Moduletest_template, nodelet::Nodelet);
// %EndTag(FULLTEXT)%
