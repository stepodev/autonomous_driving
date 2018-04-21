/**
 * @file doxygen_c.h
 * @author My Self
 * @date 9 Sep 2012
 * @brief File containing example of doxygen usage for quick reference.
 *
 * Here typically goes a more extensive explanation of what the header
 * defines. Doxygens tags are words preceeded by either a backslash @\
 * or by an at symbol @@.
 * @see http://www.stack.nl/~dimitri/doxygen/docblocks.html
 * @see http://www.stack.nl/~dimitri/doxygen/commands.html
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MODULETEST_PLATOONING_HPP
#define PLATOONING_MODULETEST_PLATOONING_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/thread.hpp>

#include "Moduletest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"
#include "platooning/Services.hpp"
#include "platooning/ServiceTypes.hpp"

namespace platooning {

/**
 * @brief Example showing how to document a function with Doxygen.
 *
 * Description of what the function does. This part may refer to the parameters
 * of the function, like @p param1 or @p param2. A word of code can also be
 * inserted like @c this which is equivalent to <tt>this</tt> and can be useful
 * to say that the function returns a @c void or an @c int. If you want to have
 * more than one word in typewriter font, then just use @<tt@>.
 * We can also include text verbatim,
 * @verbatim like this@endverbatim
 * Sometimes it is also convenient to include an example of usage:
 * @code
 * BoxStruct *out = Box_The_Function_Name(param1, param2);
 * printf("something...\n");
 * @endcode
 * Or,
 * @code{.py}
 * pyval = python_func(arg1, arg2)
 * print pyval
 * @endcode
 * when the language is not the one used in the current source file (but
 * <b>be careful</b> as this may be supported only by recent versions
 * of Doxygen). By the way, <b>this is how you write bold text</b> or,
 * if it is just one word, then you can just do @b this.
 * @param param1 Description of the first parameter of the function.
 * @param param2 The second one, which follows @p param1.
 * @return Describe what the function returns.
 * @see Box_The_Second_Function
 * @see Box_The_Last_One
 * @see http://website/
 * @note Something to note.
 * @warning Warning.
 */

class Moduletest_platooning : public Moduletest {
  public:
	void onInit();

	Moduletest_platooning();

	~Moduletest_platooning();

  private:
    uint32_t vehicle_id_;
    boost::thread_group thread_pool_;

	/**
	* @brief template_testcase does x,y,z and expects a,b,c
	*/
	void pub_templatemsg_recv_othermsg();

	void hndl_recv_othermsg(const platooning::templateMsg &msg);

	/**
	 * @brief send platooningToggle and expect platooningState to be CREATING
	 */
	platooningState platooningstate_;
	void send_platoontoggle_recv_platoonstate_creating();
	void hndl_testcase_send_platoontoggle_recv_platoonstate_creating(platooningState msg);
    void hndl_test_send_platoontoggle_recv_platoonstate_creating_timeout();

	void send_platoontoggle_recv_error_lv();

	//test timeouts
	void send_heartbeats_dont_recv_platoonstate_timeout();
	void send_broadcast_dont_recv_platoonstate_timeout();
	void lv_broadcast_timeout_recv_platoonstate_idle();
	void fv_heartbeat_timeout_recv_platoonstate_without_member_timedout();
	void fv_heartbeat_timeout_recv_platoonstate_idle();

	//test join
	void send_fv_request_recv_lv_accept();
	void hndl_tc_send_fv_request_recv_lv_accept( lv_accept msg );

	void send_fv_request_recv_platoonstate_running();
	void send_fv_request_recv_broadcast();

	//test accept
	void send_lv_accept_recv_heartbeat();

	//send to wrong role
	void send_broadcast_to_lv_recv_nothing();
	void send_heartbeat_to_fv_recv_nothing();

	//send to idle
	void send_heartbeat_to_idle_recv_nothing();
	void send_fv_request_to_idle_recv_nothing();
	void send_fv_leave_to_idle_recv_nothing();
	void send_fv_heartbeat_to_idle_recv_nothing();

	void send_lv_accept_to_idle_recv_nothing();
	void send_lv_reject_to_idle_recv_nothing();
	void send_lv_broadcast_to_idle_recv_nothing();

	//send wrong source
	void send_fv_leave_non_member_recv_nothing();

	//send duplicates
	void send_fv_request_receive_two_lv_accept();

};

} // namespace platooning

#endif //PLATOONING_MODULETEST_PLATOONING_HPP
