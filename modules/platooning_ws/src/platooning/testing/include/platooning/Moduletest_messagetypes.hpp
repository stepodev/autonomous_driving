/**
 * @file testing/include/platooning/Moduletest_messagetypes.hpp
 * @author stepo
 * @date 24.03.2018
 * @brief Header for messagetype moduletest
 *
 */

/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_MODULETEST_MESSAGETYPES_HPP
#define PLATOONING_MODULETEST_MESSAGETYPES_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "Moduletest.hpp"
#include "platooning/Topics.hpp"
#include "platooning/MessageTypes.hpp"

namespace platooning {

/**
 * @brief Nodlet running moduletests for messagetypes
 *
 */

class Moduletest_messagetypes : public Moduletest {
  public:
	void onInit();

	Moduletest_messagetypes();

	~Moduletest_messagetypes();

  private:
	/**
	 * @brief creates messages and tests for decode(encode(message)) == message
	 */
	void test_encode_decode_messages();

};

} // namespace platooning

#endif //PLATOONING_MODULETEST_MESSAGETYPES_HPP
