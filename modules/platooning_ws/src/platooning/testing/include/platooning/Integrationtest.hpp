/**
 * @file testing/src/Integrationtest.hpp
 * @author stepo
 * @date 22,03,2018
 * @brief Contains header of Integrationtest class
 *
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef PLATOONING_INTEGRATIONTEST_HPP
#define PLATOONING_INTEGRATIONTEST_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <algorithm>

#include "Moduletest.hpp"

namespace platooning {

/**
 * @class Integrationtest
 *
 * @brief Integrationtest class.
 *
 */

class Integrationtest : public Moduletest {
  public:

	Integrationtest();

	~Integrationtest() override;

};

} // namespace platooning

#endif //PLATOONING_INTEGRATIONTEST_HPP
