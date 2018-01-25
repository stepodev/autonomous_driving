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

#ifndef PLATOONING_MODULETEST_WIFI_HPP
#define PLATOONING_MODULETEST_WIFI_HPP

/*****************************************************************************
** Includes
*****************************************************************************/

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <platooning/runTestCommand.h>
#include <platooning/testResult.h>
#include <sstream>

#include "platooning/platoonProtocolOut.h" //includes topic aka message
#include "platooning/testResult.h" //includes topic aka message
#include "UdpServer.hpp"
#include "Moduletest.hpp"

using boost::asio::ip::udp;

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

  class Moduletest_wifi : public Moduletest {
  public:
    virtual void onInit();

    Moduletest_wifi();

    ~Moduletest_wifi();

  private:
    ros::NodeHandle nh_; /**< Some documentation for the member nh_. */
    std::string name_ = "moduletest_wifi";
    std::unique_ptr<UdpServer> server_;
    boost::asio::io_service io_service_;
    std::string current_test_;
    boost::thread iothread;

    ros::Subscriber sub_platoonProtocolIn;
    ros::Subscriber sub_runTestCmd;

    ros::Publisher pub_testResult;

    /**
     * @brief to achieve X does Y
     * @param msg incoming topic message
     */
    void hndl_platoonProtocolIn(platooning::platoonProtocolIn msg);
    void hndl_runTestCmd(platooning::runTestCommand msg);

    void test_send_udp_recv_protocolIn();

    void test_send_protocolOut_recv_udp();

    void handl_udp_recvd(std::shared_ptr<std::vector<char>> msg);
  };


} // namespace platooning

#endif //PLATOONING_MODULETEST_WIFI_HPP
