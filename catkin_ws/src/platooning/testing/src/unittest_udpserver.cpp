/**
 * @file unittest_template.cpp
 * @author My Self
 * @date 9 Sep 2012
 * @brief unittests for template nodelet
 *
 */

#include <gtest/gtest.h>
#include <boost/asio.hpp>
#include "platooning/UdpServer.hpp"

class UdpServerTester : public UdpServer {
 public:
  UdpServerTester() = delete;

  UdpServerTester( boost::function<void(std::pair<std::string,int32_t>)> cbfun
                , udp::endpoint bind
                , udp::endpoint target )
  : UdpServer( cbfun, bind, target) {};

  std::pair<std::string, int32_t> Convert( std::string msg, int32_t type ) {
	  this->write_to_sendbuffer(<#initializer#>, msg, type);
    recv_buffer_ = send_buffer_;
    return read_from_recvbuffer(<#initializer#>, 0);
  };
};

/**
 * @brief tests X
 *
 * Detailed explanation.
 */
TEST(UdpServer, MessageConversionTest) {

boost::function<void(std::pair<std::string,int32_t>)> cbfun;

  std::shared_ptr<UdpServerTester> server = std::shared_ptr<UdpServerTester>( new UdpServerTester(
        cbfun
        , udp::endpoint(udp::v4(),10001)
        , udp::endpoint(boost::asio::ip::address_v4::broadcast(),10000)));

  std::string message = "thisisateststringplstestme";
  int32_t message_type = 0x00112233;

  std::pair<std::string, int32_t> testout = server->Convert(message,message_type);

  ASSERT_EQ( testout.first, message);
  ASSERT_EQ( testout.second, message_type);
}

/**
 * @brief test main function for unittest template
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
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}