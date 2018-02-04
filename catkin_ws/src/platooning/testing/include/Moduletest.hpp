//
// Created by stepo on 1/22/18.
//

#ifndef PLATOONING_MODULETEST_HPP
#define PLATOONING_MODULETEST_HPP

#include <nodelet/nodelet.h>
#include <unordered_map>
#include <list>
#include <fstream>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/thread.hpp>
#include <ros/ros.h>

class TestResult {
public:
  bool success = false;
  std::string comment = "";
};

class Moduletest : public nodelet::Nodelet {

public:
  Moduletest();

  ~Moduletest();

protected:
  std::string name_;
  std::string test_result_filepath_ = "test1.txt";
  ros::NodeHandle nh_;

  boost::asio::io_service io_;
  boost::asio::deadline_timer testcase_timer_;
  boost::posix_time::time_duration timeout_;
  boost::thread_group threadpool_;

  std::map<std::string, ros::Publisher> pub_map_;
  std::map<std::string, ros::Subscriber> sub_map_;
  std::list<boost::function<void()>> testcases_to_run_;

  void set_result_filepath(std::string str) { test_result_filepath_ = std::move(str); }
  void set_current_test(std::string str) { current_test_ = std::move(str); }
  std::string get_current_test() { return current_test_; }
  void set_timeout( boost::posix_time::time_duration s) { timeout_ = s;};

  void register_testcases(boost::function<void()> test_case_fun);

  void finalize_test(TestResult result);

  void hndl_testcase_timeout( const boost::system::error_code & );

  void start_tests();

private:
  std::string current_test_;
};


#endif //PLATOONING_MODULETEST_HPP
