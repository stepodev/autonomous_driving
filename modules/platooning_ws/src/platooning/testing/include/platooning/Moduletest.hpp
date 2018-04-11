/**
 * @file testing/include/platooning/Moduletest.hpp
 * @author stepo
 * @date 22.03.2018
 *
 */

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

/***
 * @class TestResult
 * @author stepo
 *
 * @brief Describes a testresult
 */
class TestResult {
  public:
	bool success = false;
	std::string comment = "";
};

/***
 * @class Moduletest
 * @author stepo
 *
 * @brief Provides basic testing facilites.
 *
 * Provides functionality that every moduletest requires, such as a timeout, logging and a test finalization.
 */
class Moduletest : public nodelet::Nodelet {

  public:
	Moduletest();

	~Moduletest() override;

  protected:
	std::string name_ = "Moduletest";
	std::string test_result_filepath_ = "test1.txt";
	ros::NodeHandle nh_;

	/**< members for timeouts */
	boost::shared_ptr<boost::asio::io_service::work> work_;
	boost::asio::io_service io_;
	boost::asio::deadline_timer testcase_timer_;
	boost::posix_time::time_duration timeout_;
	boost::thread_group threadpool_;

	/**< maps to handle establishment and cleanup of publishers and subscribers */
	std::map<std::string, ros::Publisher> pub_map_;
	std::map<std::string, ros::Subscriber> sub_map_;

	/**< list of callbacks for each testcase to rn*/
	std::list<boost::function<void()>> testcases_to_run_;

	/**< timeout callback if any */
	boost::function<void()> timeout_callback_;

	/**
	 * @brief sets the logging filepath
	 * @param str path to file
	 */
	void set_result_filepath(std::string str) { test_result_filepath_ = std::move(str); }

	/**
	 * @brief sets the current testname for logging purposes
	 * @param str name of test
	 */
	void set_current_test(std::string str);

	/**
	 * @brief gets the current name of test for logging purposes
	 * @return name of testcase
	 */
	std::string get_current_test() { return current_test_; }

	/**
	 * @brief sets the timeout before the testcase is considered a fail
	 * @param s time duration in seconds
	 */
	void set_timeout(const boost::posix_time::time_duration &s);

	/**
	 * @brief called to register testcase methods
	 * @param test_case_fun a function object of the testmethod
	 */
	void register_testcases(boost::function<void()> test_case_fun);

	/**
	 * @brief Method that finalizes the current test.
	 *
	 * Does cleanup tasks, such as stoping the timeout timer, clearing the subscriber and publishers
	 * and logging the result
	 * @param result TestResult object containing the testresult
	 */
	void finalize_test(TestResult result);

	/**
	 * @brief testcase timeout handler.
	 *
	 * Calls finalizer.
	 *
	 * @param e errorcode of reasony why handler was called
	 */
	void hndl_testcase_timeout(const boost::system::error_code &e);

	/**
	 * @brief starts the next test or shuts down nodelet if all tests have run
	 */
	void start_tests();

	/**
	 * @brief registeres a callback to call in case of timeout
	 *
	 * Testcases may expect a timeout to be a succesful test. The requires the default testcase timeout behavior to
	 * be overriden.
	 *
	 * @param cb callback function to call
	 */
	void register_timeout_callback(boost::function<void()> cb);

  private:
	std::string current_test_;
};

#endif //PLATOONING_MODULETEST_HPP
