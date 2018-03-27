//
// Created by stepo on 1/22/18.
//
#include "platooning/Moduletest.hpp"

Moduletest::Moduletest() :
	testcase_timer_(io_) {
	timeout_ = boost::posix_time::seconds(3);
	timeout_callback_ = boost::function<void()>();

	work_ = boost::shared_ptr<boost::asio::io_service::work>( new boost::asio::io_service::work(io_));

	threadpool_.create_thread( [this] { io_.run(); });
}

Moduletest::~Moduletest() {
	try {
		NODELET_WARN("[%s] killing %i threads", name_.c_str(), (int) threadpool_.size());

		io_.reset();
		io_.stop();

		threadpool_.join_all();
	} catch (std::exception &ex) {
		NODELET_FATAL("[%s] threw %s", name_.c_str(), ex.what());
	}

	expects_timeout_ = false;

}

void Moduletest::register_testcases(boost::function<void()> test_case_fun) {
	testcases_to_run_.emplace_back(test_case_fun);
}

void Moduletest::register_timeout_callback(boost::function<void()> cb) {
	timeout_callback_ = std::move(cb);
	expects_timeout_ = true;
}

void Moduletest::finalize_test(TestResult result) {

	try {
		testcase_timer_.cancel();

		std::stringstream ss;
		std::ofstream of;
		of.open(test_result_filepath_, std::ios::app);

		std::time_t t = std::time(nullptr);
		std::put_time(std::localtime(&t), "%c %Z");
		ss << "[" << name_ << "]["
		   << current_test_ << "]["
		   << (result.success ? "SUCCESS" : "FAILURE") << "] "
		   << result.comment << std::endl;

		if (!result.success) {
			NODELET_ERROR("%s", ss.str().c_str());
		} else {
			NODELET_WARN("%s", ss.str().c_str());
		}

		of << "[" << t << "]" << ss.str();

		of.close();

		expects_timeout_ = false;

		pub_map_.clear();
		sub_map_.clear();

		timeout_callback_.clear();

		set_timeout(boost::posix_time::seconds(3));

		start_tests();

	} catch (std::exception &ex) {
		NODELET_FATAL("[%s] threw %s", name_.c_str(), ex.what());
	}
}

void Moduletest::hndl_testcase_timeout(const boost::system::error_code &ec) {

	try {
		if (ec == boost::asio::error::operation_aborted) {
			return;
		}

		if (ec == boost::system::errc::success && timeout_callback_.empty()) {
			TestResult res;
			res.success = false;
			res.comment = "testcase timeout";

			finalize_test(res);
			return;
		}

		if (ec == boost::system::errc::success && !timeout_callback_.empty()) {

			timeout_callback_();

			TestResult res;
			res.success = true;
			res.comment = "expected testcase timeout";

			timeout_callback_ = boost::function<void()>();
			finalize_test(res);
			return;
		}

	} catch (std::exception &ex) {
		NODELET_FATAL("[%s] threw %s", name_.c_str(), ex.what());
	}

	NODELET_FATAL("[%s] we shouldnt be here. testcase timeout not caught. boost error was %s",
	              name_.c_str(), ec.message().c_str());
	ros::shutdown();

}

void Moduletest::start_tests() {

	try {
		if (testcases_to_run_.empty()) {
			NODELET_WARN("[%s] no more testcases. stopping", name_.c_str());
			ros::shutdown();
			return;
		}

		boost::function<void()> test_case_fun = testcases_to_run_.front();
		testcases_to_run_.pop_front();

		threadpool_.create_thread([test_case_fun] {
			test_case_fun();
		});

		testcase_timer_.expires_from_now(timeout_);
		testcase_timer_.async_wait(boost::bind(&Moduletest::hndl_testcase_timeout, this,
		                                       boost::asio::placeholders::error));
	} catch (std::exception &ex) {
		NODELET_FATAL("[%s] threw %s", name_.c_str(), ex.what());
	}
}
void Moduletest::set_current_test(std::string str) {
	current_test_ = std::move(str);
	NODELET_WARN("[%s] %s started", name_.c_str(), current_test_.c_str());
}

void Moduletest::set_timeout(const boost::posix_time::time_duration &s) {
	timeout_ = s;

	testcase_timer_.cancel();

	testcase_timer_.expires_from_now(timeout_);
	testcase_timer_.async_wait(boost::bind(&Moduletest::hndl_testcase_timeout, this,
	                                       boost::asio::placeholders::error));
}

