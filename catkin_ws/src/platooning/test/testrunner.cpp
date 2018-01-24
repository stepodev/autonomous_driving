/**
 * @file /platooning/test/testrunner.cpp
 *
 * @brief runs all registered tests
 *
 * @author stepo
 **/

/*****************************************************************************
** Includes
*****************************************************************************/
// %Tag(FULLTEXT)%
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <fstream>
#include <chrono>
#include <boost/thread/thread.hpp>

#include "platooning/runTestCommand.h"
#include "platooning/testResult.h"
#include "platooning/registerTestcases.h"

using namespace std;

namespace platooning {

  class Testrunner : public nodelet::Nodelet {

  public:

    /*****************************************************************************
    ** Constructors
    *****************************************************************************/

    Testrunner() {};

    /*****************************************************************************
    ** Destructors
    *****************************************************************************/

    ~Testrunner() {
      myfile.close();
    };


    /*****************************************************************************
    ** Initializers
    *****************************************************************************/

    /**
    * Set-up necessary publishers/subscribers
    * @return true, if successful
    */
    void onInit() {

      myfile.open("test1.txt");

      ros::NodeHandle nh_;

      sub_testResult = nh_.subscribe("testResult", 10,
                                     &Testrunner::hndl_testResult, this);

      sub_registerTestcases = nh_.subscribe("registerTestcases", 10 ,
                                            &Testrunner::hndl_registerTestcases, this );

      pub_testRunCommand = nh_.advertise<platooning::runTestCommand>("runTestCommand", 10);

      boost::thread t([this]() {
        while(sub_registerTestcases.getNumPublishers() == 0 && testsToRunlist.empty()) {
          std::stringstream ss;
          ss << "[testrunner] waiting for registers). have testcases " << testsToRunlist.size();
          NODELET_INFO( ss.str().c_str() );
          boost::this_thread::sleep_for(boost::chrono::seconds(1));
        }

        while(sub_registerTestcases.getNumPublishers() != 0 ) {
          NODELET_INFO("[testrunner] waiting for all testcases to be published)");
          boost::this_thread::sleep_for(boost::chrono::seconds(1));
        }

        if( !testsToRunlist.empty() ) {
          runthread = unique_ptr<boost::thread>( new boost::thread(boost::bind(&Testrunner::run, this)));
          NODELET_INFO("[testrunner] started running tests");
        } else {
          NODELET_FATAL("[testrunner] no testcases have been registered");
        }

      });

      NODELET_INFO("[testrunner] init done");

    }

  private:

    /*****************************************************************************
    ** Variables
    *****************************************************************************/

    ofstream myfile = std::ofstream();
    bool keepSpinning = true;
    list<string> testsToRunlist;
    ros::Subscriber sub_testResult;
    ros::Publisher pub_testRunCommand;
    unique_ptr<boost::thread> runthread;
    ros::Subscriber sub_registerTestcases;

    /*****************************************************************************
    ** Handlers
    *****************************************************************************/


    void hndl_testResult(platooning::testResult msg) {

      myfile << testsToRunlist.front() << msg.success << " " << msg.comment << std::endl;

      testsToRunlist.pop_front();

      if (testsToRunlist.empty()) {
        keepSpinning = false;
      }
    }

    /*****************************************************************************
    ** Helpers
    *****************************************************************************/

    void hndl_registerTestcases( platooning::registerTestcases msg) {

      NODELET_INFO(std::string("[testrunner] registered testcase " + msg.testcase).c_str());

      testsToRunlist.emplace_back(msg.testcase);

    }

    void run() {

      while (!testsToRunlist.empty()) {

        platooning::runTestCommand runcmd;
        runcmd.testToRun = testsToRunlist.front();

        while (pub_testRunCommand.getNumSubscribers() == 0) {
          NODELET_WARN("found no subscribers");
          boost::this_thread::sleep_for(boost::chrono::seconds(5));
        }

        pub_testRunCommand.publish(runcmd);


        while (keepSpinning) {
          boost::this_thread::sleep_for(boost::chrono::seconds(1));
          NODELET_WARN("[testrunner] waiting for test to finish");
        }
      }

      NODELET_INFO("[testrunner] done");

    }


  }; // namespace platooning
}


PLUGINLIB_EXPORT_CLASS(platooning::Testrunner, nodelet::Nodelet);
// %EndTag(FULLTEXT)%