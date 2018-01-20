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
#include "platooning/runTestCommand.h"
#include "platooning/testResult.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <boost/thread/thread.hpp>


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

      pub_testRunCommand = nh_.advertise<platooning::runTestCommand>("runTestCommand", 10);

      registerTestcases();

      runthread = unique_ptr<boost::thread>( new boost::thread(boost::bind(&Testrunner::run, this)));

      NODELET_INFO("testrunner init done");

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

    void registerTestcases() {
      testsToRunlist = list<string>();

      testsToRunlist.emplace_back("moduleTest_platooning_leaderrequest");
    }

    void run() {

      cout << "runnin" << keepSpinning << std::endl;

      while (!testsToRunlist.empty()) {

        platooning::runTestCommand runcmd;
        runcmd.testToRun = testsToRunlist.front();

        while (pub_testRunCommand.getNumSubscribers() == 0) {
          NODELET_WARN("found no subscribers");
          this_thread::sleep_for(std::chrono::seconds(5));
        }

        pub_testRunCommand.publish(runcmd);


        while (keepSpinning) {
          this_thread::sleep_for(std::chrono::seconds(1));
          NODELET_WARN("Spinnin");
        }
      }

      NODELET_INFO("TESTRUNNER done");

    }
  }; // namespace platooning
}


PLUGINLIB_EXPORT_CLASS(platooning::Testrunner, nodelet::Nodelet);
// %EndTag(FULLTEXT)%