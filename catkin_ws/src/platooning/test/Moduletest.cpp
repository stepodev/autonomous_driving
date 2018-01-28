//
// Created by stepo on 1/22/18.
//
#include "Moduletest.hpp"

void Moduletest::register_testcases(std::list<std::string> testcases_to_register) {

  if( testcases_to_register.empty() ) {
    NODELET_FATAL(std::string("[" + name_ + "] no testcases to publish").c_str());
    return;
  }

  std::unique_ptr<boost::thread> registerthread ( new boost::thread([this, testcases_to_register] {
    ros::Publisher pub = this->nh_.advertise<platooning::registerTestcases>("registerTestcases", 10);

    int cntr = 0;
    while (pub.getNumSubscribers() == 0 && cntr++ < 10) {
      NODELET_INFO(std::string("[" + name_ + "] no subscribers found. waiting.").c_str());
      boost::this_thread::sleep_for(boost::chrono::seconds(1));
    }

    if (pub.getNumSubscribers() == 0) {
      NODELET_FATAL(std::string("[" + name_ + "] couldnt publish testcases").c_str());
      return;
    }

    if(pub.getNumSubscribers() != 0 ) {
      NODELET_INFO(std::string("[" + name_ + "] subscribers found. publishing.").c_str());

      for (auto &x : testcases_to_register) {
        boost::shared_ptr<platooning::registerTestcases> msgptr = boost::shared_ptr<platooning::registerTestcases>(
            new platooning::registerTestcases());

        msgptr->testcase = x;

        pub.publish(msgptr);
      }
      pub.shutdown();

      NODELET_INFO(std::string("[" + name_ + "] stopped publishing testcases.").c_str());
      return;
    }

    NODELET_FATAL(std::string("[" + name_ + "] shouldnt be here").c_str());


  }));

}

