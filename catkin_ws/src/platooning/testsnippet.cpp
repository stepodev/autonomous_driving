//
// Created by stepo on 1/14/18.
//

#include <iostream>
#include <boost/thread/thread.hpp>
#include "testsnippet.hpp"
#include <thread>

int main(int argc, char **argv){

  boost::thread* roscorethread =
      new boost::thread([]{ sleep(10000); std::cout << "done" << std::endl;});

  std::this_thread::yield();

  std::cout << "hello" << std::endl;


}