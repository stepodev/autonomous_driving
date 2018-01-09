//
// Created by stepo on 1/9/18.
//

#include <gtest/gtest.h>

TEST(templateTests, moduleTest1) {
  ASSERT_NE(1,2);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}