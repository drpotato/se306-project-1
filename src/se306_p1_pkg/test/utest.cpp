#include <gtest/gtest.h>

// Bring in my package's API, which is what I'm testing
#include "../src/Resident.h"
#include "../src/ActorSpawner.h"

/*
 * utest.cpp is an example file of how tests are structured for our 306 project
 */

// The fixture for testing class Foo.
class FooTest : public ::testing::Test {
 protected:
  // You can remove any or all of the following functions if its body
  // is empty.

  FooTest() {
    // You can do set-up work for each test here.
  }

  virtual ~FooTest() {
    // You can do clean-up work that doesn't throw exceptions here.
  }

  // If the constructor and destructor are not enough for setting up
  // and cleaning up each test, you can define the following methods:

  virtual void SetUp() {
    // Code here will be called immediately after the constructor (right
    // before each test).
    system("source devel/setup.bash; roscore &");
  }

  virtual void TearDown() {
    // Code here will be called immediately after each test (right
    // before the destructor).
    system("killall roscore");
  }

  // Objects declared here can be used by all tests in the test case for Foo.
};

TEST(TestSuite, testResidentInteration)
{
  ActorSpawner &spawner = ActorSpawner::getInstance();
  Resident *actor = (Resident*)spawner.getActor("Resident");
  
  msg_pkg::Interaction *msg = new msg_pkg::Interaction();
  actor->interactionCallback(*msg);

  actor->initialSetup(0, -5, 3, 0);
  actor->executeLoop();
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
