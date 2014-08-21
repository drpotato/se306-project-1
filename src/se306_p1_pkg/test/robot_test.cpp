// Bring in my package's API, which is what I'm testing
#include "../src/Actor.h"
#include "../src/Robot.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, test_actor_contructor)
{
Robot *robot = new Robot();
}
