// Bring in my package's API, which is what I'm testing
#include "../src/Actor.h"
#include "../src/Robot.h"
#include <msg_pkg/Interaction.h>
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, test_actor_contructor)
{
Robot *robot = new Robot();
}

TEST(TestSuite, test_interaction_msg)
{
msg_pkg::Interaction *msg = new msg_pkg::Interaction();
}
