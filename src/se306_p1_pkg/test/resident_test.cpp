// A hack so that I can access private members
#define private public
#define protected public

// Bring in my package's API, which is what I'm testing
#include "../src/Resident.h"
#include <msg_pkg/Interaction.h>

// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(TestSuite, test_resident_contructor)
{
Resident *resident = new Resident();
ASSERT_EQ(resident->morale_count_, 0);
ASSERT_EQ(resident->socialness_count_, 0);
}

TEST(TestSuite, test_interaction_msg)
{
msg_pkg::Interaction *msg = new msg_pkg::Interaction();
}
