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

TEST(TestSuite, test_resident_get_new_level)
{
Resident *resident = new Resident();
int result = resident->getNewLevel(0, 0);
ASSERT_EQ(result, 1);
result = resident->getNewLevel(0, 1);
ASSERT_EQ(result, 1);
result = resident->getNewLevel(0, 2);
ASSERT_EQ(result, 2);
result = resident->getNewLevel(0, 3);
ASSERT_EQ(result, 3);
result = resident->getNewLevel(0, 4);
ASSERT_EQ(result, 4);
result = resident->getNewLevel(0, 5);
ASSERT_EQ(result, 5);
result = resident->getNewLevel(0, 6);
ASSERT_EQ(result, 5);
result = resident->getNewLevel(-99, 6);
ASSERT_EQ(result, 1);
}


TEST(TestSuite, test_interaction_msg)
{
msg_pkg::Interaction *msg = new msg_pkg::Interaction();
}
