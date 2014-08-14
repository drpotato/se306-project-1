// Bring in my package's API, which is what I'm testing
#include "../src/Resident.h"
#include "../src//ActorSpawner.h"
#include "msg_pkg/Interaction.h"
// Bring in gtest
#include <gtest/gtest.h>

// Declare a test
TEST(ResidentPublish, testCanChangeEntertainedness)
{
    Resident *resident = new Resident();
    resident->setEntertainedness(10);
    ASSERT_EQ(resident->entertainedness_level_, 10);
}

// Declare another test
TEST(ResidentPublish, testInteractionCallback)
{
    Resident *resident = new Resident();
    msg_pkg::Interaction *message = new msg_pkg::Interaction();
    message->attribute="socialness";
    message->amount=1;
    Resident->interactionCallback(message);
    ASSERT_EQ(resident->socialness_level_, 6);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}