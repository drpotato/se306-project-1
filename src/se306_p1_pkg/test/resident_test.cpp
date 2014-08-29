// A hack so that I can access private members
#define private public
#define protected public

// Bring in my package's API, which is what I'm testing
#include "../src/Resident.h"
#include <msg_pkg/Interaction.h>
#include "../src/Actor.h"

// Bring in gtest
#include <gtest/gtest.h>

// Test the constructor initialises variables correctly
TEST(ResidentTestSuite, test_resident_contructor)
{
	Resident *resident = new Resident();
	resident->doInitialSetup();
	ASSERT_EQ(resident->has_eaten_breakfast_, false);
	ASSERT_EQ(resident->has_eaten_lunch_, false);
	ASSERT_EQ(resident->has_eaten_dinner_, false);
	ASSERT_EQ(resident->getOutOfBed, false);
	ASSERT_EQ(resident->goToEatingPlace, false);
	ASSERT_EQ(resident->getIntoBed, false);
	ASSERT_EQ(resident->lock_, false);
	ASSERT_EQ(resident->velLinear, 0);
	ASSERT_EQ(resident->velRotational, 0.0);
	ASSERT_EQ(resident->morale_level_, LEVEL_MAX);
	ASSERT_EQ(resident->socialness_level_, LEVEL_MAX);
	ASSERT_EQ(resident->health_level_, LEVEL_MAX);
	ASSERT_EQ(resident->hygiene_level_, LEVEL_MAX);
	ASSERT_EQ(resident->hunger_level_, LEVEL_MAX);
	ASSERT_EQ(resident->fitness_level_, LEVEL_MAX);
}

// Test the actor enum type to string conversion function
TEST(ResidentTestSuite, get_string_from_actor_type)
{
	Resident *resident = new Resident();
	ASSERT_EQ(resident->getStringFromActorType(Resident::Doctor), "Doctor");
	ASSERT_EQ(resident->getStringFromActorType(Resident::Nurse), "Nurse");
	ASSERT_EQ(resident->getStringFromActorType(Resident::Caregiver), "Caregiver");
	ASSERT_EQ(resident->getStringFromActorType(Resident::Visitor), "Visitor");
	ASSERT_EQ(resident->getStringFromActorType(Resident::Robot), "Robot");
}

// Test the actor enum type from string conversion function
TEST(ResidentTestSuite, get_actor_type_from_string)
{
	Resident *resident = new Resident();
	ASSERT_EQ(resident->getActorTypeFromString("Doctor"), Resident::Doctor);
	ASSERT_EQ(resident->getActorTypeFromString("Nurse"), Resident::Nurse);
	ASSERT_EQ(resident->getActorTypeFromString("Caregiver"), Resident::Caregiver);
	ASSERT_EQ(resident->getActorTypeFromString("Visitor"), Resident::Visitor);
	ASSERT_EQ(resident->getActorTypeFromString("Robot"), Resident::Robot);
}


// Test adding a positive amount to a level 
// Expected output: either max_level or the first value plus 10
TEST(ResidentTestSuite, test_interaction_add_positive_level)
{
	Resident *resident = new Resident();
	resident->doInitialSetup();
	int oldVal = resident->socialness_level_;
	msg_pkg::Interaction *msg = new msg_pkg::Interaction();
	msg->attribute = "socialising";
	msg->amount = 10;
	resident->interactionCallback(*msg);

	int expectedLevel = std::min(LEVEL_MAX, (oldVal + 10));
	
	ASSERT_EQ(expectedLevel, resident->socialness_level_);
}

// Test adding a negative amount to a level 
// Expected output: either min_level or the first value minus 10
TEST(ResidentTestSuite, test_interaction_add_negative_level)
{
	Resident *resident = new Resident();
	resident->doInitialSetup();
	int oldVal = resident->hunger_level_;
	msg_pkg::Interaction *msg = new msg_pkg::Interaction();
	msg->attribute = "eating";
	msg->amount = -10;
	resident->interactionCallback(*msg);

	int expectedLevel = std::max(LEVEL_MIN, (oldVal - 10));
	
	ASSERT_EQ(expectedLevel, resident->hunger_level_);
}
//Test adding a level of zero to a level
//Expected output: level doesn't change
TEST(ResidentTestSuite, test_interaction_msg_add_zero)
{
	Resident *resident = new Resident();
	resident->doInitialSetup();
	int oldVal = resident->hygiene_level_;
	msg_pkg::Interaction *msg = new msg_pkg::Interaction();
	msg->attribute = "showering";
	msg->amount = 0;
	resident->interactionCallback(*msg);

	int expectedLevel = std::max(LEVEL_MIN, oldVal);
	
	ASSERT_EQ(expectedLevel, resident->hygiene_level_);
}