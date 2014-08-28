#include <gtest/gtest.h>
#include <unistd.h>

// Bring in my package's API, which is what I'm testing
#include "../src/Caregiver.h"
#include "../src/ActorSpawner.h"


TEST_F(FooTest, testCaregiverShower)
{
  ActorSpawner &spawner = ActorSpawner::getInstance();
  Caregiver *actor = (Caregiver*)spawner.getActor("Caregiver");
  
  msg_pkg::Interaction *msg = new msg_pkg::Interaction();
  actor->interactionCallback(*msg);

  actor->initialSetup(0, -5, 3, 0);
  actor->executeLoop();
}
