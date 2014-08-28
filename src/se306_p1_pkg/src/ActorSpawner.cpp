#include <cstdio>
#include <cstring>
#include "PathPlannerListener.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>



// Load the actor subclasses
#define SE306P1___AUTOGEN__ACTORLIST___INCLUDE
#include "__autogen__actorlist__"
#undef SE306P1___AUTOGEN__ACTORLIST___INCLUDE

//TODO: Update this comment?
// A 'Factory' class to control the arrival and departure of Visitors.
// As we cannot dynamically create and remove Actors from the world, they will simply sit (or mill about/circle) outside the door of the house until needed.
// When the Resident requests a Visitor, the ActorController notifies that Visitor.
// When the Visitor has finished its task, it notifies the ActorController that it is leaving the house.
ActorSpawner::ActorSpawner():
theActor(0)
{
}

//ActorController should act as a Singleton.
ActorSpawner &ActorSpawner::getInstance()
{
  static ActorSpawner *actorSpawnerInstance;

  if (!actorSpawnerInstance)
  {
    actorSpawnerInstance = new ActorSpawner();
  }

  return *actorSpawnerInstance;
}

Actor *ActorSpawner::spawnActor(const char *actorTypeName)
{
  // Spawn the actor type
  #define SE306P1___AUTOGEN__ACTORLIST___SPAWN
  #include "__autogen__actorlist__"
  #undef SE306P1___AUTOGEN__ACTORLIST___SPAWN

  // Uh oh, we didn't match anything. Return a null pointer so it blows up immediately rather than working for a bit
  return 0;
}

Actor *ActorSpawner::getActor(const char *actorTypeName)
{
  if (theActor == 0)
    theActor = spawnActor(actorTypeName);
  return theActor;
}

Actor *ActorSpawner::getActor() const
{
  return theActor;
}

int main(int argc, char **argv)
{
  // This needs to be run as ActorSpawner <robotID> <robotType> <x> <y> <angle>
  if (argc < 6) return 0;

  ActorSpawner &spawner = ActorSpawner::getInstance();
  Actor *actor = spawner.getActor(argv[2]);
  unsigned int robotID;
  double px, py, theta;
  std::sscanf(argv[1], "%u", &robotID);
  std::sscanf(argv[3], "%lf", &px);
  std::sscanf(argv[4], "%lf", &py);
  std::sscanf(argv[5], "%lf", &theta);

  actor->initialSetup(robotID, px, py, theta);
  PathPlannerListener();
  while (actor->executeLoop());

  return 0;
}
