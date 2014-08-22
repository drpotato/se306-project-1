#include "ActorSpawner.h"
#include "Actor.h"
#include "R0.h"
#include "R1.h"
#include "Resident.h"
#include "Relative.h"
#include "EntertainmentRobot.h"

#include <cstdio>
#include <cstring>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

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

// TODO: UPDATE THIS. CAN'T JUST SPAWN NEW NODES ############################################################################################################
Actor *ActorSpawner::spawnActor(const char *actorTypeName)
{
  if (strcmp("R0", actorTypeName) == 0)
  {
    return new R0();
  }

  if (strcmp("R1", actorTypeName) == 0)
  {
    return new R1();
  }

  if (strcmp("Resident", actorTypeName) == 0)
  {
    return new Resident();
  }

  if (strcmp("Relative", actorTypeName) == 0)
  {
    return new Relative();
  }

  if (strcmp("EntertainmentRobot", actorTypeName) == 0)
  {
    return new EntertainmentRobot();
  }

  // Uh oh, we didn't match anything ... return an R0?
  return new R0();
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
  while (actor->executeLoop());

  return 0;
}