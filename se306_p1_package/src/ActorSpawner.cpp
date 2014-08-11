#include "ActorSpawner.h"
#include "Actor.h"
#include "R0.h"
#include "R1.h"

#include <cstdio>
#include <cstring>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>


ActorSpawner::ActorSpawner()
{
}

ActorSpawner &ActorSpawner::getInstance()
{
	static ActorSpawner *actorSpawnerInstance;
	
	if (!actorSpawnerInstance)
	{
		actorSpawnerInstance = new ActorSpawner();
	}
	
	return *actorSpawnerInstance;
}

Actor *ActorSpawner::spawnActor(const char *actorTypeName) const
{
	if (strcmp("R0", actorTypeName) == 0)
	{
		return new R0();
	}
	
	if (strcmp("R1", actorTypeName) == 0)
	{
		return new R1();
	}
	
	// Uh oh, we didn't match anything ... return an R0?
	return new R0();
}

int main(int argc, char **argv)
{
	// This needs to be run as ActorSpawner <robotID> <robotType>
	if (argc < 3) return 0;
	
	ActorSpawner &spawner = ActorSpawner::getInstance();
	Actor *actor = spawner.spawnActor(argv[2]);
	
	unsigned int robotID;
	std::sscanf(argv[1], "%u", &robotID);
	
	actor->initialSetup(robotID);
	while (actor->executeLoop());
	
	return 0;
}