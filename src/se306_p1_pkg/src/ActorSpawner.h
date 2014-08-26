#ifndef SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED
#define SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED

#include "PathPlanner.h"

class Actor; // This a compilation optimisation
class ActorSpawner
{
public:
	static ActorSpawner &getInstance();

	Actor *getActor(const char *actorTypeName);
	Actor *getActor() const;
    void setupPathPlanner();
private:
	ActorSpawner();
    Actor *spawnActor(const char *actorTypeName);
    Actor *theActor;
    PathPlanner* pathPlanner;
};

#endif // #ifndef SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED