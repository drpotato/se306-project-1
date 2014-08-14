#ifndef SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED
#define SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED

class Actor; // This a compilation optimisation
class ActorSpawner
{
public:
	static ActorSpawner &getInstance();

	Actor *getActor(const char *actorTypeName);
	Actor *getActor() const;
private:
	ActorSpawner();
        Actor *spawnActor(const char *actorTypeName);
        Actor *theActor;
};

#endif // #ifndef SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED