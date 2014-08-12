#ifndef SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED
#define SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED

class Actor;
class ActorSpawner
{
public:
	static ActorSpawner &getInstance();

	Actor *spawnActor(const char *actorTypeName) const;
private:
	ActorSpawner();
};

#endif // #ifndef SE306P1_ACTOR_ACTORSPAWNER_H_DEFINED