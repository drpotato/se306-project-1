#ifndef SE306P1_ACTOR_VISITOR_H_DEFINED
#define SE306P1_ACTOR_VISITOR_H_DEFINED

#include "Human.h"

class Visitor : public Human
{
public:
    virtual void publishLeave();
protected:
	virtual void doInitialSetup();
	virtual void doExecuteLoop();
	ros::Publisher publisherLeave;
};


#endif // #ifndef SE306P1_ACTOR_VISITOR_H_DEFINED

