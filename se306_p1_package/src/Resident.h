#ifndef SE306P1_ACTOR_RESIDENT_H_DEFINED
#define SE306P1_ACTOR_RESIDENT_H_DEFINED

#include "Human.h"

class Resident : public Human
{
public:
  virtual bool isLocked();
  virtual void lock();
  virtual void unlock();
  
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  
private:
  bool lock_;
};


#endif

