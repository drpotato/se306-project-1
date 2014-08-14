#ifndef SE306P1_ACTOR_RELATIVE_H_DEFINED
#define SE306P1_ACTOR_RELATIVE_H_DEFINED

#include "Visitor.h"

class Relative : public Visitor
{

public:
  
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  int status;
  
private:
  // Variable for holding whether or not the relative
  // is interacting with the Resident actor
  bool interacting;
};


#endif

