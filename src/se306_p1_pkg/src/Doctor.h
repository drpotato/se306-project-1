#ifndef SE306P1_ACTOR_DOCTOR_H_DEFINED
#define SE306P1_ACTOR_DOCTOR_H_DEFINED

#include "Visitor.h"
#include <msg_pkg/Telephone.h>
#include <msg_pkg/Nurse.h>
#include <msg_pkg/Health.h>
#include "ActorSpawner.h"

class Doctor : public Visitor
{
protected:
  virtual void doInitialSetup();
  virtual void doExecuteLoop();
  string getActorName();
  
  void emergency();
  void callNurses();
  void attendPatient();

  static void telephoneCallback(msg_pkg::Telephone msg);
  static void healthCallback(msg_pkg::Health msg);

  ros::Subscriber subscriberTelephone;
  ros::Subscriber subscriberHealth;
  ros::Publisher publisherNurse1;
  ros::Publisher publisherNurse2;

  bool homeVisit;
  bool travellingToResident;
  bool treating;
  bool first;
  bool goHome;
  int8_t healthLevel;

};


#endif