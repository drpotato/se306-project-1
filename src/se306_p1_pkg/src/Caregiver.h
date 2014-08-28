#ifndef SE306P1_ACTOR_CAREGIVER_H_DEFINED
#define SE306P1_ACTOR_CAREGIVER_H_DEFINED

#include "Visitor.h"
#include "ActorSpawner.h"
#include <msg_pkg/Fitness.h>
#include <msg_pkg/Hunger.h>
#include <msg_pkg/Hygiene.h>
#include <msg_pkg/Morale.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Time.h>
#include <msg_pkg/Telephone.h>

class Caregiver : public Visitor
{
protected:
    virtual void doInitialSetup();
    virtual void doExecuteLoop();
    string getActorName();
    void caring();

    ros::Subscriber subscriberFitness;
    ros::Subscriber subscriberHunger;
    ros::Subscriber subscriberHygiene;
    ros::Subscriber subscriberMorale;
    ros::Subscriber subscriberSocialness;
    ros::Subscriber subscriberTime;
    ros::Subscriber subscriberTelephone;
//  ros::Subscriber subscriberLockStatus;

    static void fitnessCallback(msg_pkg::Fitness msg);
    static void hungerCallback(msg_pkg::Hunger msg);
    static void hygieneCallback(msg_pkg::Hygiene msg);
    static void moraleCallback(msg_pkg::Morale msg);
    static void socialnessCallback(msg_pkg::Socialness msg);
    static void timeCallback(msg_pkg::Time msg);
    static void telephoneCallback(msg_pkg::Telephone msg);
//  static void lockStatusCallback(msg_pkg::LockStatus msg);

    bool homeVisit;
    bool movingToResident;
    bool odd;
    string nodename;
    int caregiverId;
    int hour;
    int y;

    bool fitnessLevel, hungerLevel, hygieneLevel, moraleLevel, socialnessLevel;
    bool exercising, eating, showering, entertaining, socialising;
    bool locked;

    bool checkFitnessLevel();
    bool checkHungerLevel();
    bool checkHygieneLevel();
    bool checkMoraleLevel();
    bool checkSocialnessLevel();
};


#endif
