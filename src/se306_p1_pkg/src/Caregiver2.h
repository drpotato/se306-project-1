#ifndef SE306P1_ACTOR_Caregiver2_H_DEFINED
#define SE306P1_ACTOR_Caregiver2_H_DEFINED

#include "Caregiver.h"
#include "ActorSpawner.h"
#include <msg_pkg/Fitness.h>
#include <msg_pkg/Hunger.h>
#include <msg_pkg/Hygiene.h>
#include <msg_pkg/Morale.h>
#include <msg_pkg/Socialness.h>
#include <msg_pkg/Interaction.h>
#include <msg_pkg/Time.h>

class Caregiver2 : public Caregiver
{
protected:
    virtual void doInitialSetup();
    virtual void doExecuteLoop();
/*
    bool checkFitnessLevel();
    bool checkHungerLevel();
    bool checkHygieneLevel();
    bool checkMoraleLevel();
    bool checkSocialLevel();

    int8_t fitnessLevel;
    int8_t hungerLevel;
    int8_t hygieneLevel; 
    int8_t moraleLevel;
    int8_t socialnessLevel;

    bool exercising;  // Increase fitnessLevel
    bool feeding;  // Increase hungerLevel
    bool bathing;  // Increase hygieneLevel
    bool moralesupporting;  // Increase moraleLevel
    bool talking;  // Increase socialnessLevel
*/
    static void fitnessCallback(msg_pkg::Fitness msg);
    static void hungerCallback(msg_pkg::Hunger msg);
    static void hygieneCallback(msg_pkg::Hygiene msg);
    static void moraleCallback(msg_pkg::Morale msg);
    static void socialnessCallback(msg_pkg::Socialness msg);
    static void timeCallback(msg_pkg::Time msg);
/*    
    ros::Subscriber subscriberFitness;
    ros::Subscriber subscriberHunger;
    ros::Subscriber subscriberHygiene;
    ros::Subscriber subscriberMorale;
    ros::Subscriber subscriberSocialness;
    ros::Subscriber subscriberTime;

    string caregiverName;
    int y1;
    int y2;
    int y3;
    int y4;
    int y5;
    int x;
    bool first;
    bool first_call;
    bool returningHome;
    bool returningHome_first;
    int hour;
    bool odd;
*/
};


#endif  // #ifndef SE306P1_ACTOR_CAREGIVER_H_DEFINED
