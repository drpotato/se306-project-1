#ifndef SE306P1_CLOCK_DEFINED
#define SE306P1_CLOCK_DEFINED

#include "ros/ros.h"
#include <ctime>
#include <time.h>
#include <msg_pkg/Interaction.h>
#include "Actor.h"

class Clock : public Actor
{
public:  
	virtual void doInitialSetup();
  virtual void doExecuteLoop();

  // Publisher for time
  ros::Publisher publisherTime;

private:
  // Stores the time of day in the Ultron world
  std::time_t time_of_day;

  // Stores the number of seconds needed to add to time_of_day upon each loop
  double seconds_to_add;

  // Stores an object that allows you to get separate values easily
   struct tm * ptm;


  // Number of seconds to increase Ultron world time by on each ROS loop
  int secondIncreasePerLoop();
  // Hours are inverted 12 hours for some reason. This function inverts it.
  void invertHours(int hour);

};


#endif