#include "ros/ros.h"

#include "Actor.h"
#include "Robot.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

class Robot : public Actor
{
public:
  virtual int mainHook(int argc, char **argv)
  {    
    return 0;
  }
};

int main(int argc, char **argv)
{
  Robot* robot = new Robot();
  robot->mainHook(argc, argv);
  return 0;
}