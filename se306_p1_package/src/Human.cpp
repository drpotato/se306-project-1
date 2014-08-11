#include "ros/ros.h"

#include "Actor.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

class Human : public Actor
{
public:
  virtual int mainHook(int argc, char **argv)
  {    
    return 0;
  }
};

int main(int argc, char **argv)
{
  Human* human = new Human();
  human->mainHook(argc, argv);
  return 0;
}