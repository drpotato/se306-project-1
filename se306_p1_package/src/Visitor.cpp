#include "ros/ros.h"

#include "Actor.h"
#include "Human.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

class Visitor : public Human
{
public:
  virtual int mainHook(int argc, char **argv)
  {    
    return 0;
  }
};

int main(int argc, char **argv)
{
  Visitor* visitor = new Visitor();
  visitor->mainHook(argc, argv);
  return 0;
}