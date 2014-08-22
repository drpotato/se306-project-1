#ifndef SE306P1_ACTORLOCATION_H_DEFINED
#define SE306P1_ACTORLOCATION_H_DEFINED

#include <msg_pkg/Location.h>
#include <string>

class ActorLocation
{
public:
    ActorLocation(std::string, double, double);
    ActorLocation(msg_pkg::Location);

    // Id of the robot
    std::string id;

    // Position of the robot
    double xpos;
    double ypos;

};

#endif // #ifndef SE306P1_ACTORLOCATION_H_DEFINED