#include <msg_pkg/Location.h>
#include <string>

#include "ActorLocation.h"

ActorLocation::ActorLocation(std::string name, double x, double y) {
    this->id = name;
    this->xpos = x;
    this->ypos = y;
}

ActorLocation::ActorLocation(msg_pkg::Location msg) {
    this->id = msg.id;
    this->xpos = msg.xpos;
    this->ypos = msg.ypos;
}