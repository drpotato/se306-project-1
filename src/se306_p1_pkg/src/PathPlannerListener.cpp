#include "PathPlannerListener.h"
#include "ros/ros.h"
#include <msg_pkg/Location.h>

// This class maintains a graph of navigation waypoint nodes, and calculates the shortest (fewest nodes) path between any two of them.

ros::Subscriber PathPlannerListener::subscriberLocation;
ros::NodeHandle* PathPlannerListener::nodeHandle;
boost::unordered_map<std::string, queue<msg_pkg::Location> > PathPlannerListener::messageQueues;

PathPlannerListener::PathPlannerListener() {
    nodeHandle = new ros::NodeHandle();
    subscriberLocation = nodeHandle->subscribe("location", 1000, PathPlannerListener::locationCallback);
}

void PathPlannerListener::locationCallback(msg_pkg::Location msg){
  // Find Actor of this name in graph and remove it.
  string name = msg.id;
  double x = msg.xpos;
  double y = msg.ypos;

  //See if the message is already in the map
  if (!messageQueues.count(name)){
    messageQueues[name] = queue<msg_pkg::Location>();
  }

  messageQueues[name].push(msg);
}

queue<msg_pkg::Location>* PathPlannerListener::getMessages(string name){
  return &(messageQueues[name]);
}
