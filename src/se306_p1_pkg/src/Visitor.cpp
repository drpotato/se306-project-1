#include "Visitor.h"
#include <msg_pkg/Leave.h>

void Visitor::doInitialSetup()
{

    // Create leave publisher.
    publisherLeave = nodeHandle->advertise<msg_pkg::Leave>("leave", 1000);
}

void Visitor::doExecuteLoop()
{
}


void Visitor::publishLeave()
{
	//Create a leave message to publish
	msg_pkg::Leave leaveMessage;
	//Assign id to rosName
	leaveMessage.id = rosName;
	//Publish the message
	publisherLeave.publish(leaveMessage);
}
