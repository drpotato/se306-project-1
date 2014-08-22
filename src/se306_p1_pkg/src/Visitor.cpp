#include "Visitor.h"
#include <msg_pkg/Leave.h>

// Superclass to separate the Resident from other Humans (Visitors) and implement common functionality.
void Visitor::doInitialSetup()
{
    // Messages will be published to the 'leave' topic to notify the VisitorController that this Visitor has left the house.
    publisherLeave = nodeHandle->advertise<msg_pkg::Leave>("leave", 1000);
}

void Visitor::doExecuteLoop()
{
}

void Visitor::publishLeave()
{
	// Create a leave message to publish.
	msg_pkg::Leave leaveMessage;
	// Insert own rosName into the id field.
	leaveMessage.id = rosName;
	// Publish the message.
	publisherLeave.publish(leaveMessage);
}
