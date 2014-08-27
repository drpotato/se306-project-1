#include "ROSComm.hpp"

#include "../Keyboard.hpp"
#include <msg_pkg/KeyInput.h>

ups::ROSComm &ups::ROSComm::getROSComm()
{
	static ups::ROSComm *instance;
	
	if (!instance)
	{
		instance = new ups::ROSComm();
	}
	
	return *instance;
}

ups::ROSComm::ROSComm():
	rosName("UpstageROSComm")
{
	// ros::init needs L-values, so we can't just directly pass (0, ...)
	// We don't need support for any special command-line arguments for ROS
	int fakeArgC = 0;
	ros::init(fakeArgC, 0, rosName.c_str());
	
	nodeHandle = new ros::NodeHandle();
	loopRate = new ros::Rate(10);
	
	publisherKeyInput = nodeHandle->advertise<msg_pkg::KeyInput>("upstagekeyinput", 1000);
}

ups::ROSComm::~ROSComm()
{
}

// To avoid losing keys, the set of active keys is added to, but not cleared until the message is sent.
void ups::ROSComm::publishKeys(const ups::Keyboard &keyboard)
{
	std::set<unsigned char> activeKeysFromKeyboard = keyboard.getActiveKeyCodes();
	for (std::set<unsigned char>::iterator it = activeKeysFromKeyboard.begin(); it != activeKeysFromKeyboard.end(); ++it)
	{
		activeKeys.insert(*it);
	}
}

// Flush the activeKeys set, and send the messages.
void ups::ROSComm::doPublishKeys()
{
	// Create a KeyInput message to publish.
	msg_pkg::KeyInput keyInputMessage;
	
	// Add all stored key codes
	for (std::set<unsigned char>::iterator it = activeKeys.begin(); it != activeKeys.end(); ++it)
	{
		keyInputMessage.pressedkeys.push_back(*it);
	}
	
	// Send the message
	publisherKeyInput.publish(keyInputMessage);
	
	// Clear the key code set
	activeKeys.clear();
}

bool ups::ROSComm::executeLoop()
{
	if (ros::ok())
	{
		doPublishKeys();

		ros::spinOnce();
		loopRate->sleep();
		return true;
	}

	return false;
}