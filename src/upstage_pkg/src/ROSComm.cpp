#include "ROSComm.hpp"

#include "Keyboard.hpp"
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

void ups::ROSComm::publishKeys(const ups::Keyboard &keyboard)
{

}

void ups::ROSComm::doPublishKeys()
{

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