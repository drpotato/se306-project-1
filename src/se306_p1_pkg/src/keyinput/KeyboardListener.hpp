#include "KeyCodes.hpp"
#include "ros/ros.h"
#include <msg_pkg/KeyInput.h>
#include <set>

class KeyboardListener
{
public:
	~KeyboardListener();
	
	static KeyboardListener &getInstance();
	static void init();
	
	bool isKeyPressed(ups::KeyCode keyCode) const;
	bool isKeyTapped(ups::KeyCode keyCode) const;
private:
	KeyboardListener();
	KeyboardListener(const KeyboardListener &);
	
	static void keyInputCallback(msg_pkg::KeyInput msg);
	
	std::set<unsigned char> activeKeys;
	std::set<unsigned char> activeKeysPrevious;
	ros::Subscriber keyInputSubscriber;
};