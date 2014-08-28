#include "../Actor.h"
#include "../ActorSpawner.h"
#include "KeyboardListener.hpp"

KeyboardListener &KeyboardListener::getInstance()
{
	static KeyboardListener *instance;
	
	if (!instance)
	{
		instance = new KeyboardListener();
	}
	
	return *instance;
}

void KeyboardListener::init()
{
	getInstance();
}

KeyboardListener::KeyboardListener()
{
	ros::NodeHandle &nodeHandle = ActorSpawner::getInstance().getActor()->getNodeHandle();
	keyInputSubscriber = nodeHandle.subscribe("upstagekeyinput", 1000, KeyboardListener::keyInputCallback);
}

KeyboardListener::~KeyboardListener()
{
}

void KeyboardListener::keyInputCallback(msg_pkg::KeyInput msg)
{
	KeyboardListener &keyboardListener = getInstance();
	
	// Clear the old active keys
	keyboardListener.activeKeys.clear();
	
	// Add the keys from the message
	for (int i = 0; i < msg.pressedkeys.size(); ++i)
	{
		keyboardListener.activeKeys.insert(msg.pressedkeys[i]);
	}
}

bool KeyboardListener::isKeyPressed(ups::KeyCode keyCode) const
{
	return activeKeys.find(keyCode) != activeKeys.end();
}