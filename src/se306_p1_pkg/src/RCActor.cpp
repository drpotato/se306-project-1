#include "RCActor.h"
#include "keyinput/KeyboardListener.hpp"

void RCActor::doInitialSetup()
{
}

void RCActor::doExecuteLoop()
{
	KeyboardListener &keyboardListener = KeyboardListener::getInstance();
	velRotational = 0.0;
	velLinear = 0.0;
	
	if (keyboardListener.isKeyPressed(ups::KEY_W_CODE))
	{
		velLinear += 1.0;
	}
	
	if (keyboardListener.isKeyPressed(ups::KEY_S_CODE))
	{
		velLinear -= 1.0;
	}
	
	if (keyboardListener.isKeyPressed(ups::KEY_A_CODE))
	{
		velRotational += 1.0;
	}
	
	if (keyboardListener.isKeyPressed(ups::KEY_D_CODE))
	{
		velRotational -= 1.0;
	}
}

