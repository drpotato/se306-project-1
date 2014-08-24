#include "Keyboard.hpp"

namespace
{
	std::string keyCodeMap[KEYBOARD_STATES_COUNT];
	
	void initKeyCodeMap();
	std::string mapKeyCodeToString(unsigned char keyCode);
}

std::set<unsigned char> ups::Keyboard::getActiveKeyCodes() const
{
	std::set<unsigned char> activeKeyCodeSet;
	
	for (unsigned int i = 0; i < KEYBOARD_STATES_NBYTES; ++i)
	{
		if (!keyStates[i]) continue;
		
		for (unsigned int shift = 0, keyCode = i << 3; shift < 8; ++shift, ++keyCode)
		{
			if (keyStates[i] & (1 << shift))
			{
				activeKeyCodeSet.insert(keyCode);
			}
		}
	}
	
	return activeKeyCodeSet;
}

// Since this is based on the raw keycodes, it's not sensitive to remapping.
// This is actually what you want for things like WASD movement, since WASD is super-inconvenient
// on something like a dvorak layout.
// 
// That said, it's probably easier to use the one above.
std::set<std::string> ups::Keyboard::getActiveKeyCodeNames() const
{
	std::set<unsigned char> activeKeyCodes = getActiveKeyCodes();
	std::set<std::string> activeKeyCodeNames;
	
	for (std::set<unsigned char>::iterator it = activeKeyCodes.begin(); it != activeKeyCodes.end(); ++it)
	{
		activeKeyCodeNames.insert(mapKeyCodeToString(*it));
	}
	
	return activeKeyCodeNames;
}

namespace
{
	void initKeyCodeMap()
	{
		for (unsigned int i = 0; i < KEYBOARD_STATES_COUNT; ++i)
		{
			keyCodeMap[KEYBOARD_STATES_COUNT] = std::string();
		}
		
		// From http://www.gp32x.com/board/index.php?/topic/57164-raw-x11-keycodes/
		//keyCodeMap[10] = "1";
	}
	
	std::string mapKeyCodeToString(unsigned char keyCode)
	{
		static bool initKeyCodeMapCompleted;
		if (!initKeyCodeMapCompleted)
		{
			initKeyCodeMap();
			initKeyCodeMapCompleted = true;
		}
	}
}