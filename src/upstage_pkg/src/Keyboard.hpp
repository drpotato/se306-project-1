#ifndef SE306P1_UPSTAGE_KEYBOARD_HPP_DEFINED
#define SE306P1_UPSTAGE_KEYBOARD_HPP_DEFINED

#include "KeyCodes.hpp"
#include <X11/Xlib.h>
#include <set>
#include <string>

#define KEYBOARD_STATES_COUNT 256

#define KEYBOARD_STATES_NBYTES ((KEYBOARD_STATES_COUNT + 7) / 8)

namespace ups
{
	class Keyboard
	{
	public:
		Keyboard();
		~Keyboard();
		
		void setCodeState(unsigned char keyCode, bool state);
		bool getCodeState(unsigned char keyCode) const;
		bool getKeyState(Display *display, KeySym keySymbol) const;
		
		std::set<unsigned char> getActiveKeyCodes() const;
		std::set<std::string> getActiveKeyCodeNames() const;
	private:
		unsigned char keyStates[KEYBOARD_STATES_NBYTES];
	};
}

inline ups::Keyboard::Keyboard()
{
	for (unsigned int i = 0; i < KEYBOARD_STATES_NBYTES; ++i)
	{
		keyStates[i] = 0;
	}
}

inline ups::Keyboard::~Keyboard()
{
}

		
inline void ups::Keyboard::setCodeState(unsigned char keycode, bool state)
{
	unsigned char intraByteOffset = keycode & 0x07;
	unsigned char interByteOffset = keycode >> 3;
	unsigned char *keyStatePtr = keyStates + (interByteOffset);
	
	*keyStatePtr = (state << intraByteOffset) | (*keyStatePtr & ~(1 << intraByteOffset));
}

inline bool ups::Keyboard::getCodeState(unsigned char keycode) const
{
	unsigned char intraByteOffset = keycode & 0x07;
	unsigned char interByteOffset = keycode >> 3;
	
	return (keyStates[interByteOffset] >> intraByteOffset) & 1;
}

inline bool ups::Keyboard::getKeyState(Display *display, KeySym keySymbol) const
{
	return getCodeState(XKeysymToKeycode(display, keySymbol));
}

#endif // #ifndef SE306P1_UPSTAGE_KEYBOARD_HPP_DEFINED