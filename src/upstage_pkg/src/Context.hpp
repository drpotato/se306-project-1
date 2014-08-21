#ifndef SE306P1_UPSTAGE_CONTEXT_HPP_DEFINED
#define SE306P1_UPSTAGE_CONTEXT_HPP_DEFINED

#include "Keyboard.hpp"
#include <X11/X.h>
#include <X11/Xlib.h>
#include <GL/glx.h>

namespace ups
{
	class Context
	{
	public:
		~Context();
		
		bool step();
		void drawStart();
		void drawEnd();
		
		bool minGLVersion(int majorMin, int minorMin) const;
		void getGLVersion(int &major, int &minor) const;
		
		static Context &getContext();
	private:
		Context();
		
		void onResize();
		void setupGL();
		void clearGL();
		
		Keyboard _keyboard;
		int _w, _h;
		
		int _glVersionMajor, _glVersionMinor;
		
		long _eventMask;
		Display *_display;
		Window _window;
		GLXContext _glContext;
		Atom _wmDeleteMessage;
	};
}

inline bool ups::Context::minGLVersion(int majorMin, int minorMin) const
{
	return _glVersionMajor != majorMin ? _glVersionMajor > majorMin : _glVersionMinor >= minorMin;
}

inline void ups::Context::getGLVersion(int &major, int &minor) const
{
	major = _glVersionMajor;
	minor = _glVersionMinor;
}

#endif // #ifndef SE306P1_UPSTAGE_CONTEXT_HPP_DEFINED