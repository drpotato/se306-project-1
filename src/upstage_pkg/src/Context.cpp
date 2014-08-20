#include "Context.hpp"
#include "Debug.hpp"
#include <unistd.h>

ups::Context &ups::Context::getContext()
{
	static ups::Context *instance;
	
	if (!instance)
	{
		instance = new ups::Context();
	}
	
	return *instance;
}

ups::Context::Context():
	_w(800),
	_h(600)
{
	GLint windowAttributes[] = {GLX_RGBA, GLX_DEPTH_SIZE, 24, GLX_DOUBLEBUFFER, None}; // {GLX_RGBA, None}
	Window rootWindow;
	XVisualInfo *visualInfo;
	Colormap colourMap;
	XSetWindowAttributes setWindowAttributes;
	
	// Open X server connection to the display
	// (using the DISPLAY environment variable, since display_name is NULL)
	UPS_ASSERT(_display = XOpenDisplay(NULL), "Cannot connect to X server.");
	
	// Get the root window of the display
	UPS_ASSERT(rootWindow = DefaultRootWindow(_display), "Unable to get the root window.");
	
	// Select a visual to approximate our attributes
	UPS_ASSERT(visualInfo = glXChooseVisual(_display, 0, windowAttributes), "Could not select an appropriate visual");
	
	// Create a colourmap of the specified visual type
	colourMap = XCreateColormap(_display, rootWindow, visualInfo->visual, AllocNone);
	
	// Specify events and colour maps.
	_eventMask						= // http://tronche.com/gui/x/xlib/events/mask.html
		KeyPressMask				|
		KeyReleaseMask				|
		ButtonPressMask				|
		StructureNotifyMask			;
	setWindowAttributes.event_mask = _eventMask;
	setWindowAttributes.colormap = colourMap;
	
	// Create the window!
	_window = XCreateWindow(
		_display,					// Display*					display
		rootWindow,					// Window					parent
		0,							// int						x
		0,							// int						y
		_w,							// unsigned int				width
		_h,							// unsigned int				height
		0,							// unsigned int				border_width
		visualInfo->depth,			// int						depth
		InputOutput,				// unsigned int				class
		visualInfo->visual,			// Visual*					visual
		CWColormap | CWEventMask,	// unsigned long			valuemask
		&setWindowAttributes		// XSetWindowAttributes*	attributes
	);
	
	// Allow capturing WM_DELETE_WINDOW events
	// Technically we're not actually going to capture them... but it stops the user aborting execution with the X button.
	_wmDeleteMessage = XInternAtom(_display, "WM_DELETE_WINDOW", False);
	XSetWMProtocols(_display, _window, &_wmDeleteMessage, 1);
	
	// Map the window to make it visible
	XMapWindow(_display, _window);
	
	// Name the window
	XStoreName(_display, _window, "Team Google Ultron Dev Team - Upstage");
	
	// Create a direct GL context, and run setup
	_glContext = glXCreateContext(_display, visualInfo, NULL, GL_TRUE);
	glXMakeCurrent(_display, _window, _glContext);
	setupGL();
	
	std::printf("Great success!\n\n");
}

ups::Context::~Context()
{
	glXMakeCurrent(_display, None, NULL);
	glXDestroyContext(_display, _glContext);
	XDestroyWindow(_display, _window);
	XCloseDisplay(_display);
}

void ups::Context::onResize()
{
	printf("Resized (%d, %d)\n", _w, _h);
	glViewport(0, 0, _w, _h);
}

void ups::Context::setupGL()
{
	glEnable(GL_DEPTH_TEST);
	glClearColor(1.f, 0.f, 1.f, 1.f);
}

void ups::Context::clearGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

bool ups::Context::step()
{
	XEvent xEvent;
	while (XCheckWindowEvent(_display, _window, _eventMask, &xEvent))
	{
		// http://tronche.com/gui/x/xlib/events/structures.html
		switch (xEvent.type)
		{
		/*
        case ClientMessage:
			printf("Received ClientMessage (%d) | (%d)", xEvent.xclient.data.l[0], _wmDeleteMessage);
            if (xEvent.xclient.data.l[0] == _wmDeleteMessage)
                return false;
            break;*/
		case KeyPress:
			_keyboard.setCodeState(xEvent.xkey.keycode, true);
			break;
		case KeyRelease:
			_keyboard.setCodeState(xEvent.xkey.keycode, false);
			break;
		case ConfigureNotify:
			if (xEvent.xconfigure.width != _w || xEvent.xconfigure.height != _h)
			{
				_w = xEvent.xconfigure.width;
				_h = xEvent.xconfigure.height;
				onResize();
			}
			_keyboard.setCodeState(xEvent.xkey.keycode, false);
			break;
		default:
			break;
		}
	}
	
	// Quit when CTRL+C is pressed
	if ((_keyboard.getKeyState(_display, XK_Control_L) ||
		_keyboard.getKeyState(_display, XK_Control_R))
		&& _keyboard.getKeyState(_display, XK_C))
	{
		return false;
	}

	return true;
}

void ups::Context::drawStart()
{
	clearGL();
}

void ups::Context::drawEnd()
{
	glXSwapBuffers(_display, _window);
}