#include "Renderer.hpp"
#include "Context.hpp"
#include "Debug.hpp"

#define GL_MIN_MAJOR 2
#define GL_MIN_MINOR 0

ups::Renderer::Renderer(ups::Context &context)
{
	_context = &context;
	
	// Check the OpenGL version is at least what we require
	int glMaj, glMin; context.getGLVersion(glMaj, glMin);
	UPS_ASSERTF(context.minGLVersion(GL_MIN_MAJOR, GL_MIN_MINOR), "This program requires at least OpenGL %d.%d.\n"
		"The context created was only OpenGL %d.%d!\n"
		"Things to try:\n\t1.\tUpdate graphics drivers\n\t2.\tTry a computer with a newer graphics card\n"
		"\t3.\tPerhaps counter-intuitively, try taking the card out.\n\t\tThis will hopefully cause it to fall back on\n"
		"\t\tMesa, which is slow, but should have sufficient OpenGL support.\n\n"
		"If nothing works, please contact jpet987@aucklanduni.ac.nz", GL_MIN_MAJOR, GL_MIN_MINOR, glMaj, glMin);
	

}
ups::Renderer::~Renderer()
{
}

void ups::Renderer::render()
{
	doRenderArrangeTasks();
	doRenderBefore();
	doRenderExecuteTasks();
	doRenderAfter();
}


void ups::Renderer::doRenderArrangeTasks()
{
}

void ups::Renderer::doRenderBefore()
{
}

void ups::Renderer::doRenderExecuteTasks()
{
}

void ups::Renderer::doRenderAfter()
{
}


void ups::Renderer::setEnvClearColour(float r, float g, float b)
{
}