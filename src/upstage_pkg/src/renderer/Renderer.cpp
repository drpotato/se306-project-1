#include "Renderer.hpp"
#include "../Context.hpp"
#include "../Debug.hpp"

namespace
{
	const static int GL_MIN_MAJOR = 2;
	const static int GL_MIN_MINOR = 0;
	const static size_t FRAME_ALLOC_SIZE = 4 * 1024 * 1024; // 4 MiB
}

ups::Renderer::Renderer(ups::Context &context) :
	_frameAlloc(FRAME_ALLOC_SIZE)
{
	_context = &context;
	
	// Check the OpenGL version is at least what we require
	int glMaj, glMin; context.getGLVersion(glMaj, glMin);
	UPS_ASSERTF(context.minGLVersion(GL_MIN_MAJOR, GL_MIN_MINOR),
		"This program requires at least OpenGL %d.%d.\n"
		"The context created was only OpenGL %d.%d!\n"
		"Things to try:\n"
		"\t1.\tUpdate graphics drivers\n"
		"\t2.\tTry a computer with a newer graphics card\n"
		"\t3.\tPerhaps counter-intuitively, try taking the card out.\n"
		"\t\tThis will hopefully cause it to fall back on\n"
		"\t\tMesa, which is slow, but should have sufficient OpenGL support.\n\n"
		"If nothing works, please contact jpet987@aucklanduni.ac.nz",
		GL_MIN_MAJOR, GL_MIN_MINOR, glMaj, glMin);
	
	for (int slot = 0; slot < _TLS_NTypes; ++slot)
	{
		_renderTaskSlots.push_back(std::vector<RenderTask *>());
	}
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
	for (int slot = 0; slot < _TLS_NTypes; ++slot)
	{
		switch (slot)
		{
		case TLS_3D:
			// Configure for 3D
			break;
		case TLS_2D:
			// Configure for 2D
			break;
		default:
			break;
		}
		
		for (int task = 0; task < _renderTaskSlots[slot].size(); ++task)
		{
			doTask(_renderTaskSlots[slot][task]);
		}
	}
}

void ups::Renderer::doRenderAfter()
{
	// All of the pointers in the task lists are about to be invalidated, so clear them
	for (int slot = 0; slot < _TLS_NTypes; ++slot)
	{
		_renderTaskSlots[slot].clear();
	}
	
	// Clear the frame allocator for the next frame
	_frameAlloc.clear();
}

void ups::Renderer::doTask(RenderTask *task)
{
	switch (task->type)
	{
	case RenderTask::RT_Clear:
		glClearColor(task->clear.col.r, task->clear.col.g, task->clear.col.b, task->clear.col.a);
		break;
	default:
		break;
	}
}

void ups::Renderer::setEnvClearColour(const Colour &colour)
{
	RenderTask *rt = _frameAlloc.allocate<RenderTask>();
	rt->type = RenderTask::RT_Clear;
	rt->clear.col = colour;
	
	addToTaskList(rt, TLS_Env);
}