#include "Renderer.hpp"
#include "Texture.hpp"
#include "../Debug.hpp"

#define LOAD_GL_VERSION_2_1
#include "opengl/glloader.hpp"

namespace
{
	const static int GL_MIN_MAJOR = 2;
	const static int GL_MIN_MINOR = 1;
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
	
	// Create the render task slots
	for (int slot = 0; slot < _TLS_NTypes; ++slot)
	{
		_renderTaskSlots.push_back(std::vector<RenderTask *>());
	}
	
	// Set up the GL context, including loading function pointers
	loadOpenGL();
	
	UPS_LOGF("glCompileShader:%p", (void*)glCompileShader);
	glEnable(GL_TEXTURE_2D);
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
			glEnable(GL_DEPTH_TEST);
			setUpMatrix3D();
			break;
		case TLS_2D:
			// Configure for 2D
			glDisable(GL_DEPTH_TEST);
			setUpMatrix2D();
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

void ups::Renderer::setUpMatrix2D()
{
	int w = _context->getWidth();
	int h = _context->getHeight();
	
	float mat[16] = {0.f};
	
	// Transform from [0 to w, 0 to h] to [-1 to 1, -1 to 1] 
	mat[0] = 2.f / w;
	mat[5] = 2.f / h;
	mat[12] = -1.f;
	mat[13] = -1.f;
	mat[15] = 1.f;
	
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(mat);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
}

void ups::Renderer::setUpMatrix3D()
{
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
	case RenderTask::RT_TestQuad:
		glColor4f(task->testQuad.col.r, task->testQuad.col.g, task->testQuad.col.b, task->testQuad.col.a);
		glBegin(GL_QUADS);
		glVertex2f(task->testQuad.x, task->testQuad.y);
		glVertex2f(task->testQuad.x, task->testQuad.y + task->testQuad.h);
		glVertex2f(task->testQuad.x + task->testQuad.w, task->testQuad.y + task->testQuad.h);
		glVertex2f(task->testQuad.x + task->testQuad.w, task->testQuad.y);
		glEnd();
		glColor4f(1.f,1.f,1.f,1.f);
		break;
	default:
		break;
	}
}

void ups::Renderer::setEnvClearColour(const ups::Colour &colour)
{
	RenderTask *rt = _frameAlloc.allocate<RenderTask>();
	rt->type = RenderTask::RT_Clear;
	rt->clear.col = colour;
	
	addToTaskList(rt, TLS_Env);
}

void ups::Renderer::drawTestQuad(const ups::Colour &colour, float x, float y, float w, float h)
{
	RenderTask *rt = _frameAlloc.allocate<RenderTask>();
	rt->type = RenderTask::RT_TestQuad;
	rt->testQuad.col = colour;
	rt->testQuad.x = x;
	rt->testQuad.y = y;
	rt->testQuad.w = w;
	rt->testQuad.h = h;
	
	addToTaskList(rt, TLS_2D);
}

ups::TexHandle ups::Renderer::makeTexHandle(ups::Texture &tex)
{
	GLuint texHandle;
	glGenTextures(1, &texHandle);
	
	glBindTexture(GL_TEXTURE_2D, texHandle);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
	
	switch (tex.getTextureType())
	{
	case ups::Texture::TT_RGBA8:
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex.getWidth(), tex.getHeight(), 0, GL_RGBA, GL_UNSIGNED_BYTE, tex.getData());
		break;
	}	
	
	return texHandle;
}