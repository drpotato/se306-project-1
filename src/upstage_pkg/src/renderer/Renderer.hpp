#ifndef SE306P1_UPSTAGE_RENDERER_HPP_DEFINED
#define SE306P1_UPSTAGE_RENDERER_HPP_DEFINED

#include "Types.hpp"
#include "RenderTask.hpp"
#include "../Context.hpp"
#include "../StackAllocator.hpp"
#include <string>
#include <vector>

namespace ups
{
	class Texture;
	class Renderer
	{
	public:
		Renderer(Context &context);
		virtual ~Renderer();
		
		void render();
		
		// Functionality
		void setEnvClearColour(const Colour &colour);
		void drawTestQuad(const Colour &colour, float x, float y, float w, float h);
		void drawTexQuad(const Colour &colour, const std::string &texName, float x, float y, float w, float h);
		void drawTexQuad(const std::string &texName, float x, float y, float w, float h);
		void drawTextGlyph(const Colour &colour, const std::string &texName, float x, float y, float w, float h, float u, float v);
		TexHandle makeTexHandle(Texture &tex);
		
		int getWidth() const;
		int getHeight() const;
	protected:
		
		
	private:
		enum TaskListSlot
		{
			TLS_Env,
			TLS_3D,
			TLS_2D,
			_TLS_NTypes
		};
		
		void doRenderArrangeTasks();
		void doRenderBefore();
		void doRenderExecuteTasks();
		void doRenderAfter();
		void doTask(RenderTask *task);
		
		void setUpMatrix2D();
		void setUpMatrix3D();
		
		void addToTaskList(RenderTask *task, TaskListSlot slot);
		
		Context *_context;
		StackAllocator _frameAlloc;
		
		std::vector<std::vector<RenderTask *> > _renderTaskSlots;
	};
	

	inline void Renderer::addToTaskList(RenderTask *task, TaskListSlot slot)
	{
		_renderTaskSlots[slot].push_back(task);
	}
	
	
	inline int Renderer::getWidth() const
	{
		return _context->getWidth();
	}
	
	inline int Renderer::getHeight() const
	{
		return _context->getHeight();
	}
}
#endif // #ifndef SE306P1_UPSTAGE_RENDERER_HPP_DEFINED