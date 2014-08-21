#ifndef SE306P1_UPSTAGE_RENDERER_HPP_DEFINED
#define SE306P1_UPSTAGE_RENDERER_HPP_DEFINED

#include "RenderTask.hpp"
#include "StackAllocator.hpp"
#include <vector>

namespace ups
{
	class Context;
	class Renderer
	{
	public:
		Renderer(Context &context);
		virtual ~Renderer();
		
		void render();
		
		// Functionality
		void setEnvClearColour(float r, float g, float b);
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
		
		void addToTaskList(RenderTask *task, TaskListSlot slot);
		
		Context *_context;
		StackAllocator _frameAlloc;
		
		std::vector<std::vector<RenderTask *> > _renderTaskSlots;
	};
	

	inline void Renderer::addToTaskList(RenderTask *task, TaskListSlot slot)
	{
		_renderTaskSlots[slot].push_back(task);
	}
}
#endif // #ifndef SE306P1_UPSTAGE_RENDERER_HPP_DEFINED