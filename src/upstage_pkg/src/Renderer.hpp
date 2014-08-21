#ifndef SE306P1_UPSTAGE_RENDERER_HPP_DEFINED
#define SE306P1_UPSTAGE_RENDERER_HPP_DEFINED

#include "RenderTask.hpp"
#include "PointerUnique.hpp"
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
		void doRenderArrangeTasks();
		void doRenderBefore();
		void doRenderExecuteTasks();
		void doRenderAfter();
		
		Context *_context;
		
		std::vector<PointerUnique<RenderTask> > _renderEnvTasks;
		std::vector<PointerUnique<RenderTask> > _render3DTasks;
		std::vector<PointerUnique<RenderTask> > _render2DTasks;
	};
}
#endif // #ifndef SE306P1_UPSTAGE_RENDERER_HPP_DEFINED