#include "Context.hpp"
#include "ROSComm.hpp"
#include "StackAllocator.hpp"
#include "renderer/Renderer.hpp"

int main(int argc, char **argv)
{
	ups::Context &context = ups::Context::getContext();
	ups::Renderer renderer(context);
	ups::ROSComm &rosComm = ups::ROSComm::getROSComm();
	
	bool isContinuing = true;
	while (isContinuing)
	{
		isContinuing = context.step();
		
		rosComm.publishKeys(context.getKeyboard());
		isContinuing&= rosComm.executeLoop();
		
		// Update positions etc. here
		renderer.setEnvClearColour(1.f, 0.5f, 0.f);
		
		
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}