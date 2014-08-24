#include "Context.hpp"
#include "ROSComm.hpp"
#include "Resource.hpp"
#include "ResourceManager.hpp"
#include "StackAllocator.hpp"
#include "renderer/Renderer.hpp"

int main(int argc, char **argv)
{
	ups::Context &context = ups::Context::getContext();
	ups::Renderer renderer(context);
	ups::ROSComm &rosComm = ups::ROSComm::getROSComm();
	ups::ResourceManager &resMan = ups::ResourceManager::getInstance();
	resMan.addPriorityPath("upstage");
	
	resMan.fetch<ups::Resource>("test.xml");
	
	renderer.setEnvClearColour(0.f, 0.f, 0.f);
	
	bool isContinuing = true;
	while (isContinuing)
	{
		isContinuing = context.step();
		
		rosComm.publishKeys(context.getKeyboard());
		isContinuing&= rosComm.executeLoop();
		
		// Update positions etc. here
		
		
		
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}