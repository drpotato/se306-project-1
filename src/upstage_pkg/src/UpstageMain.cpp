#include "Context.hpp"
#include "rosworld/ROSComm.hpp"
#include "Resource.hpp"
#include "ResourceManager.hpp"
#include "StackAllocator.hpp"
#include "UpstageEnvironment.hpp"
#include "renderer/Renderer.hpp"
#include "renderer/Texture.hpp"

int main(int argc, char **argv)
{
	ups::Context &context = ups::Context::getContext();
	ups::Renderer renderer(context);
	ups::ROSComm &rosComm = ups::ROSComm::getROSComm();
	ups::ResourceManager &resMan = ups::ResourceManager::getInstance();
	resMan.addPriorityPath("upstage");
	
	ups::UpstageEnvironment *env = resMan.fetch<ups::UpstageEnvironment>("upstageenv.unv");

	
	bool isContinuing = true;
	while (isContinuing)
	{
		isContinuing = context.step();
		
		rosComm.publishKeys(context.getKeyboard());
		isContinuing&= rosComm.executeLoop();
		
		// Update positions etc. here
		env->step();
		env->draw(renderer);
		ups::Colour c0 = ups::Colour::rgb(1.f, 0.f, 0.f, 1.f);
		ups::Colour c1 = ups::Colour::rgb(0.f, 1.f, 0.f, 1.f);
		renderer.drawTestQuad(c0, 32.f, 32.f, 64.f, 64.f);
		renderer.drawTexQuad(c1, "src/se306_p1_pkg/world/worldmaps/worldwalls.bmp", 32.f, 96.f, 256.f, 256.f);
		//renderer.drawTestQuad(c1, 0.0, 1.0, 1.0, -0.5);
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}