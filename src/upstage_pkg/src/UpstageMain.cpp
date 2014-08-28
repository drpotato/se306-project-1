#include "Context.hpp"
#include "rosworld/ROSComm.hpp"
#include "Resource.hpp"
#include "ResourceManager.hpp"
#include "StackAllocator.hpp"
#include "UpstageEnvironment.hpp"
#include "renderer/Font.hpp"
#include "renderer/Renderer.hpp"
#include "renderer/Text.hpp"
#include "renderer/Image.hpp"

int main(int argc, char **argv)
{
	ups::Context &context = ups::Context::getContext();
	ups::Renderer renderer(context);
	ups::ROSComm &rosComm = ups::ROSComm::getROSComm();
	ups::ResourceManager &resMan = ups::ResourceManager::getInstance();
	resMan.addPriorityPath("upstage");
	
	ups::UpstageEnvironment *env = resMan.fetch<ups::UpstageEnvironment>("upstageenv.unv");
	// ups::Image image("test.bmp");
	// image.setOffsetsL(0.f, 0.f);
	// image.setOffsetsR(0.f, 1.f);
	// image.setOffsetsU(0.f, 1.f);
	// image.setOffsetsD(0.f, 0.f);
	// ups::Font *font = resMan.fetch<ups::Font>("fonts/tiny.fnt");
	// ups::Font *font2 = resMan.fetch<ups::Font>("fonts/ubuntu_mono.fnt");

	// ups::Text textTest("Testing testing testing", *font, 80.f, 256.f, 480.f);
	// ups::Text textTest2("Testing testing testing", *font2, 80.f, 384.f, 480.f);
	
	bool isContinuing = true;
	while (isContinuing)
	{
		isContinuing = context.step();
		
		rosComm.publishKeys(context.getKeyboard());
		isContinuing&= rosComm.executeLoop();
		
		// Update positions etc. here
		env->step();
		env->draw(renderer);
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}