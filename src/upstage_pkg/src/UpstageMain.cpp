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
	ups::Image image("test.bmp");
	image.setOffsetsL(0.f, 0.f);
	image.setOffsetsR(0.f, 1.f);
	image.setOffsetsU(0.f, 1.f);
	image.setOffsetsD(0.f, 0.f);
	ups::Font *font = resMan.fetch<ups::Font>("fonts/tiny.fnt");
	ups::Font *font2 = resMan.fetch<ups::Font>("fonts/ubuntu_mono.fnt");

	ups::Text textTest("Testing testing testing", *font, 80.f, 256.f, 480.f);
	ups::Text textTest2("Testing testing testing", *font2, 80.f, 384.f, 480.f);
	
	bool isContinuing = true;
	while (isContinuing)
	{
		isContinuing = context.step();
		
		rosComm.publishKeys(context.getKeyboard());
		isContinuing&= rosComm.executeLoop();
		
		// Update positions etc. here
		env->step();
		env->draw(renderer);
		image.draw(renderer);
		textTest.draw(renderer);
		textTest2.draw(renderer);
		// ups::Colour c0 = ups::Colour::rgb(1.f, 0.f, 0.f, 1.f);
		// ups::Colour c1 = ups::Colour::rgb(0.f, 1.f, 0.f, 1.f);
		// renderer.drawTestQuad(c0, 32.f, 32.f, 64.f, 64.f);
		// renderer.drawTexQuad(ups::Colour::rgb(1.f, 1.f, 1.f, 1.f), "fonts/ubuntu_mono.tga", 32.f, 96.f, 256.f, 256.f);
		// renderer.drawTestQuad(c1, 0.0, 1.0, 1.0, -0.5);
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}