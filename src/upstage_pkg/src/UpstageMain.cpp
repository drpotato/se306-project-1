#include "Context.hpp"
#include "ROSComm.hpp"
#include "Resource.hpp"
#include "ResourceManager.hpp"
#include "StackAllocator.hpp"
#include "UpstageEnvironment.hpp"
#include "renderer/Renderer.hpp"
#include "maths/Quaternion.hpp"

int main(int argc, char **argv)
{
	ups::Context &context = ups::Context::getContext();
	ups::Renderer renderer(context);
	ups::ROSComm &rosComm = ups::ROSComm::getROSComm();
	ups::ResourceManager &resMan = ups::ResourceManager::getInstance();
	resMan.addPriorityPath("upstage");
	
	ups::UpstageEnvironment *env = resMan.fetch<ups::UpstageEnvironment>("upstageenv.unv");
	
	ups::Quaternion q = ups::Quaternion::fromEulerDegrees(45.f, 0.f, 45.f);
	ups::Quaternion::qUnit mat[16];
	
	q.getMatrix(mat);
	printf("Mat:\n");
	for (int i = 0; i < 16; ++i)
	{
		int transI = ((i & 3) << 2) | (i >> 2);
		printf("%f%s", mat[transI], ((i + 1) & 3) ? " " : "\n");
	}
	
	bool isContinuing = true;
	while (isContinuing)
	{
		isContinuing = context.step();
		
		rosComm.publishKeys(context.getKeyboard());
		isContinuing&= rosComm.executeLoop();
		
		// Update positions etc. here
		env->step();
		env->draw(renderer);
		ups::Colour c0; c0.r = 1.f; c0.g = 0.f; c0.b = 0.f; c0.a = 1.f;
		ups::Colour c1; c1.r = 0.f; c1.g = 1.f; c1.b = 0.f; c1.a = 1.f;
		renderer.drawTestQuad(c0, 32.f, 32.f, 64.f, 64.f);
		renderer.drawTestQuad(c1, 32.f, 96.f, 64.f, 64.f);
		//renderer.drawTestQuad(c1, 0.0, 1.0, 1.0, -0.5);
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}