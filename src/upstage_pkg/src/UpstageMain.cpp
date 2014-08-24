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
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}