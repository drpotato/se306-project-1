#include "Context.hpp"
#include "Renderer.hpp"
#include <iostream>

int main(int argc, char **argv)
{
	ups::Context &context = ups::Context::getContext();
	ups::Renderer renderer(context);
	std::cout << "Hello World!" << std::endl;
	
	while (context.step())
	{
		// Update positions etc. here
		renderer.setEnvClearColour(1.f, 0.5f, 0.f);
		
		context.drawStart();
		renderer.render();
		context.drawEnd();
	}
	return 0;
}