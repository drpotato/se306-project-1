#include "Context.hpp"
#include <iostream>

int main(int argc, char **argv)
{
	ups::Context &context = ups::Context::getContext();
	std::cout << "Hello World!" << std::endl;
	
	while (context.step())
	{
		// Update positions etc. here
		
		context.drawStart();
		// Draw code here
		context.drawEnd();
	}
	return 0;
}