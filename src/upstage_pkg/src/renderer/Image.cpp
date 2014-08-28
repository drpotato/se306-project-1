#include "Image.hpp"
#include "Renderer.hpp"

void ups::Image::draw(ups::Renderer &renderer)
{
	resize(renderer);
	
	renderer.drawTexQuad(_colour, _texName, getL(), getD(), getR() - getL(), getU() - getD());
	//getL()
}