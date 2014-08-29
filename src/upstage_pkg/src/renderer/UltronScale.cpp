#include "UltronScale.hpp"
#include "Renderer.hpp"

void ups::UltronScale::resize(const Renderer &renderer)
{
	float w = (float)renderer.getWidth();
	float h = (float)renderer.getHeight();
	
	_left.updateBounds  (0.f, w);
	_right.updateBounds (0.f, w);
	_up.updateBounds    (0.f, h);
	_down.updateBounds  (0.f, h);
}