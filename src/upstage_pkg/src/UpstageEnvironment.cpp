#include "UpstageEnvironment.hpp"

ups::UpstageEnvironment::UpstageEnvironment()
{
	_bgColour.r = 1.0f; _bgColour.g = 0.0f; _bgColour.b = 1.0f; _bgColour.a = 1.f;
}

void ups::UpstageEnvironment::step()
{
}

void ups::UpstageEnvironment::draw(ups::Renderer &renderer)
{
	renderer.setEnvClearColour(_bgColour);
	
	for (UltronVector::iterator it = _elements.begin(); it != _elements.end(); ++it)
	{
		(**it).draw(renderer);
	}
}

void ups::UpstageEnvironment::setBGColour(const ups::Colour &bgColour)
{
	_bgColour = bgColour;
}

void ups::UpstageEnvironment::addUltronScale(ups::UltronScale *element)
{
	_elements.push_back(element);
}