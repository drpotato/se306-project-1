#include "ROSWorld.hpp"
#include "Actor.hpp"
#include "../renderer/Renderer.hpp"

ups::ROSWorld::ROSWorld()
{
}

ups::ROSWorld::~ROSWorld()
{
}

void ups::ROSWorld::step()
{
}

void ups::ROSWorld::draw(Renderer &renderer)
{
	renderer.drawTexQuad(ups::Colour::rgb(0.8f, 0.8f, 1.f, 1.f), "src/se306_p1_pkg/world/worldmaps/worldwalls.bmp", 32.f, 96.f, 512.f, 512.f);
}