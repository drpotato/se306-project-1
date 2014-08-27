#include "Actor.hpp"
#include "renderer/Renderer.hpp"

ups::Actor::Actor():
	_x(0.f),
	_y(0.f),
	_theta(0.f),
	_vLinear(0.f),
	_vAngular(0.f)
{
}

ups::Actor::~Actor()
{
}

void ups::Actor::update(float x, float y, float theta, float vLinear, float vAngular)
{
	_x = x;
	_y = y;
	_theta = theta;
	_vLinear = vLinear;
	_vAngular = vAngular;
}

void ups::Actor::draw(ups::Renderer &renderer)
{
	ups::Colour col;
	col.r = 0.8;
	col.g = 0.5;
	col.b = 0.2;
	col.a = 1.0;
	
	float scale = 10.f / renderer.getWidth();
	float w = 10.f;
	float h = 10.f;
	renderer.drawTestQuad(col, _x * scale - w * 0.5f, _y * scale - h * 0.5f, w, h);
}