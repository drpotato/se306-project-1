#include "Texture.hpp"
#include "Renderer.hpp"
#include "../Util.hpp"

ups::Texture::Texture(ups::Texture::Texture::TextureType tt, unsigned int width, unsigned int height, unsigned int innerWidth, unsigned int innerHeight) :
	_tt(tt),
	_width(width),
	_height(height),
	_isInRenderer(false),
	_data(0)
{
	_innerWidth = (innerWidth) ? innerWidth : potAbove(width);
	_innerHeight = (innerHeight) ? innerHeight : potAbove(height);
}

ups::Texture::~Texture()
{
	delete[] _data;
}

ups::TexHandle &ups::Texture::getRendererHandle(ups::Renderer &renderer)
{
	if (!_isInRenderer)
	{
		_rendererHandle = renderer.makeTexHandle(*this);
		_isInRenderer = true;
	}
	
	return _rendererHandle;
}