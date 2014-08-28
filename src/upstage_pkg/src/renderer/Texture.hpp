#ifndef SE306P1_UPSTAGE_TEXTURE_HPP_DEFINED
#define SE306P1_UPSTAGE_TEXTURE_HPP_DEFINED

#include "Types.hpp"
#include "../Resource.hpp"
#include <vector>

namespace ups
{
	class Renderer;
	class Texture : public Resource
	{
	public:
		enum TextureType
		{
			TT_RGBA8,
			TT_EMPTY
		};
		
		Texture(TextureType tt, unsigned int width, unsigned int height, unsigned int innerWidth = 0, unsigned int innerHeight = 0);
		~Texture();
		void setData(unsigned char *data);
		
		TexHandle &getRendererHandle(Renderer &renderer);
		const TextureType &getTextureType() const;
		unsigned int getWidth() const;
		unsigned int getHeight() const;
		unsigned int getInnerWidth() const;
		unsigned int getInnerHeight() const;
		float getMaxTexX() const;
		float getMaxTexY() const;
		const unsigned char *getData() const;
		
	private:
		TextureType _tt;
		unsigned int _innerWidth;
		unsigned int _innerHeight;
		unsigned int _width;
		unsigned int _height;
		unsigned char *_data;
		
		TexHandle _rendererHandle;
		bool _isInRenderer;
	};
	
	inline const Texture::TextureType &Texture::getTextureType() const
	{
		return _tt;
	}
	
	inline unsigned int Texture::getWidth() const
	{
		return _width;
	}
	
	inline unsigned int Texture::getHeight() const
	{
		return _height;
	}
	
	inline unsigned int Texture::getInnerWidth() const
	{
		return _innerWidth;
	}
	
	inline unsigned int Texture::getInnerHeight() const
	{
		return _innerHeight;
	}
	
	inline float Texture::getMaxTexX() const
	{
		return static_cast<float>(_innerWidth) / _width;
	}
	
	inline float Texture::getMaxTexY() const
	{
		return static_cast<float>(_innerHeight) / _height;
	}

	inline void Texture::setData(unsigned char *data)
	{
		_data = data;
	}
	
	inline const unsigned char *Texture::getData() const
	{
		return _data;
	}
}
#endif // #ifndef SE306P1_UPSTAGE_TEXTURE_HPP_DEFINED