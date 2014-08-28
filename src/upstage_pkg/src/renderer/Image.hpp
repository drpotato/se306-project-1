#ifndef SE306P1_UPSTAGE_IMAGE_HPP_DEFINED
#define SE306P1_UPSTAGE_IMAGE_HPP_DEFINED

#include "Types.hpp"
#include "UltronScale.hpp"
#include <string>

namespace ups
{
	class Renderer;
	class Image : public UltronScale
	{
	public:
		Image(const std::string &texName);
		void draw(Renderer &renderer);
		
	private:
		std::string _texName;
		Colour _colour;
	};
	
	
	Image::Image(const std::string &texName) :
		_texName(texName),
		_colour(ups::Colour::rgb(1.f,1.f,1.f))
	{
	}
}
#endif // #ifndef SE306P1_UPSTAGE_IMAGE_HPP_DEFINED