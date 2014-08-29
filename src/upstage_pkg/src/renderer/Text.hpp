#ifndef SE306P1_UPSTAGE_TEXT_HPP_DEFINED
#define SE306P1_UPSTAGE_TEXT_HPP_DEFINED

#include "Font.hpp"
#include "Types.hpp"
#include "UltronScale.hpp"
namespace ups
{
	class Renderer;
	
	struct GlyphStamp
	{
		float x;
		float y;
		float w;
		float h;
		float u;
		float v;
	};
	
	class Text : public UltronScale
	{
	public:
		Text(const std::string &text, const Font &font);
		~Text();
		
		void setCentred(bool isCentred);
		void setColour(const ups::Colour &colour);
		void draw(Renderer &renderer);
	private:
		void unfinalise();
		void finalise();
		
		ups::Colour _colour;
		std::vector<GlyphStamp *> _stamps;
		std::string _text;
		const Font &_font;
		float _centredMargin;
		bool _centred;
		bool _finalised;
	};
	
	inline void Text::setCentred(bool isCentred)
	{
		_centred = isCentred;
	}
	
	inline void Text::setColour(const ups::Colour &colour)
	{
		_colour = colour;
	}
}
#endif // #ifndef SE306P1_UPSTAGE_TEXT_HPP_DEFINED