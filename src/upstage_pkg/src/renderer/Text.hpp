#ifndef SE306P1_UPSTAGE_TEXT_HPP_DEFINED
#define SE306P1_UPSTAGE_TEXT_HPP_DEFINED

#include "Font.hpp"
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
		
		void draw(Renderer &renderer);
	private:
		void unfinalise();
		void finalise();
		
		std::vector<GlyphStamp *> _stamps;
		std::string _text;
		const Font &_font;
		bool _finalised;
	};
}
#endif // #ifndef SE306P1_UPSTAGE_TEXT_HPP_DEFINED