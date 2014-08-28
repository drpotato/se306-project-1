#include "Text.hpp"
#include "Renderer.hpp"
#include "../Debug.hpp"

ups::Text::Text(const std::string text, const Font &font, long width) :
	_text(text),
	_font(font),
	_finalised(false),
	_width(width)
{
}

ups::Text::~Text()
{
	unfinalise();
}

void ups::Text::draw(Renderer &renderer)
{
	for (std::vector<GlyphStamp *>::const_iterator it = _stamps.begin(); it != _stamps.end(); ++it)
	{
		UPS_LOGF("Stamp: x=%ld, y=%ld | w=%ld, h=%ld | u=%ld, v=%ld",
			(**it).x,
			(**it).y,
			(**it).w,
			(**it).h,
			(**it).u,
			(**it).v);
	}
}

void ups::Text::unfinalise()
{
	for (std::vector<GlyphStamp *>::const_iterator it = _stamps.begin(); it != _stamps.end(); ++it)
	{
		delete *it;
	}
	_stamps.clear();
}

void ups::Text::finalise()
{
	for (std::string::iterator it = _text.begin(); it != _text.end(); ++it)
	{
		const FontChar &fontChar = _font.getChar(*it);
	}
}