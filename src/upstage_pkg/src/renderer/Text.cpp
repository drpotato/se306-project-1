#include "Text.hpp"
#include "Renderer.hpp"
#include "../Debug.hpp"
#include "../Util.hpp"

ups::Text::Text(const std::string text, const Font &font, float x, float y, float width) :
	_text(text),
	_font(font),
	_finalised(false),
	_width(width),
	_x(x),
	_y(y)
{
}

ups::Text::~Text()
{
	unfinalise();
}

void ups::Text::draw(Renderer &renderer)
{
	finalise();
	ups::Colour colour = ups::Colour::rgb(1.f,1.f,1.f);
	for (std::vector<GlyphStamp *>::const_iterator it = _stamps.begin(); it != _stamps.end(); ++it)
	{
		GlyphStamp &glyphStamp = **it;
		renderer.drawTextGlyph(colour, _font.getTexName(), _x + glyphStamp.x, _y + glyphStamp.y, glyphStamp.w, glyphStamp.h, glyphStamp.u, glyphStamp.v);
		/*UPS_LOGF("Stamp: x=%lf, y=%lf | w=%lf, h=%lf | u=%lf, v=%lf",
			(**it).x,
			(**it).y,
			(**it).w,
			(**it).h,
			(**it).u,
			(**it).v);*/
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
	if (_finalised) return;
	unfinalise();
	
	long lastWordPos = 0;
	long lastWordX = 0;
	
	long cursorX = 0;
	long cursorY = 0;
	
	long borderSize = 1;
	
	for (long pos = 0; pos < _text.length(); ++pos)
	{
		const FontChar &fontChar = _font.getChar(_text[pos]);
		
		// Each word boundary, if the line doesn't exceed the width, populate the list until the boundary - it's locked-in.
		if (isWhitespace(_text[pos]) || pos == _text.length() - 1)
		{
			cursorX = lastWordX;
			for (; lastWordPos < pos; ++lastWordPos)
			{
				const FontChar &previousFontChar = _font.getChar(_text[pos]);
				cursorX += previousFontChar.xAdv;
			}
			
			cursorX += fontChar.xAdv;
			lastWordPos = pos;
			lastWordX = cursorX;
		}
		else
		{
			// If the width is exceeded, roll back to the last word boundary and insert a newline.
			// Orrrrr.... if it's too wide, just split the word
			if (cursorX >= _width)
			{
				// If we're already as wide as we can be ...
				if (lastWordX == 0)
				{
					// Pretend we have a newline just before this character
					lastWordPos = pos - 1;
					lastWordX = 0;
					cursorY -= _font.getLineHeight();
				}
				else // We rewind to the last word boundary, insert a newline, and let the main loop take care of it.
				{
					pos = lastWordPos;
					cursorX = 0;
					cursorY -= _font.getLineHeight();
					break;
				}
			}
			
			// Add the glyph stamp
			GlyphStamp *glyphStamp = new GlyphStamp();
			glyphStamp->x = cursorX + fontChar.xOff;
			glyphStamp->y = cursorY + fontChar.h - fontChar.yOff;// + fontChar.yOff;
			glyphStamp->w = fontChar.w;
			glyphStamp->h = fontChar.h;
			glyphStamp->u = fontChar.x;
			glyphStamp->v = fontChar.y;
			
			// Expand the border
			glyphStamp->x -= borderSize;
			glyphStamp->y -= borderSize;
			glyphStamp->w += borderSize * 2;
			glyphStamp->h += borderSize * 2;
			glyphStamp->u -= borderSize;
			glyphStamp->v -= borderSize;
			
			// Add to the stamp list
			_stamps.push_back(glyphStamp);
			
			// Shift along
			cursorX += fontChar.xAdv;
		}
	}
	_finalised = true;
}