#include "Text.hpp"
#include "Renderer.hpp"
#include "../Debug.hpp"
#include "../Util.hpp"

ups::Text::Text(const std::string &text, const Font &font) :
	_text(text),
	_font(font),
	_centredMargin(0.f),
	_colour(ups::Colour::rgb(1.f,1.f,1.f)),
	_centred(false),
	_finalised(false)
{
}

ups::Text::~Text()
{
	unfinalise();
}

void ups::Text::draw(Renderer &renderer)
{
	resize(renderer);
	_finalised = false;
	finalise();
	
	float baseX = getL();
	float baseY = getU();
	
	if (_centred)
	{
		baseX += _centredMargin;
	}
	
	for (std::vector<GlyphStamp *>::const_iterator it = _stamps.begin(); it != _stamps.end(); ++it)
	{
		GlyphStamp &glyphStamp = **it;
		renderer.drawTextGlyph(_colour, _font.getTexName(), baseX + glyphStamp.x, baseY + glyphStamp.y, glyphStamp.w, glyphStamp.h, glyphStamp.u, glyphStamp.v);
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
	
	long maxX = 0;
	long cursorX = 0;
	long cursorY = 0;
	long width = getR() - getL();
	
	long borderSize = 1;
	
	for (long pos = 0; pos <= _text.length(); ++pos)
	{
		
		// Each word boundary, if the line doesn't exceed the width, populate the list until the boundary - it's locked-in.
		if (isWhitespace(_text[pos]) || pos == _text.length())
		{
			cursorX = lastWordX;
			for (; lastWordPos < pos; ++lastWordPos)
			{
				const FontChar &previousFontChar = _font.getChar(_text[lastWordPos]);
				
				// Add the glyph stamp
				GlyphStamp *glyphStamp = new GlyphStamp();
				glyphStamp->x = cursorX + previousFontChar.xOff;
				glyphStamp->y = cursorY + _font.getLineHeight() - previousFontChar.h - previousFontChar.yOff;
				glyphStamp->w = previousFontChar.w;
				glyphStamp->h = previousFontChar.h;
				glyphStamp->u = previousFontChar.x;
				glyphStamp->v = previousFontChar.y;
				
				// Expand the border
				glyphStamp->x -= borderSize;
				glyphStamp->y -= borderSize;
				glyphStamp->w += borderSize * 2;
				glyphStamp->h += borderSize * 2;
				glyphStamp->u -= borderSize;
				glyphStamp->v -= borderSize;
				
				// Add to the stamp list
				_stamps.push_back(glyphStamp);
				
				cursorX += previousFontChar.xAdv;
				
				long maxXCandidate = cursorX + previousFontChar.w;
				maxX = maxX > maxXCandidate ? maxX : maxXCandidate;
			}
			
			if (pos < _text.length())
			{
				cursorX += _font.getChar(_text[pos]).xAdv;
				lastWordPos = pos;
				lastWordX = cursorX;
			}
		}
		else
		{
			// If the width is exceeded, roll back to the last word boundary and insert a newline.
			// Orrrrr.... if it's too wide, just split the word
			if (cursorX >= width)
			{
				// If we're already as wide as we can be ...
				if (lastWordX == 0)
				{
					// Pretend we have a newline just before this character
					lastWordPos = pos - 1;
					lastWordX = 0;
					cursorY -= _font.getLineHeight();
				}
				else
				{
					// We rewind to the last word boundary, insert a newline, and let the main loop take care of it.
					pos = lastWordPos++; // Increment last word pos to avoid getting stuck.
					lastWordX = 0;
					cursorX = 0;
					cursorY -= _font.getLineHeight();
					continue;
				}
			}
			
			// Shift along
			cursorX += _font.getChar(_text[pos]).xAdv;
		}
	}
	_centredMargin = 0.5f * (width - maxX);
	_finalised = true;
}