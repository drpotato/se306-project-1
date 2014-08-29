#ifndef SE306P1_UPSTAGE_FONT_HPP_DEFINED
#define SE306P1_UPSTAGE_FONT_HPP_DEFINED

#include "Texture.hpp"
#include "../Resource.hpp"
#include <map>
#include <string>

namespace ups
{
	class Renderer;
	
	struct FontChar
	{
		long id;
		long x;
		long y;
		long w;
		long h;
		long xOff;
		long yOff;
		long xAdv;
	};
	
	class Font : public Resource
	{
	public:
		Font();
		~Font();
		
		const std::string &getFaceName() const;
		const std::string &getTexName() const;
		long getNativeSize() const;
		long getLineHeight() const;
		
		void setFaceName(const std::string &faceName);
		void setTexName(const std::string &texName);
		void setNativeSize(long nativeSize);
		void setLineHeight(long lineHeight);
		
		void addChar(long id, long x, long y, long w, long h, long xOff, long yOff, long xAdv);
		const FontChar &getChar(long id) const;
	private:
		std::string _faceName;
		std::string _texName;
		
		std::map<long, FontChar> _chars;
		long _nativeSize;
		long _lineHeight;
	};
	
		
	inline const std::string &Font::getFaceName() const
	{
		return _faceName;
	}
	
	inline const std::string &Font::getTexName() const
	{
		return _texName;
	}
	
	inline long Font::getNativeSize() const
	{
		return _nativeSize;
	}
	
	inline long Font::getLineHeight() const
	{
		return _lineHeight;
	}
	
	inline void Font::setFaceName(const std::string &faceName)
	{
		_faceName = faceName;
	}
	
	inline void Font::setTexName(const std::string &texName)
	{
		_texName = texName;
	}
	
	inline void Font::setNativeSize(long nativeSize)
	{
		_nativeSize = nativeSize;
	}
	
	inline void Font::setLineHeight(long lineHeight)
	{
		_lineHeight = lineHeight;
	}
	
	inline void Font::addChar(long id, long x, long y, long w, long h, long xOff, long yOff, long xAdv)
	{
		FontChar      fontChar;
		fontChar.id    =    id;
		fontChar.x     =     x;
		fontChar.y     =     y;
		fontChar.w     =     w;
		fontChar.h     =     h;
		fontChar.xOff  =  xOff;
		fontChar.yOff  =  yOff;
		fontChar.xAdv  =  xAdv;
		_chars[id]  = fontChar;
	}
	
	inline const FontChar &Font::getChar(long id) const
	{
		std::map<long, FontChar>::const_iterator it = _chars.find(id);
		
		// If we don't have the symbol, just pick one.
		if (it == _chars.end())
		{
			it = _chars.begin();
		}
		
		return it->second;
	}
}
#endif // #ifndef SE306P1_UPSTAGE_FONT_HPP_DEFINED