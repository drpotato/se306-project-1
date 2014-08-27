#ifndef SE306P1_UPSTAGE_RENDERERTYPES_HPP_DEFINED
#define SE306P1_UPSTAGE_RENDERERTYPES_HPP_DEFINED

#include <GL/glx.h>

namespace ups
{
	typedef GLuint TexHandle;
	struct Colour
	{
		float r, g, b, a;
		
		static Colour rgb(float r, float g, float b, float a = 1.f);
	};
	
	inline Colour Colour::rgb(float r, float g, float b, float a)
	{
		Colour outCol;
		outCol.r = r;
		outCol.g = g;
		outCol.b = b;
		outCol.a = a;
		
		return outCol;
	}
}

#endif // #ifndef SE306P1_UPSTAGE_RENDERERTYPES_HPP_DEFINED