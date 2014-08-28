#ifdef PP_RT_MODE_ENUMNAME
RT_TextGlyph, // The enum name
#endif

#ifdef PP_RT_MODE_UNIONMEMBER
RenderTaskTextGlyph textGlyph; // A member in the containing union, should be of this RenderTask type
#endif

#ifdef PP_RT_MODE_DEFINE
// The class info
struct RenderTaskTextGlyph
{
	Colour col;
	TexHandle texHandle;
	float x, y, w, h;
	float ul,vl,uh,vh;
};
#endif