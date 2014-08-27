#ifdef PP_RT_MODE_ENUMNAME
RT_TexQuad, // The enum name
#endif

#ifdef PP_RT_MODE_UNIONMEMBER
RenderTaskTexQuad texQuad; // A member in the containing union, should be of this RenderTask type
#endif

#ifdef PP_RT_MODE_DEFINE
// The class info
struct RenderTaskTexQuad
{
	Colour col;
	TexHandle texHandle;
	float x, y, w, h;
	float maxU, maxV;
};
#endif