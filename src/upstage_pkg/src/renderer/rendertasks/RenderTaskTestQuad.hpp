#ifdef PP_RT_MODE_ENUMNAME
RT_TestQuad, // The enum name
#endif

#ifdef PP_RT_MODE_UNIONMEMBER
RenderTaskTestQuad testQuad; // A member in the containing union, should be of this RenderTask type
#endif

#ifdef PP_RT_MODE_DEFINE
// The class info
struct RenderTaskTestQuad
{
	Colour col;
	float x, y, w, h;
};
#endif