#ifdef PP_RT_MODE_ENUMNAME
RT_Clear, // The enum name
#endif

#ifdef PP_RT_MODE_UNIONMEMBER
RenderTaskClear clear; // A member in the containing union, should be of this RenderTask type
#endif

#ifdef PP_RT_MODE_DEFINE
// The class info
struct RenderTaskClear
{
	float r;
	float g;
	float b;
};
#endif