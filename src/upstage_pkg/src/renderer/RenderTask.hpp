#ifndef SE306P1_UPSTAGE_RENDERTASK_HPP_DEFINED
#define SE306P1_UPSTAGE_RENDERTASK_HPP_DEFINED

#include "Types.hpp"

// If you don't like macros, this may not be the header for you.
// The advantage of this is that you don't have to maintain 3 lists,
// instead you maintain a single list in RenderTaskList.inc

namespace ups
{
	#define PP_RT_MODE_DEFINE
	#include "RenderTaskList.inc"
	#undef  PP_RT_MODE_DEFINE
	
	struct RenderTask
	{
		enum
		{
			#define PP_RT_MODE_ENUMNAME
			#include "RenderTaskList.inc"
			#undef  PP_RT_MODE_ENUMNAME
			_RT_NTypes
		} type;
		
		union
		{
			#define PP_RT_MODE_UNIONMEMBER
			#include "RenderTaskList.inc"
			#undef  PP_RT_MODE_UNIONMEMBER
		};
	};
}

#endif // #ifndef SE306P1_UPSTAGE_RENDERTASK_HPP_DEFINED