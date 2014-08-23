#include "../ResourceLoader.hpp"
#include "../Resource.hpp"
#include "../Debug.hpp"

#include <cstdio>

namespace ups
{
	// Version which is told a type - can be used to force non-default loading.
	template<>
	Resource *ResourceLoader::loadFrom<ResourceLoader::RL_LT_StageWorld>(const std::string &filePath) const
	{
		UPS_LOGF("Looking at RL_LT_StageWorld for %s", filePath.c_str());

		return new Resource();
	}
}