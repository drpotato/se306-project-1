#include "../ResourceLoader.hpp"
#include "../Resource.hpp"

#include <cstdio>

namespace ups
{
	// Version which is told a type - can be used to force non-default loading.
	template<>
	Resource *ResourceLoader::loadFrom<ResourceLoader::RL_LT_UpstageEnv>(const std::string &filePath) const
	{
		std::printf("Looking at RL_LT_UpstageEnv\n");
		return new Resource();
	}
}