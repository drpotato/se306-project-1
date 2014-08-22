#include "ResourceLoader.hpp"
#include "Resource.hpp"

ups::ResourceLoader::ResourceLoader()
{
}

ups::ResourceLoader::~ResourceLoader()
{
}

// Attempt to guess the type and run the appropriate loader
ups::Resource *ups::ResourceLoader::loadFrom(const std::string &filePath) const
{
	return ups::ResourceLoader::loadFrom<RL_LT_UpstageEnv>(filePath);
}