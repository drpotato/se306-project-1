#include "ResourceLoader.hpp"
#include "Resource.hpp"
#include "Util.hpp"
#include <map>

namespace
{
	typedef std::map<std::string, ups::ResourceLoader::LoaderType> TypeMap;
	TypeMap typeMap;
	
	void initTypeMap();
}

ups::ResourceLoader::ResourceLoader()
{
}

ups::ResourceLoader::~ResourceLoader()
{
}

// Attempt to guess the type and run the appropriate loader
ups::Resource *ups::ResourceLoader::loadFrom(const std::string &filePath) const
{
	initTypeMap();
	
	std::string extension = getFileExtension(filePath);
	TypeMap::iterator it = typeMap.find(extension);
	
	if (it == typeMap.end())
	{
		return ups::ResourceLoader::loadFrom<RL_LT_GenericXML>(filePath);
	}
	
	switch (it->second)
	{
	case RL_LT_BMP:
		return ups::ResourceLoader::loadFrom<RL_LT_BMP>(filePath);
	case RL_LT_GenericXML:
		return ups::ResourceLoader::loadFrom<RL_LT_GenericXML>(filePath);
	case RL_LT_UpstageEnv:
		return ups::ResourceLoader::loadFrom<RL_LT_UpstageEnv>(filePath);
	case RL_LT_StageWorld:
		return ups::ResourceLoader::loadFrom<RL_LT_StageWorld>(filePath);
	default:
		return ups::ResourceLoader::loadFrom<RL_LT_GenericXML>(filePath);
	}
}

namespace
{
	inline void initTypeMap()
	{
		static bool initialised = false;
		if (initialised)
		{
			return;
		}

		typeMap["bmp"] = ups::ResourceLoader::RL_LT_BMP;
		typeMap["xml"] = ups::ResourceLoader::RL_LT_GenericXML;
		typeMap["unv"] = ups::ResourceLoader::RL_LT_UpstageEnv;
		typeMap["world"] = ups::ResourceLoader::RL_LT_StageWorld;
		
		initialised = true;
	}
}