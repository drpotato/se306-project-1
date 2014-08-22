#include "ResourceManager.hpp"
#include "Resource.hpp"

ups::ResourceManager::ResourceManager()
{
}

ups::ResourceManager::~ResourceManager()
{
	for (ups::ResMap::iterator it = _resources.begin(); it != _resources.end(); ++it)
	{
		delete it->second;
	}
}

void ups::ResourceManager::add(const std::string &resName, Resource *resource)
{
	_resources[resName] = resource;
}

std::string ups::ResourceManager::resolvePath(const std::string &resName) const
{
	return resName;
}

ups::Resource *ups::ResourceManager::load(const std::string &resName)
{
	std::string resourcePath = resolvePath(resName);
	if (resourcePath == "") 
	{
		return 0;
	}
	
	ups::Resource *resource = _loader.loadFrom(resName);
	add(resName, resource);
	return resource;
}