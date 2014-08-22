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

template<typename t>
t *ups::ResourceManager::fetch(const std::string &resName) const
{
	ups::ResMap::iterator it = _resources.find(resName);
	
	if (it == _resources.end())
	{
		return load(resName);
	}
	
	return *it;
}

void ups::ResourceManager::add(const std::string &resName, Resource *resource)
{
	_resources[resName] = resource;
}

std::string ups::ResourceManager::resolvePath(const std::string &resName) const
{
	return resName;
}

ups::Resource *ups::ResourceManager::load(const std::string &resName) const
{
	std::string resourcePath = resolvePath(resName);
	if (resourcePath == "") 
	{
		return 0;
	}
	
	
}