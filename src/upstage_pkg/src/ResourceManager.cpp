#include "ResourceManager.hpp"
#include "Resource.hpp"
#include "Util.hpp"

ups::ResourceManager::ResourceManager()
{
	_paths.push_back(std::string());
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

void ups::ResourceManager::addPriorityPath(const std::string &path)
{
	_paths.push_front(path);
}

void ups::ResourceManager::addFallbackPath(const std::string &path)
{
	_paths.push_back(path);
}

std::string ups::ResourceManager::resolvePath(const std::string &resName) const
{
	std::string resolvedPath;
	for (ups::PathList::const_iterator it = _paths.begin(); it != _paths.end(); ++it)
	{
		std::string testPath = *it + "/" + resName;
		
		if (canRead(testPath))
		{
			resolvedPath = testPath;
			break;
		}
	}
	
	UPS_LOGF("Searched for \"%s\", found \"%s\"", resName.c_str(), resolvedPath.c_str());
	
	return resolvedPath;
}

ups::Resource *ups::ResourceManager::load(const std::string &resName)
{
	std::string resourcePath = resolvePath(resName);
	if (resourcePath == "") 
	{
		return 0;
	}
	
	ups::Resource *resource = _loader.loadFrom(resourcePath);
	add(resName, resource);
	return resource;
}