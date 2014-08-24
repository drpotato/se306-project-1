#ifndef SE306P1_UPSTAGE_RESOURCEMANAGER_HPP_DEFINED
#define SE306P1_UPSTAGE_RESOURCEMANAGER_HPP_DEFINED

#include "ResourceLoader.hpp"
#include <map>
#include <list>
#include <string>

namespace ups
{
	class Resource;
	typedef std::map<std::string, Resource *> ResMap;
	typedef std::list<std::string> PathList;
	class ResourceManager
	{
	public:
		~ResourceManager();
		static ResourceManager &getInstance();
		
		template<typename t>
		t *fetch(const std::string &resName);
		void add(const std::string &resName, Resource *resource);
		void addPriorityPath(const std::string &path);
		void addFallbackPath(const std::string &path);
		
	private:
		std::string resolvePath(const std::string &resName) const;
		Resource *load(const std::string &resName);
		
		ResourceManager();
		
		PathList _paths;
		ResourceLoader _loader;
		ResMap _resources;
	};
	
	inline ResourceManager &ResourceManager::getInstance()
	{
		static ResourceManager *instance;
		
		if (!instance)
		{
			instance = new ResourceManager();
		}
		
		return *instance;
	}
	
	template<typename t>
	inline t *ResourceManager::fetch(const std::string &resName)
	{
		ResMap::iterator it = _resources.find(resName);
		if (it == _resources.end())
		{
			return load(resName);
		}
		
		return it->second;
	}
}

#endif // #ifndef SE306P1_UPSTAGE_RESOURCEMANAGER_HPP_DEFINED