#ifndef SE306P1_UPSTAGE_RESOURCEMANAGER_HPP_DEFINED
#define SE306P1_UPSTAGE_RESOURCEMANAGER_HPP_DEFINED

#include <map>
#include <string>

namespace ups
{
	class Resource;
	typedef std::map<std::string, Resource *> ResMap;
	class ResourceManager
	{
	public:
		~ResourceManager();
		static ResourceManager &getInstance();
		
		template<typename t>
		t *fetch(const std::string &resName) const;
		void add(const std::string &resName, Resource *resource);
		
	private:
		std::string resolvePath(const std::string &resName) const;
		Resource *load(const std::string &resName) const;
		
		ResourceManager();
		
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
}

#endif // #ifndef SE306P1_UPSTAGE_RESOURCEMANAGER_HPP_DEFINED