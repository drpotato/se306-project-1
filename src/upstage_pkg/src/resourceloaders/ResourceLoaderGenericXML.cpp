#include "../ResourceLoader.hpp"
#include "../Resource.hpp"
#include "../Debug.hpp"
#include "../xml/XML.hpp"

#include <cstdio>

namespace ups
{
	// Version which is told a type - can be used to force non-default loading.
	template<>
	Resource *ResourceLoader::loadFrom<ResourceLoader::RL_LT_GenericXML>(const std::string &filePath) const
	{
		UPS_LOGF("Looking at RL_LT_GenericXML for %s", filePath.c_str());
		
		std::FILE *f = std::fopen(filePath.c_str(), "rb");
		PointerUnique<XML> xml = XML::fromFile(f);
		fclose(f);
		
		xml->print();

		return new Resource();
	}
}