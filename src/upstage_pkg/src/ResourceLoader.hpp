#ifndef SE306P1_UPSTAGE_RESOURCELOADER_HPP_DEFINED
#define SE306P1_UPSTAGE_RESOURCELOADER_HPP_DEFINED

#include <string>

namespace ups
{
	class Resource;
	
	template<typename t>
	class ResourceLoader
	{
	public:
		ResourceLoader();
		~ResourceLoader();

		Resource *loadFrom(const std::string &resName);
		
	private:
		
	};
}

#endif // #ifndef SE306P1_UPSTAGE_RESOURCELOADER_HPP_DEFINED