#ifndef SE306P1_UPSTAGE_RESOURCELOADER_HPP_DEFINED
#define SE306P1_UPSTAGE_RESOURCELOADER_HPP_DEFINED

#include <string>

namespace ups
{
	class Resource;
	
	class ResourceLoader
	{
	public:
		enum LoaderType
		{
			RL_LT_GenericXML,
			RL_LT_UpstageEnv,
			RL_LT_StageWorld
		};
		
		ResourceLoader();
		~ResourceLoader();

		// Version which is told a type - can be used to force non-default loading.
		template<LoaderType t>
		Resource *loadFrom(const std::string &filePath) const;
		
		// Version which attempts to guess the type
		Resource *loadFrom(const std::string &filePath) const;
		
	private:
	};
}

#endif // #ifndef SE306P1_UPSTAGE_RESOURCELOADER_HPP_DEFINED