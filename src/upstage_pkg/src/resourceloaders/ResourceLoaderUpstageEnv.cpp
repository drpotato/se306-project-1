#include "../Debug.hpp"
#include "../ResourceLoader.hpp"
#include "../Resource.hpp"
#include "../UpstageEnvironment.hpp"
#include "../xml/XML.hpp"

#include <cstdio>

namespace ups
{
	template<>
	Resource *ResourceLoader::loadFrom<ResourceLoader::RL_LT_UpstageEnv>(const std::string &filePath) const
	{
		UpstageEnvironment *env = new UpstageEnvironment();
		UPS_LOGF("Looking at RL_LT_UpstageEnv for %s", filePath.c_str());
		
		// Load the raw data from the XML
		std::FILE *f = std::fopen(filePath.c_str(), "rb");
		PointerUnique<XML> xml = XML::fromFile(f);
		fclose(f);
		
		// Process the "env" node of the XML
		const XML &xmlEnv = xml->getChild("env");
		
		// Env|BG Colour
		const XML &xmlEnvBGCol = xmlEnv.getChild("bgcol");
		Colour bgCol;
		bgCol.r = xmlEnvBGCol.getAttr<float>("r", 0.f);
		bgCol.g = xmlEnvBGCol.getAttr<float>("g", 0.f);
		bgCol.b = xmlEnvBGCol.getAttr<float>("b", 0.f);
		bgCol.a = xmlEnvBGCol.getAttr<float>("a", 1.f);
		env->setBGColour(bgCol);
		
		xml->print();
		
		
		return env;
	}
}