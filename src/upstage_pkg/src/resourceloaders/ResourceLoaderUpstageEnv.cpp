#include "../Debug.hpp"
#include "../ResourceLoader.hpp"
#include "../ResourceManager.hpp"
#include "../Resource.hpp"
#include "../UpstageEnvironment.hpp"
#include "../renderer/Image.hpp"
#include "../renderer/Text.hpp"
#include "../renderer/UltronScale.hpp"
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
		
		// Oh boy, the piece de resistance!
		const XML &xmlLayout = xml->getChild("layout");
		const XML::ChildList &layoutChildren = xmlLayout.getAllChildren();
		
		for (XML::ChildList::const_iterator it = layoutChildren.begin(); it != layoutChildren.end(); ++it)
		{
			XML &child = **it;
			const std::string &name = child.getName();
			
			UltronScale *newUS = 0;
			
			// Subclass-specific stuff
			if (name == "image")
			{
				newUS = new Image(child.getAttr<const char *>("file", ""));
			}
			else if (name == "text")
			{
				// Made slightly trickier because we have to get the font first. If the font's missing, no text child.
				Font *font = ResourceManager::getInstance().fetch<Font>(child.getAttr<const char *>("font", ""));
				
				// No font? NO ENTRY!
				if (!font) break;
				
				// Create the text
				Text *newText = new Text(child.getAttr<const char *>("text", ""), *font);
				
				// Centred?
				newText->setCentred(child.getAttr<long>("centred", 0));
				
				// Colour?
				float colR = child.getAttr<float>("colour_r", 1.f);
				float colG = child.getAttr<float>("colour_g", 1.f);
				float colB = child.getAttr<float>("colour_b", 1.f);
				float colA = child.getAttr<float>("colour_a", 1.f);
				ups::Colour colour = ups::Colour::rgb(colR, colG, colB, colA);
				newText->setColour(colour);
				
				// Set the UltronScale pointer to our new object
				newUS = newText;
			}

			if (newUS)
			{
				// Common UltronScale stuff
				newUS->setOffsetsL(child.getAttr<float>("l_abs", 0.f), child.getAttr<float>("l_rel", 0.f));
				newUS->setOffsetsR(child.getAttr<float>("r_abs", 0.f), child.getAttr<float>("r_rel", 1.f));
				newUS->setOffsetsD(child.getAttr<float>("d_abs", 0.f), child.getAttr<float>("d_rel", 0.f));
				newUS->setOffsetsU(child.getAttr<float>("u_abs", 0.f), child.getAttr<float>("u_rel", 1.f));
				
				env->addUltronScale(newUS);
			}
		}
		
		// This is for debugging - it's not mission-critical.
		xml->print();
		
		return env;
	}
}