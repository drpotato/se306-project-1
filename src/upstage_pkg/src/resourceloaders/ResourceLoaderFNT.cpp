#include "../Debug.hpp"
#include "../ResourceLoader.hpp"
#include "../Resource.hpp"
#include "../renderer/Font.hpp"
#include "../xml/XML.hpp"

#include <cstdio>

namespace ups
{
	template<>
	Resource *ResourceLoader::loadFrom<ResourceLoader::RL_LT_FNT>(const std::string &filePath) const
	{
		Font *font = new Font();
		UPS_LOGF("Looking at RL_LT_FNT for %s", filePath.c_str());
		
		// Load the raw data from the FNT (it's XML)
		std::FILE *f = std::fopen(filePath.c_str(), "rb");
		PointerUnique<XML> xml = XML::fromFile(f);
		fclose(f);
		
		// Process the "info" node of the FNT
		const XML &xmlInfo = xml->getChild("info");
		font->setFaceName(xmlInfo.getAttr<const char *>("face", "unknown font name"));
		font->setNativeSize(xmlInfo.getAttr<long>("size", 32));
		
		// Process the "common" node of the FNT
		const XML &xmlCommon = xml->getChild("common");
		font->setLineHeight(xmlCommon.getAttr<long>("lineHeight", font->getNativeSize()));
		
		// We only support fonts with one page
		long nPages = xmlCommon.getAttr<long>("pages", 0);
		UPS_ASSERTF(nPages == 1, "Only fonts with one texture page are supported. Fix %s - it's got %ld pages.", filePath.c_str(), nPages);
		
		// Get the font texture from the page.
		const XML &xmlPages = xml->getChild("pages");
		const XML &xmlPage = xmlPages.getChild("page");
		font->setTexName(xmlPage.getAttr<const char *>("file", ""));
		UPS_LOGF("Loading font \"%s\" with size %ld, lh %ld. Tex = \"%s\"",
			font->getFaceName().c_str(),
			font->getNativeSize(),
			font->getLineHeight(),
			font->getTexName().c_str());
		
		// Load chars
		const XML &xmlChars = xml->getChild("chars");
		std::vector<const XML *> children = xmlChars.getChildren("char");
		
		for (std::vector<const XML *>::iterator it = children.begin(); it != children.end(); ++it)
		{
			const XML *xmlChar = *it;
			long charID   = xmlChar->getAttr<long>("id",       0);
			long charX    = xmlChar->getAttr<long>("x",        0);
			long charY    = xmlChar->getAttr<long>("y",        0);
			long charW    = xmlChar->getAttr<long>("width",    0);
			long charH    = xmlChar->getAttr<long>("height",   0);
			long charXOff = xmlChar->getAttr<long>("xoffset",  0);
			long charYOff = xmlChar->getAttr<long>("yoffset",  0);
			long charXAdv = xmlChar->getAttr<long>("xadvance", 0);
			
			font->addChar(
				charID,
				charX,
				charY,
				charW,
				charH,
				charXOff,
				charYOff,
				charXAdv);
		}
		
		// Env|BG Colour
		/*
		const XML &xmlEnvBGCol = xmlEnv.getChild("bgcol");
		Colour bgCol;
		bgCol.r = xmlEnvBGCol.getAttr<float>("r", 0.f);
		bgCol.g = xmlEnvBGCol.getAttr<float>("g", 0.f);
		bgCol.b = xmlEnvBGCol.getAttr<float>("b", 0.f);
		bgCol.a = xmlEnvBGCol.getAttr<float>("a", 1.f);
		env->setBGColour(bgCol);
		*/
		
		//xml->print();
		
		
		return font;
	}
}