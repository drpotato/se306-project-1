#include "XML.hpp"
#include "../Util.hpp"

ups::PointerUnique<ups::XML> ups::XML::fromFile(std::FILE *f)
{
	std::vector<char> buffer;
	int fChr;
	bool rootNodeStarted = false;
	std::vector<XML *> nodeHierarchy;
	
	while ((fChr = std::fgetc(f)) != EOF)
	{
		// Ignore whitespace until the root node is found.
		if (!isWhitespace(fChr) || rootNodeStarted)
		{
			
		}
		
		buffer.push_back(fChr);
	}
	
	return new XML("test");
}

ups::XML::XML(const std::string &name):
	_name(name)
{
	std::printf("XML CONSTRUCTOR FOR %s\n", _name.c_str());
}

ups::XML::~XML()
{
	std::printf("XML DESTRUCTOR FOR %s\n", _name.c_str());
}
		
void ups::XML::setContent(const std::string &content)
{
	
}

void ups::XML::addChild(XML *child)
{
	_children.push_back(PointerUnique<XML>(child));
}

void ups::XML::print(int indentDepth) const
{
	std::string indentString(indentDepth, '\t');
	std::printf("%s+XML(%s)", indentString.c_str(), _name.c_str());
	for (AttrMap::const_iterator it = _attributes.begin(); it != _attributes.end(); ++it)
	{
		std::printf(" \"%s\"->\"%s\"", it->first.c_str(), it->second.c_str());
	}
	std::printf("\n");
	
	for (ChildList::const_iterator it = _children.begin(); it != _children.end(); ++it)
	{
		(*it)->print(indentDepth + 1);
	}
	
	std::printf("%s-XML(%s)\n", indentString.c_str(), _name.c_str());
}