#include "XML.hpp"
#include "../Util.hpp"

ups::PointerUnique<ups::XML> ups::XML::fromFile(std::FILE *f)
{
	// Read everything up-front to make it easier to go backwards and forwards.
	std::vector<char> buffer = readAll(f);
	
	XML *lastNode = 0;
	enum TagType
	{
		XMLTagComment,
		XMLTagDeclaration,
		XMLTagStart,
		XMLTagEnd,
		XMLTagEmpty,
		XMLTagInvalid
	};
	
	for (unsigned long pos = 0; pos < buffer.size(); ++pos)
	{
		// Ignore everything until the first tag
		if (lastNode == 0 && buffer[pos] != '<')
		{
			continue;
		}
		
		// Start of a tag
		if (buffer[pos] == '<')
		{
			TagType tagType = XMLTagInvalid; // ... until we can determine what kind it really is.
			
			// Find the end of the tag
			unsigned long tagEnd = pos;
			while (++tagEnd < buffer.size() && buffer[tagEnd] != '>');
			
			UPS_ASSERTF(tagEnd != buffer.size(), "XML tag from position %lu does not terminate!", pos);

			// Determine what kind of tag it is.
			if (buffer[pos + 1] == '!')
			{
				tagType = XMLTagComment;
			}
			else if (buffer[pos + 1] == '?')
			{
				tagType = XMLTagDeclaration;
			}
			else if (buffer[pos + 1] == '/')
			{
				tagType = XMLTagEnd;
			}
			else if (buffer[tagEnd - 1] == '/')
			{
				tagType = XMLTagEmpty;
			}
			else
			{
				tagType = XMLTagStart;
			}
			
			UPS_LOGF("XML tag %d", static_cast<int>(tagType));
		}
	}
	
	return new XML(0);
}

ups::XML::XML(XML *parent):
	_parent(parent)
{
	std::printf("XML CONSTRUCTOR\n");
}

ups::XML::~XML()
{
	std::printf("XML DESTRUCTOR FOR %s\n", _name.c_str());
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