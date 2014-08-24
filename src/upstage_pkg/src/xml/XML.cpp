#include "XML.hpp"
#include "../Util.hpp"

//#define XML_DEBUG_LOG

#ifdef XML_DEBUG_LOG
	#define XML_LOG(msg) UPS_LOG(msg)
	#define XML_LOGF(msg, ...) UPS_LOGF(msg, __VA_ARGS__)
	#define XML_ASSERT(msg) UPS_ASSERT(msg)
	#define XML_ASSERTF(msg, ...) UPS_ASSERTF(msg, __VA_ARGS__)
#else
	#define XML_LOG(msg)
	#define XML_LOGF(msg, ...)
	#define XML_ASSERT(msg)
	#define XML_ASSERTF(msg, ...)
#endif

ups::PointerUnique<ups::XML> ups::XML::fromFile(std::FILE *f, unsigned int parserFlags)
{
	// Read everything up-front to make it easier to go backwards and forwards.
	std::vector<char> buffer = readAll(f);
	
	unsigned long lastNodePos = 0;
	XML *openNode = 0;
	XML *rootNode = 0;
	enum TagType
	{
		XMLTagComment,
		XMLTagDeclaration,
		XMLTagStart,
		XMLTagEnd,
		XMLTagEmpty,
		XMLTagInvalid
	};
	
	for (long pos = 0; pos < buffer.size(); ++pos)
	{
		// Ignore everything until the first tag
		if (rootNode == 0 && buffer[pos] != '<')
		{
			continue;
		}
		
		// Start of a tag
		if (buffer[pos] == '<')
		{
			// If we have a node to add it to, push the text data as a text node (unless it's empty)
			long textLength = pos - lastNodePos;
			if (openNode && textLength > 0)
			{
				std::string textNode = std::string(buffer.begin() + lastNodePos, buffer.begin() + pos);
				
				// If we've been told to ignore pure whitespace, we need to check that that's not what we have.
				if (parserFlags & XML_IGNORE_PURE_WHITESPACE)
				{
					long nonWhitespacePos = 0;
					while (++nonWhitespacePos < textNode.length() && isWhitespace(textNode[nonWhitespacePos]));
					
					if (nonWhitespacePos != textNode.length())
					{
						openNode->addTextNode(textNode);
					}
				}
				else // Otherwise, we can just accept it.
				{
					openNode->addTextNode(textNode);
				}
			}
			
			
			TagType tagType = XMLTagInvalid;
			
			// Find the end of the tag
			long tagEnd = pos;
			while (++tagEnd < buffer.size() && buffer[tagEnd] != '>');
			
			XML_ASSERTF(tagEnd != buffer.size(), "XML tag from position %lu does not terminate!", pos);

			// Determine what kind of tag it is.
			if (buffer[pos + 1] == '!')
			{
				tagType = XMLTagComment;
				
				// Comments have special behaviour - they ignore >s that don't have -- before them.
				// That's so that you can comment out XML markup, but it means we may have tagEnd set to a > before the end
				while (buffer[tagEnd-1] != '-' || buffer[tagEnd-2] != '-')
				{
					while (++tagEnd < buffer.size() && buffer[tagEnd] != '>');
				}
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
			
			// <name.*/?>
			if (tagType == XMLTagEmpty or tagType == XMLTagStart)
			{
				// The tag name begins immediately after the < character, and continues until the
				// first whitespace, or the end of the tag (whichever comes first).
				long nameStart = pos + 1;
				long nameEnd = nameStart;
				while (++nameEnd < tagEnd && !isWhitespace(buffer[nameEnd]));
				std::string tagName(buffer.begin() + nameStart, buffer.begin() + nameEnd);
				
				// We now have enough information to create the XML tag. 
				XML *newTag = new XML(openNode);
				newTag->setName(tagName);
				
				// Get attributes
				long attrStart = nameEnd;
				while (attrStart < tagEnd)
				{
					// Scan past whitespace
					while (++attrStart < tagEnd && isWhitespace(buffer[attrStart]));
					
					// Make sure it's not the the end of an tag that we're bumping into, rather than an attribute.
					if (buffer[attrStart] == '/' || buffer[attrStart] == '>')
					{
						break;
					}
					
					// Find the "="
					long attrEnd = attrStart;
					while (++attrEnd < tagEnd && buffer[attrEnd] != '=');
					
					// Find the start of the quoted value section
					long attValStart = attrEnd;
					while (++attValStart < tagEnd && buffer[attValStart] != '\'' && buffer[attValStart] != '"');
					
					// Make sure the closing quote matches the opening one
					char quoteType = buffer[attValStart];
					
					// Find the end of the quoted value section
					long attValEnd = attValStart;
					while (++attValEnd < tagEnd && buffer[attValEnd] != quoteType);
					
					// We now have enough information to get the attribute name and value.
					std::string attrName(buffer.begin() + attrStart, buffer.begin() + attrEnd);
					std::string attrValue(buffer.begin() + attValStart + 1, buffer.begin() + attValEnd);
					
					// Add the attribute, and shift along
					newTag->addAttribute(attrName, attrValue);
					attrStart = attValEnd;
				}
				
				// If this tag is the first tag, it is the "root" tag. If it is also an "empty" tag,
				// then the XML document ends here.
				if (rootNode == 0)
				{
					rootNode = newTag;
					if (tagType == XMLTagEmpty)
					{
						break;
					}
					openNode = newTag;
				}
				else // The node is not the root, so it has a parent.
				{
					openNode->addChild(newTag);
					if (tagType != XMLTagEmpty)
					{
						openNode = newTag;
					}
				}
				
				XML_LOGF("XML opened tag %d \"%s\"", static_cast<int>(tagType), tagName.c_str());
			}
			else if (tagType == XMLTagEnd)
			{
				// Normally, I would check to see that the closing tag matches the opening tag, but if there's a
				// missing tag, it will manifest itself as an invalid structure anyway.
				// If it's not missing, then it's probably a typo, and I
				// don't think it's worth unduly punishing the XML writer for such a trivial mistake, especially if it
				// means extra effort on the part of the reader.
				// 
				// Soooo ... instead I ignore that piece of redundancy.
				
				XML *closedNode = openNode;
				openNode = openNode->_parent;
				
				XML_LOGF("XML closed tag %d \"%s\"", static_cast<int>(tagType), closedNode->_name.c_str());
				
				// If we closed the root, the parsing ends. There can only be one root in the XML spec, and I rely on that fact.
				if (closedNode == rootNode)
				{
					XML_LOG("XML root closed");
					break;
				}
			}
			else
			{
				XML_LOGF("XML tag type enum val %d", static_cast<int>(tagType));
			}
			
			// We've parsed the tag, so we can move on to the next thing
			pos = tagEnd;
			lastNodePos = tagEnd + 1;
		}
	}
	
	return rootNode;
}

ups::XML::XML(XML *parent):
	_parent(parent)
{
}

ups::XML::~XML()
{
}

void ups::XML::print(int indentDepth) const
{
	std::string indentString(indentDepth, '\t');
	
	std::printf("%s<%s", indentString.c_str(), _name.c_str());
	
	for (AttrMap::const_iterator it = _attributes.begin(); it != _attributes.end(); ++it)
	{
		std::printf(" %s=\"%s\"", it->first.c_str(), it->second.c_str());
	}
	
	if (_children.empty())
	{
		std::printf(" />\n");
		return;
	}
	
	std::printf(">");
	
	if (_textNodes.empty())
	{
		std::printf("\n");
	}
	else
	{
		bool leaveGap = false;
		for (TextList::const_iterator it = _textNodes.begin(); it != _textNodes.end(); ++it)
		{
			std::printf("%s%s", leaveGap ? "\n" : "", it->c_str());
			leaveGap = true;
		}
	}
	
	for (ChildList::const_iterator it = _children.begin(); it != _children.end(); ++it)
	{
		(*it)->print(indentDepth + 1);
	}
	
	if (!_children.empty())
	{
		std::printf("%s</%s>\n", indentString.c_str(), _name.c_str());
	}
}