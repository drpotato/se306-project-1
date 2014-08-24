#ifndef SE306P1_UPSTAGE_XML_HPP_DEFINED
#define SE306P1_UPSTAGE_XML_HPP_DEFINED

#include "../Resource.hpp"
#include "../PointerUnique.hpp"
#include "../Util.hpp"
#include <cstdio>
#include <map>
#include <string>
#include <vector>

// A simple XML parser. XML parsers are meant to throw errors if everything isn't just-so, but
// this parser is ignorant to a lot of the XML spec. It should parse any well-formed XML document,
// but it probably accepts just about anything else too, so don't take the fact that it generated
// output to mean anything significant.
// 
// A significant exception to the can-parse-anything-XML rule is that this parser has no support for characters
// outside of the ASCII range.
// 
// Another thing worth mentioning is that the XML is transformed into a representation that doesn't
// preserve the order of children. For that reason, text interspersed with child elements won't stay
// in the same structure, but the advantage is faster querying of a child with a particular name.

namespace ups
{
	class XML : public Resource
	{
	public:
		enum ParserConfig {
			XML_IGNORE_PURE_WHITESPACE = 0x00000001
		};
		
		typedef std::map<std::string, std::string> AttrMap;
		typedef std::vector<PointerUnique<XML> > ChildList;
		typedef std::vector<std::string> TextList;
		
		XML(XML *parent = 0);
		~XML();
		
		static ups::PointerUnique<XML> fromFile(std::FILE *f, unsigned int parserFlags = XML_IGNORE_PURE_WHITESPACE);
		void setName(const std::string &name);
		void addTextNode(const std::string &text);
		void addChild(XML *child);
		void addAttribute(const std::string &name, const std::string &value);
		
		std::vector<const XML *>getChildren(const std::string &childName) const;
		const XML &getChild(const std::string &childName) const;
		template<typename t>
		const t getAttr(const std::string &attrName, const t &attrDefault) const;
		
		void print(int indentDepth = 0) const;
		
	private:
		XML *_parent;
		std::string _name;
		TextList _textNodes;
		ChildList _children;
		AttrMap _attributes;
		
		static const XML xmlNull;
	};
	
	inline void XML::setName(const std::string &name)
	{
		_name = name;
	}
	
	inline void XML::addTextNode(const std::string &text)
	{
		_textNodes.push_back(text);
	}
	
	inline void XML::addChild(XML *child)
	{
		_children.push_back(PointerUnique<XML>(child));
	}
	
	inline void XML::addAttribute(const std::string &name, const std::string &value)
	{
		_attributes[name] = value;
	}
	
	inline std::vector<const XML *>XML::getChildren(const std::string &childName) const
	{
		std::vector<const XML *> outputVector;
		
		for (ChildList::const_iterator it = _children.begin(); it != _children.end(); ++it)
		{
			if ((*it)->_name == childName)
			{
				outputVector.push_back(&**it);
			}
		}
		
		return outputVector;
	}
	
	inline const XML &XML::getChild(const std::string &childName) const
	{
		for (ChildList::const_iterator it = _children.begin(); it != _children.end(); ++it)
		{
			if ((*it)->_name == childName)
			{
				return **it;
			}
		}
		
		return xmlNull;
	}
	
	template<>
	inline const char *const XML::getAttr<const char *>(const std::string &attrName, const char *const &attrDefault) const
	{
		AttrMap::const_iterator it = _attributes.find(attrName);
		
		if (it == _attributes.end())
		{
			return attrDefault;
		}
		
		return it->second.c_str();
	}
	
	template<>
	inline const float XML::getAttr<float>(const std::string &attrName, const float &attrDefault) const
	{
		float output;
		return s2num<float>(output, getAttr<const char *>(attrName, "")) ? output : attrDefault;
	}
	
	template<>
	inline const double XML::getAttr<double>(const std::string &attrName, const double &attrDefault) const
	{
		double output;
		return s2num<double>(output, getAttr<const char *>(attrName, "")) ? output : attrDefault;
	}
}

#endif // #ifndef SE306P1_UPSTAGE_XML_HPP_DEFINED