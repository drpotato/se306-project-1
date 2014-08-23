#ifndef SE306P1_UPSTAGE_XML_HPP_DEFINED
#define SE306P1_UPSTAGE_XML_HPP_DEFINED

#include "../Resource.hpp"
#include "../PointerUnique.hpp"
#include <cstdio>
#include <map>
#include <string>
#include <vector>

// A super-relaxed XML parser. XML parsers are meant to throw errors if everything isn't just-so, but
// this parser is ignorant to a lot of the XML spec. It should correctly parse any well-formed XML
// document, but it probably accepts just about anything else too, so don't take the fact that it generated
// output to mean anything significant.
// 
// A significant exception to the can-parse-anything-XML rule is that this parser has no support for characters
// outside of the ASCII range.

namespace ups
{
	class XML : public Resource
	{
	public:
		typedef std::map<std::string, std::string> AttrMap;
		typedef std::vector<PointerUnique<XML> > ChildList;
		
		XML(XML *parent = 0);
		~XML();
		
		static ups::PointerUnique<XML> fromFile(std::FILE *f);
		void setContent(const std::string &content);
		void addChild(XML *child);
		void setName(const std::string &name);
		
		void print(int indentDepth = 0) const;
		
	private:
		XML *_parent;
		std::string _name;
		std::string _content;
		ChildList _children;
		AttrMap _attributes;
	};
	
	inline void XML::setContent(const std::string &content)
	{
		_content = content;
	}
	
	inline void XML::addChild(XML *child)
	{
		_children.push_back(PointerUnique<XML>(child));
	}
	
	inline void XML::setName(const std::string &name)
	{
		_name = name;
	}
}

#endif // #ifndef SE306P1_UPSTAGE_XML_HPP_DEFINED