#ifndef SE306P1_UPSTAGE_UPSTAGEENVIRONMENT_HPP_DEFINED
#define SE306P1_UPSTAGE_UPSTAGEENVIRONMENT_HPP_DEFINED

#include "PointerUnique.hpp"
#include "Resource.hpp"
#include "renderer/Renderer.hpp"
#include "renderer/UltronScale.hpp"
#include <vector>

namespace ups
{
	class UpstageEnvironment : public Resource
	{
	public:
		UpstageEnvironment();
		
		void step();
		void draw(Renderer &renderer);
		void addUltronScale(UltronScale *element);
		
		void setBGColour(const Colour &bgColour);
		
	private:
		typedef std::vector<PointerUnique<UltronScale> > UltronVector;
		
		Colour _bgColour;
		UltronVector _elements;
	};
}

#endif // #ifndef SE306P1_UPSTAGE_UPSTAGEENVIRONMENT_HPP_DEFINED