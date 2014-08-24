#ifndef SE306P1_UPSTAGE_UPSTAGEENVIRONMENT_HPP_DEFINED
#define SE306P1_UPSTAGE_UPSTAGEENVIRONMENT_HPP_DEFINED

#include "Resource.hpp"
#include "renderer/Renderer.hpp"

namespace ups
{
	class UpstageEnvironment : public Resource
	{
	public:
		UpstageEnvironment();
		
		void step();
		void draw(Renderer &renderer);
		
		void setBGColour(const Colour &bgColour);
	private:
		Colour _bgColour;
	};
}

#endif // #ifndef SE306P1_UPSTAGE_UPSTAGEENVIRONMENT_HPP_DEFINED