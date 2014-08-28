#ifndef SE306P1_UPSTAGE_ROSWORLD_HPP_DEFINED
#define SE306P1_UPSTAGE_ROSWORLD_HPP_DEFINED

#include "../PointerUnique.hpp"
#include "../Resource.hpp"
#include <vector>

namespace ups
{
	class Actor;
	class Renderer;
	class ROSWorld : public Resource
	{
	public:
		typedef std::vector<PointerUnique<Actor> > ActorList;
		
		ROSWorld();
		~ROSWorld();
		
		void step();
		void draw(Renderer &renderer);
	private:
		ActorList _actorList;
	};
}

#endif // #ifndef SE306P1_UPSTAGE_ROSWORLD_HPP_DEFINED