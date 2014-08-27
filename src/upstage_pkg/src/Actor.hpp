#ifndef SE306P1_UPSTAGE_ACTOR_HPP_DEFINED
#define SE306P1_UPSTAGE_ACTOR_HPP_DEFINED

namespace ups
{
	class Renderer;
	class Actor
	{
	public:
		Actor();
		~Actor();
		void update(float x, float y, float theta, float vLinear, float vAngular);
		
		void draw(Renderer &renderer);
	private:
		float _x, _y, _theta, _vLinear, _vAngular;
	};
}

#endif // #ifndef SE306P1_UPSTAGE_ACTOR_HPP_DEFINED