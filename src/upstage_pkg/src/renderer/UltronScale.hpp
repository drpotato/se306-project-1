#ifndef SE306P1_UPSTAGE_ULTRONSCALE_HPP_DEFINED
#define SE306P1_UPSTAGE_ULTRONSCALE_HPP_DEFINED

#include "Types.hpp"

namespace ups
{
	class Renderer;
	class UltronScale
	{
	public:
		virtual void draw(Renderer &renderer)=0;
		void resize(const Renderer &renderer);
		virtual ~UltronScale();
		
		float getL();
		float getR();
		float getU();
		float getD();
		
		void setOffsetsL(float offsetAbs, float offsetRel);
		void setOffsetsR(float offsetAbs, float offsetRel);
		void setOffsetsU(float offsetAbs, float offsetRel);
		void setOffsetsD(float offsetAbs, float offsetRel);
		
	protected:
		class UltronWeightedDimension
		{
		public:
			inline UltronWeightedDimension() :
				_low(0.f),
				_high(0.f),
				_offsetAbs(0.f),
				_offsetRel(0.f),
				_concrete(0.f),
				_dirty(true)
			{
			}
			
			inline void setOffsets(float offsetAbs, float offsetRel)
			{
				_offsetAbs = offsetAbs;
				_offsetRel = offsetRel;
				_dirty = true;
			}
			
			inline void updateBounds(float low, float high)
			{
				_low = low;
				_high = high;
				_dirty = true;
			}
			
			inline float getConcrete()
			{
				if (_dirty)
				{
					recalculate();
				}
				
				return _concrete;
			}
			
		private:
			inline void recalculate()
			{
				_concrete = _low + (_high - _low) * _offsetRel + _offsetAbs;
				_dirty = false;
			}
			
			float _low;
			float _high;
			float _offsetAbs;
			float _offsetRel;
			float _concrete;
			bool _dirty;
		};
	
		UltronWeightedDimension _left;
		UltronWeightedDimension _right;
		UltronWeightedDimension _up;
		UltronWeightedDimension _down;
	};
	
	inline UltronScale::~UltronScale()
	{
	}
	
	inline float UltronScale::getL()
	{
		return _left.getConcrete();
	}
	
	inline float UltronScale::getR()
	{
		return _right.getConcrete();
	}
	
	inline float UltronScale::getU()
	{
		return _up.getConcrete();
	}
	
	inline float UltronScale::getD()
	{
		return _down.getConcrete();
	}
		
	inline void UltronScale::setOffsetsL(float offsetAbs, float offsetRel)
	{
		_left.setOffsets(offsetAbs, offsetRel);
	}
	
	inline void UltronScale::setOffsetsR(float offsetAbs, float offsetRel)
	{
		_right.setOffsets(offsetAbs, offsetRel);
	}
	
	inline void UltronScale::setOffsetsU(float offsetAbs, float offsetRel)
	{
		_up.setOffsets(offsetAbs, offsetRel);
	}
	
	inline void UltronScale::setOffsetsD(float offsetAbs, float offsetRel)
	{
		_down.setOffsets(offsetAbs, offsetRel);
	}
}
#endif // #ifndef SE306P1_UPSTAGE_ULTRONSCALE_HPP_DEFINED