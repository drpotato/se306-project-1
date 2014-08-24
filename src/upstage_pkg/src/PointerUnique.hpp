#ifndef SE306P1_UPSTAGE_POINTERUNIQUE_HPP_DEFINED
#define SE306P1_UPSTAGE_POINTERUNIQUE_HPP_DEFINED

// Basically just a really shonky version of unique_ptr to avoid needing to compile with C++11
// Since we can't have R-value references (nor move constructors etc.), this instead nulls out
// the old pointer as it's being copied.

namespace ups
{
	template <typename baseType>
	class PointerUnique
	{
	public:
		PointerUnique(baseType *data);
		PointerUnique(const PointerUnique &stealFrom);
		~PointerUnique();
		PointerUnique &operator=(PointerUnique &stealFrom);
		
		baseType &operator*() const;
		baseType *operator->() const;
	private:
		mutable baseType* _data; // Making it mutable is a dirty hack to sidestep constness - necessary to get copying-as-move working with C++98
	};
	
	template <typename baseType>
	PointerUnique<baseType>::PointerUnique(baseType *data) :
		_data(data)
	{
	}
	
	template <typename baseType>
	PointerUnique<baseType>::PointerUnique(const PointerUnique<baseType> &stealFrom) :
		_data(stealFrom._data)
	{
		stealFrom._data = 0;
	}
	
	template <typename baseType>
	PointerUnique<baseType>::~PointerUnique()
	{
		delete _data;
	}
	
	template <typename baseType>
	PointerUnique<baseType> &PointerUnique<baseType>::operator=(PointerUnique<baseType> &stealFrom)
	{
		delete _data;
		_data = stealFrom._data;
		stealFrom._data = 0;
		
		return *this;
	}
	
	template <typename baseType>
	baseType &PointerUnique<baseType>::operator*() const
	{
		return *_data;
	}
	
	template <typename baseType>
	baseType *PointerUnique<baseType>::operator->() const
	{
		return _data;
	}
}

#endif // #ifndef SE306P1_UPSTAGE_POINTERUNIQUE_HPP_DEFINED