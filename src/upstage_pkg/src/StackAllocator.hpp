#ifndef SE306P1_UPSTAGE_STACKALLOCATOR_HPP_DEFINED
#define SE306P1_UPSTAGE_STACKALLOCATOR_HPP_DEFINED

#include "Debug.hpp"
#include <cstdlib>
#include <vector>

// A stack allocator. If the stack size is exceeded, it prints a warning, and then starts allocating using new.
// Obviously, if that starts happening, you're losing any performance benefit, but it does mean this won't
// cause crashes (I hope).
// 
// It's probably worth mentioning that this won't call constructors or destructors

namespace ups
{
	class StackAllocator
	{
	public:
		StackAllocator(size_t allocatorSize);
		~StackAllocator();
		
		template <typename t>
		t *allocate();
		
		void clear();
		
	private:
		void clearEmergencyAllocs();
		
		unsigned char *_dataMin; // The start of the block of memory
		unsigned char *_dataPtr; // The current data pointer
		unsigned char *_dataMax; // One past the block of memory
		
		// Data that had to be allocated because the stack size was exceeded.
		std::vector<void *> _emergencyAllocs;
	};
	
	inline StackAllocator::StackAllocator(size_t allocatorSize)
	{
		_dataPtr = _dataMin = reinterpret_cast<unsigned char *>(std::malloc(allocatorSize));
		_dataMax = _dataMin + allocatorSize;
	}
	
	inline StackAllocator::~StackAllocator()
	{
		// Clear the "nice" memory
		std::free(_dataPtr);
		
		// Also clear the "bad" memory
		clearEmergencyAllocs();
	}
	
	inline void StackAllocator::clear()
	{
		_dataPtr = _dataMin;
		clearEmergencyAllocs();
	}
	
	inline void StackAllocator::clearEmergencyAllocs()
	{
		for (std::vector<void *>::iterator it = _emergencyAllocs.begin(); it != _emergencyAllocs.end(); ++it)
		{
			std::free(*it);
		}
		
		_emergencyAllocs.clear();
	}
	
	template <typename t>
	inline t *StackAllocator::allocate()
	{
		unsigned char *newDataPtr = _dataPtr + sizeof(t);
		t *allocatedMemory;
		
		// Check if we have a room available
		if (newDataPtr > _dataMax)
		{
			// Uh-oh, we're fully booked, but we can offer the laundry cupboard instead.
			allocatedMemory = reinterpret_cast<t *>(std::malloc(sizeof(t)));
			
			// Let's see if you'll be sharing the cupboard ...
			if (_emergencyAllocs.size() == 0)
			{
				// Looks like you're the first here. I'm so sorry sir, this has never happened before.
				// I'd better phone management ...
				UPS_LOGF("StackAllocator storage exceeded. Current limit is %d.", (int)_dataMax - (int)_dataMin);
			}
			
			// I'll write your name on this napkin so we'll know where to send your continental breakfast.
			_emergencyAllocs.push_back(reinterpret_cast<void *>(allocatedMemory));
		}
		else
		{
			// Ahh yes, right this way, sir. Here is your key.
			allocatedMemory = reinterpret_cast<t *>(_dataPtr);
			
			// I'll just mark your room as occupied.
			_dataPtr = newDataPtr;
		}
		
		return allocatedMemory;
	}
}

#endif // #ifndef SE306P1_UPSTAGE_STACKALLOCATOR_HPP_DEFINED