#ifndef SE306P1_UPSTAGE_ROSCOMM_HPP_DEFINED
#define SE306P1_UPSTAGE_ROSCOMM_HPP_DEFINED

#include "ros/ros.h"
#include <set>
#include <string>

// Sends KeyInput messages

namespace ups
{
	class Keyboard;
	class ROSComm
	{
	public:
		~ROSComm();
		
		bool executeLoop();
		void publishKeys(const Keyboard &keyboard);
		
		static ROSComm &getROSComm();
		
	private:
		ROSComm();
		
		void doPublishKeys();
		
		std::string rosName;
		ros::NodeHandle *nodeHandle;
		ros::Rate *loopRate;
		ros::Publisher publisherKeyInput;
	};
}

#endif // #ifndef SE306P1_UPSTAGE_ROSCOMM_HPP_DEFINED