#ifndef ACTORMAIN
#define ACTORMAIN

class Actor
{

protected:
	//velocity of the robot
	double linear_x;
	double angular_z;

	//pose of the robot
	double px;
	double py;
	double theta;
	
	void execute();

	

public:
	//void StageOdom_callback(nav_msgs::Odometry msg);
	//void StageLaser_callback(sensor_msgs::LaserScan msg);
	//int main(int argc, char **argv);
	
	virtual void init(int argc, char **argv) = 0;
	virtual void executeHook() = 0; 
};


#endif

