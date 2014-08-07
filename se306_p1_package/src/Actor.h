#ifndef ACTORMAIN
#define ACTORMAIN

class Actor
{

private:
	//velocity of the robot
	double linear_x;
	double angular_z;

	//pose of the robot
	double px;
	double py;
	double theta;

	virtual int mainHook(int argc, char **argv);

	//Not 100% on which should be public and private
public:
	void StageOdom_callback(nav_msgs::Odometry msg);
	void StageLaser_callback(sensor_msgs::LaserScan msg);
	int main(int argc, char **argv);
};


#endif

