#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher pub;
bool isTooClose;
float correctedCourse;

float correctCourse(sensor_msgs::LaserScan scan);
void teleopCallBack(const geometry_msgs::Twist::ConstPtr& msg);
void sensorCallBack(const sensor_msgs::LaserScan::ConstPtr& msg);

int main(int argc, char** argv)
{
  isTooClose = false;
  ros::init(argc, argv, "Semi_autonomous");
  ros::NodeHandle n;
  ros::Subscriber sensor_sub = n.subscribe("/base_scan", 1000, sensorCallBack);
  ros::Subscriber teleop_sub = n.subscribe("/temp_cmd_vel", 1000, teleopCallBack);
  

  
  pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::spin();

  return 0;
}

//find the optimal direction
float correctCourse(sensor_msgs::LaserScan scan)
{
  int size = scan.ranges.size();
  int indexOfLargest = 0;
  for(int i = 0; i < size; i++)
    {
      if(scan.ranges[i] > scan.ranges[indexOfLargest])
	{
	  indexOfLargest = i;
	}


    }


  //for now just return the direction of the longest range 
  return scan.angle_min + (indexOfLargest * scan.angle_increment);


}

 void teleopCallBack(const geometry_msgs::Twist::ConstPtr& msg)
 {
   geometry_msgs::Twist autoCmd;
   autoCmd.linear.x = 0;
   //autoCmd.angular.z = 3.14;

   if(isTooClose)
     {
       autoCmd.angular.z = correctedCourse;
       autoCmd.linear.x = -1;
       pub.publish(autoCmd);
     }
   else
     {
       pub.publish(msg);
     }

 }

 void sensorCallBack(const sensor_msgs::LaserScan::ConstPtr& msg)
 {
   int center = msg->ranges.size() / 2;

   isTooClose = false;
   for(int i  = center - 75; i < center + 75; i++)
     {
       if(msg->ranges[i] < 0.5)
	 {
	   isTooClose = true;
	   correctedCourse = correctCourse(*msg);

	 }

     }
 }
