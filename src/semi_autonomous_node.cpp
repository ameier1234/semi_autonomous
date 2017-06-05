#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher pub;
bool isTooClose;

void teleopCallBack(const geometry_msgs::Twist::ConstPtr& msg)
{
  geometry_msgs::Twist autoCmd;
  autoCmd.linear.x = 0;
  autoCmd.angular.z = 3.14;

  if(isTooClose)
    {
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
  isTooClose = msg->ranges[center] < 0.25;

}

float correctCourse(sensor_msgs::LaserScan scan);

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


  //for now just return the direction of the largest node
  return scan.angle_min + (indexOfLargest * scan.angle_increment);


}
