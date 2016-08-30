#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <iostream>

using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dummy_found_object");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Int8>("found_object", 1);

  ros::Rate loop_rate(5);

  cin.get();
  while (ros::ok()) {
  	std_msgs::Int8 msg;
  	msg.data = 1;
  	pub.publish(msg);
  	ros::spinOnce();
	loop_rate.sleep();
  }

  return 0;
}
