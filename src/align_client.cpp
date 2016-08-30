#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace std;

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class alignClient {

public:

  ros::NodeHandle nh;
  ros::Subscriber found_object_sub;
  ros::Publisher cmd_vel_pub;
  tf::TransformListener listener;

  alignClient() : nh()
  {
//      found_object_sub = nh.subscribe("/ROI_coordinate", 1,
//         &alignClient::foundObjectCallback, this);
     found_object_sub = nh.subscribe("/found_object", 1,
	&alignClient::foundObjectCallback, this);
     cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
  }

private:

  void foundObjectCallback(std_msgs::Int8 msg)
//  void foundObjectCallback(geometry_msgs::Point msg)
  {
        if (msg.data == 1) {
//MoveBaseClient client("move_base", true);

//while(!client.waitForServer(ros::Duration(5.0))) {
//   ROS_INFO("Waiting for the move_base action server to come up");
//}

//move_base_msgs::MoveBaseGoal goal;

	tf::StampedTransform transform;

	try {
	   listener.lookupTransform("/base_link", "/cam_base",
		ros::Time(0), transform);
	}
	catch (tf::TransformException ex) {
	   ROS_ERROR("%s", ex.what());
	   ros::Duration(1.0).sleep();
	}

	tf::Matrix3x3 m(transform.getRotation());
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	geometry_msgs::Quaternion q;
	tf::Quaternion goalOrientation = tf::createQuaternionFromRPY(0.0, 0.0, yaw);
	tf::quaternionTFToMsg(goalOrientation, q);

	geometry_msgs::Twist cmd_vel;
	cmd_vel.angular.z = yaw;
	cmd_vel_pub.publish(cmd_vel);
//goal.target_pose.header.frame_id = "base_link";
//goal.target_pose.header.stamp = ros::Time::now();
//goal.target_pose.pose.orientation = q;

//ROS_INFO("Sending goal");

//client.sendGoal(goal);
//client.waitForResult();

//if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
//   ROS_INFO("Goal pose reached");
//else
//   ROS_INFO("The base failed to align with the webcam");

//printf("Current State: %s\n", client.getState().toString().c_str());

	return;
     }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "align_client");
  alignClient client;
  ros::spin();
  return 0;
}
