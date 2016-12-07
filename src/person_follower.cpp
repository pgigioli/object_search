#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <darknet_ros/bbox_array.h>
#include <darknet_ros/bbox.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

const std::string DEPTH_PRED_TOPIC_NAME = "/depth_prediction";
const std::string YOLO_BBOXES_TOPIC_NAME = "/YOLO_bboxes";
const std::string CMD_VEL_TOPIC_NAME = "/cmd_vel_mux/input/teleop";

class PersonFollower
{
   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;
   image_transport::Subscriber _depth_predictions_sub;
   ros::Subscriber _YOLO_bboxes_sub;
   ros::Publisher _cmd_vel_pub;
   int _bbox_coordinates[4];
   bool _found_person;
   float _person_distance;

public:
   PersonFollower() : _it(_nh)
   {
      _depth_predictions_sub = _it.subscribe(DEPTH_PRED_TOPIC_NAME, 1,
                               &PersonFollower::depthPredCallback,this);
      _YOLO_bboxes_sub = _nh.subscribe(YOLO_BBOXES_TOPIC_NAME, 1, &PersonFollower::YOLObboxesCallback, this);
      _cmd_vel_pub = _nh.advertise<geometry_msgs::Twist>(CMD_VEL_TOPIC_NAME, 1);
   }

private:
   void YOLObboxesCallback(const darknet_ros::bbox_array::ConstPtr& bboxes_msg)
   {

      int num_bboxes = bboxes_msg->bboxes.size();

      for (int i = 0; i < num_bboxes; ++i)
      {
         darknet_ros::bbox object = bboxes_msg->bboxes[i];

         if (object.Class == "person")
         {
            _bbox_coordinates[0] = object.xmin;
            _bbox_coordinates[1] = object.ymin;
            _bbox_coordinates[2] = object.xmax;
            _bbox_coordinates[3] = object.ymax;

            _found_person = true;
            break;
         }
         else
         {
            _found_person = false;
         }
      }
   }

   void depthPredCallback(const sensor_msgs::ImageConstPtr& depth_msg)
   {
      cv_bridge::CvImageConstPtr depth_pred;
      depth_pred = cv_bridge::toCvShare(depth_msg);

      if (_found_person == true)
      {
         int bbox_w = _bbox_coordinates[2] - _bbox_coordinates[0];
         int bbox_h = _bbox_coordinates[3] - _bbox_coordinates[1];
         int center_box_xmin = _bbox_coordinates[0] + bbox_w/3;
         int center_box_xmax = _bbox_coordinates[2] - bbox_w/3;
         int center_box_ymin = _bbox_coordinates[1] + bbox_h/6;
         int center_box_ymax = _bbox_coordinates[3] - bbox_h/2;
         int center_box_w = center_box_xmax - center_box_xmin;
         int center_box_h = center_box_ymax - center_box_ymin;

         float total_bbox_depth = 0;

         for (int i = center_box_xmin; i < center_box_xmax; ++i)
         {
            for (int j = center_box_ymin; j < center_box_ymax; ++j)
            {
               float pixel_depth = depth_pred->image.at<float>(j,i);
               total_bbox_depth += pixel_depth;
            }
         }
         _person_distance = total_bbox_depth/((center_box_w/3) * (center_box_h/3));
         approachPerson(_person_distance);
         _found_person = false;
      }
   }

   void approachPerson(const float &dist)
   {
      std::cout << "person distance = " << dist << std::endl;

      geometry_msgs::Twist cmd_vel;

      if (dist > 3.0)
      {
         cmd_vel.linear.x = 0.1;
         for (int i = 0; i < 10; ++i)
         {
            _cmd_vel_pub.publish(cmd_vel);
            ros::Duration(0.1).sleep();
         }
      }
      if (dist < 2.0)
      {
         cmd_vel.linear.x = -0.1;
         for (int i = 0; i < 10; ++i)
         {
            _cmd_vel_pub.publish(cmd_vel);
            ros::Duration(0.1).sleep();
         }
      }
   }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "person_follower");
   PersonFollower pf;
   ros::spin();
   return 0;
}
