#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

ros::Subscriber ucListener;


void ucListenerCallback(const std_msgs::String::ConstPtr& msg) {

  double theta = atof(msg->data.c_str())*M_PI/180;
  double wheelRadius = 0.033;
  
  static tf::TransformBroadcaster transformBroadcaster;
  tf::Vector3 v(0,-theta*wheelRadius,wheelRadius);
  tf::Quaternion q;
  q.setEuler(0,theta,0);
  tf::Transform trans(q,v);

  tf::StampedTransform transStmp(trans, ros::Time::now(),
				"base_frame", "robot_frame");
  transformBroadcaster.sendTransform(transStmp);
							  
}
  

int main(int argc, char **argv){

  ros::init(argc, argv, "arduinoDataParsing");
  ros::NodeHandle rosNode;
  ROS_INFO("arduinoDataParsing starting");



  ucListener = rosNode.subscribe("/uc0Response", 100, ucListenerCallback);
  // transformBroadcaster = tf::TransformBroadcaster();
  
  ros::spin();

  ROS_INFO("arduinoParsing stopping");
  return 0;
}
