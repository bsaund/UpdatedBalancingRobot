#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <sstream>

ros::Subscriber ucListener;
long lEnc, rEnc;
tf::Transform unrotTrans;
double wheelRadius = 0.033;
double wheelGap = .17;
double prevLEnc=0, prevREnc=0;


tf::StampedTransform makeRotatedMsg(double theta){
  tf::Vector3 v(0,-theta*wheelRadius,wheelRadius);
  tf::Quaternion q;
  q.setEuler(0,theta,0);
  tf::Transform trans(q,v);

  return tf::StampedTransform(trans, ros::Time::now(),
			      "robot_unrotated_frame", "robot_frame");
}

tf::StampedTransform makeUnrotatedMsg(double dL, double dR){
  tf::Vector3 v(0,(dL+dR)*wheelRadius/2, 0);
  tf::Quaternion q;
  q.setEuler(0,0,(dL-dR)*wheelRadius/wheelGap);

  tf::Transform delta(q, v);
  unrotTrans = unrotTrans * delta;
  return tf::StampedTransform(unrotTrans, ros::Time::now(),
			      "base_frame", "robot_unrotated_frame");

}


void ucListenerCallback(const std_msgs::String::ConstPtr& msg) {

  std::stringstream stream(msg->data.c_str());
  double theta, lEncNew, rEncNew;

  stream >> theta;
  stream >> lEncNew;
  stream >> rEncNew;

  double dL = lEncNew - prevLEnc;
  double dR = rEncNew - prevREnc;
  prevLEnc = lEncNew;
  prevREnc = rEncNew;
  
  static tf::TransformBroadcaster tfBroad;
  tfBroad.sendTransform(makeRotatedMsg(theta*M_PI/180));
  tfBroad.sendTransform(makeUnrotatedMsg(dL, dR));

							  
}

  

int main(int argc, char **argv){

  ros::init(argc, argv, "arduinoDataParsing");
  ros::NodeHandle rosNode;
  ROS_INFO("arduinoDataParsing starting");

  unrotTrans.setIdentity();

  ucListener = rosNode.subscribe("/uc0Response", 100, ucListenerCallback);

  
  ros::spin();

  ROS_INFO("arduinoParsing stopping");
  return 0;
}
