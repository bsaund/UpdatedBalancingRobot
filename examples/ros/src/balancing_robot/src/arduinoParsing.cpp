#include "ros/ros.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Twist.h"
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <sstream>

ros::Subscriber ucListener;
ros::Publisher visPub;
ros::Publisher speedPub;
long lEnc, rEnc;
tf::Transform unrotTrans;
double wheelRadius = 0.033;
double wheelGap = .17;
double prevLEnc=0, prevREnc=0;
tf::TransformBroadcaster *tfBroad;

int count = 0;

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
  q.setEuler(0,0,(dR-dL)*wheelRadius/wheelGap);

  tf::Transform delta(q, v);
  unrotTrans = unrotTrans * delta;
  return tf::StampedTransform(unrotTrans, ros::Time::now(),
			      "base_frame", "robot_unrotated_frame");

}


visualization_msgs::Marker makeSpeedMarker(double vl, double vr){
 
  double speed = (vl + vr)/2;
  std::ostringstream strs;
  strs << "Speed: " << speed;
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = "/robot_unrotated_frame";
  marker.header.stamp = ros::Time::now();
  marker.ns = "robot_msgs";
  marker.id = 100;
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  // marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.z = .2;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = .05;
  marker.scale.y = .05;
  marker.scale.z = .05;

  marker.color.a = 1;
  marker.color.g = 1;
  // marker.text = strs.str();
  marker.text = "HI";
  // marker.lifetime = ros::Duration();
  
  return marker;
}


void ucListenerCallback(const std_msgs::String::ConstPtr& msg) {
  if(count < 100){ // Ignore the first several messages, as they may contain bad data
    count++;
    return;
  }
  
  std::stringstream stream(msg->data.c_str());
  double theta, lEncNew, rEncNew, lSpeed, rSpeed;
  std::string debugPrefix("debug");

  if(!msg->data.compare(0, debugPrefix.size(), debugPrefix)){
    ROS_INFO("arduino %s", msg->data.c_str());
    return;
  }
     
     

  stream >> theta;
  stream >> lEncNew;
  stream >> rEncNew;
  stream >> lSpeed;
  stream >> rSpeed;

  double dL = lEncNew - prevLEnc;
  double dR = rEncNew - prevREnc;
  prevLEnc = lEncNew;
  prevREnc = rEncNew;
  

  tfBroad->sendTransform(makeRotatedMsg(theta*M_PI/180));
  tfBroad->sendTransform(makeUnrotatedMsg(dL, dR));
  // visPub.publish(makeSpeedMarker(lSpeed, rSpeed));
  // tf::Twist tw(
  // speedPub.publish(
  
							  
}

  

int main(int argc, char **argv){

  ros::init(argc, argv, "arduinoDataParsing");
  ros::NodeHandle rosNode;
  tfBroad = new tf::TransformBroadcaster;
  ROS_INFO("arduinoDataParsing starting");

  unrotTrans.setIdentity();

  ucListener = rosNode.subscribe("/uc0Response", 100, ucListenerCallback);
  visPub = rosNode.advertise<visualization_msgs::Marker>("visualization_marker",10);

  
  ros::spin();

  ROS_INFO("arduinoParsing stopping");
  return 0;
}
