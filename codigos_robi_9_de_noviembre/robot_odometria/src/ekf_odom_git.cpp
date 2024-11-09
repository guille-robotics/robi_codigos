#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

using namespace std;
double x=0.0;
double y=0.0;
double phi=0.0;
double uMeas=0.0;
double wMeas=0.0;
// Callback functions for each topic
void xCallback(const std_msgs::Float32& x_data) {
  x = x_data.data;
}

void yCallback(const std_msgs::Float32& y_data) {
  y = y_data.data;
}

void phiCallback(const std_msgs::Float32& phi_data) {
  phi = phi_data.data;
}

void uMeasCallback(const std_msgs::Float32& uMeas_data) {
  uMeas = uMeas_data.data;
}

void wMeasCallback(const std_msgs::Float32& wMeas_data) {
  wMeas = wMeas_data.data;
}


int main(int argc, char **argv) {
  // Launch ROS and create a node
  ros::init(argc, argv, "ekf_odom_pub");  // Nombre del Nodo

  ros::NodeHandle node; //Nombre para usar el nodo

  ros::Publisher odom_data_publisher= node.advertise<nav_msgs::Odometry>("odom", 100); //Odometria
  
  tf::TransformBroadcaster br; //TF

  // Subscribe to ROS topics
  ros::Subscriber x_sub = node.subscribe("x", 100, xCallback);
  ros::Subscriber y_sub = node.subscribe("y", 100, yCallback);
  ros::Subscriber phi_sub = node.subscribe("phi", 100, phiCallback);
  ros::Subscriber uMeas_sub = node.subscribe("u", 100, uMeasCallback);
  ros::Subscriber wMeas_sub = node.subscribe("w", 100, wMeasCallback);
 
  ros::Rate loop_rate(30);

 
  while (ros::ok()) {

	  // Publicacion de Tf
    	  tf2::Quaternion q;
          q.setRPY(0, 0, phi);
	 // Publicacion de Odometria
	  nav_msgs::Odometry quatOdom;
	  quatOdom.header.stamp = ros::Time::now();
	  quatOdom.header.frame_id = "odom";
          quatOdom.child_frame_id = "base_footprint";
	  quatOdom.pose.pose.position.x = x;
	  quatOdom.pose.pose.position.y = y;
	  quatOdom.pose.pose.orientation.x = q.x();
	  quatOdom.pose.pose.orientation.y = q.y();
	  quatOdom.pose.pose.orientation.z = q.z();
	  quatOdom.pose.pose.orientation.w = q.w();
	  
	  quatOdom.twist.twist.linear.x = uMeas;
	  quatOdom.twist.twist.angular.z = wMeas;
	 
	  odom_data_publisher.publish(quatOdom);

	  geometry_msgs::TransformStamped transform_stamped;
	  transform_stamped.header.stamp = ros::Time::now();
	  transform_stamped.header.frame_id = "odom";
	  transform_stamped.child_frame_id = "base_footprint";

	  transform_stamped.transform.translation.x = x;
	  transform_stamped.transform.translation.y = y;
	  transform_stamped.transform.translation.z = 0.0;
	  transform_stamped.transform.rotation.x = q.x();
	  transform_stamped.transform.rotation.y = q.y();
	  transform_stamped.transform.rotation.z = q.z();
	  transform_stamped.transform.rotation.w = q.w();

	  br.sendTransform(transform_stamped);
          ros::spinOnce();
          loop_rate.sleep();
  }
 
  return 0;
}

