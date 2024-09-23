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
 ros::Publisher odom_pub= node.advertise<nav_msgs::Odometry>("odom", 100); //Odometria

  tf::TransformBroadcaster odom_broadcaster; //TF

  // Subscribe to ROS topics
  ros::Subscriber x_sub = node.subscribe("x", 100, xCallback);
  ros::Subscriber y_sub = node.subscribe("y", 100, yCallback);
  ros::Subscriber phi_sub = node.subscribe("phi", 100, phiCallback);
  ros::Subscriber uMeas_sub = node.subscribe("u", 100, uMeasCallback);
  ros::Subscriber wMeas_sub = node.subscribe("w", 100, wMeasCallback);

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate loop_rate(30);


  while (ros::ok()) {

          // Publicacion de Tf
          //tf2::Quaternion q;
          /*tf::Quaternion q;
          q.setRPY(0, 0, phi);*/
          ros::spinOnce();
          current_time = ros::Time::now();
          geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(phi);


          geometry_msgs::TransformStamped odom_trans;
          odom_trans.header.stamp = current_time;
          odom_trans.header.frame_id = "odom";
          odom_trans.child_frame_id = "base_footprint";

          odom_trans.transform.translation.x = x;
          odom_trans.transform.translation.y = y;
          odom_trans.transform.translation.z = 0.0;
          odom_trans.transform.rotation = odom_quat;

          //send the transform
          odom_broadcaster.sendTransform(odom_trans);
           //next, we'll publish the odometry message over ROS
          nav_msgs::Odometry odom;
          odom.header.stamp = current_time;
          odom.header.frame_id = "odom";

                    //set the position
          odom.pose.pose.position.x = x;
          odom.pose.pose.position.y = y;
          odom.pose.pose.position.z = 0.0;
          odom.pose.pose.orientation = odom_quat;

                    //set the velocity
          odom.child_frame_id = "base_footprint";
          odom.twist.twist.linear.x = uMeas;
          //odom.twist.twist.linear.y = vy;
          odom.twist.twist.angular.z = wMeas;

          //publish the message
          odom_pub.publish(odom);

          last_time = current_time;
          loop_rate.sleep();
  }

  return 0;
}

