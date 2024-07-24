#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_publisher;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

/////////////////////// VELOCIDAD ANGULAR //////////////////////
double wR = 0.0;  // Velocidad angular Rueda Derecha en rad/s.
double wL = 0.0;  // Velocidad angular Rueda Izquierda en rad/s.

/////////////////////// Valores //////////////////////
double constValue = 0.20; //(1000*2*pi)/R, R= 25910

//////////////////////// ROBOT /////////////////////////
double uMeas  = 0;
double wMeas  = 0;
double xp = 0.0, yp = 0.0;
double x = 0.0, y = 0.0;
double phi = 0.0;
const double R = 0.24; // radio de la llanta
const double d = 0.70; // Distancia entre llantas

unsigned long lastTime = 0;    // Tiempo anterior
unsigned long sampleTime = 100; // Tiempo de muestreo

using namespace std;

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

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() { //Publicar como Quaternion
  tf2::Quaternion q;
  q.setRPY(0, 0, phi);
 
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = ros::Time::now();
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = x;
  quatOdom.pose.pose.position.y = y;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = uMeas;
  quatOdom.twist.twist.angular.z = wMeas;
 
  odom_data_publisher.publish(quatOdom);
}

int main(int argc, char **argv) {
  // Launch ROS and create a node
  ros::init(argc, argv, "ekf_odom_pub");
  ros::NodeHandle node;
 
  // Subscribe to ROS topics
  ros::Subscriber x_sub = node.subscribe("x", 100, xCallback);
  ros::Subscriber y_sub = node.subscribe("y", 100, yCallback);
  ros::Subscriber phi_sub = node.subscribe("phi", 100, phiCallback);
  ros::Subscriber uMeas_sub = node.subscribe("uMeas", 100, uMeasCallback);
  ros::Subscriber wMeas_sub = node.subscribe("wMeas", 100, wMeasCallback);

  // Publisher of full odom message where orientation is quaternion
  odom_data_publisher = node.advertise<nav_msgs::Odometry>("odom_", 100);
 
  ros::Rate loop_rate(30);
 
  while (ros::ok()) {
    publish_quat();
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
