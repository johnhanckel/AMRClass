#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

void imuCallback(const sensor_msgs::Imu::ConstPtr& data)
{
  ROS_INFO("Angular Velocity = %f", data->angular_velocity.z);//print out the angular velocity
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_turn");
  ros::NodeHandle nh;
  ros::Publisher vel_pub;
  vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1, true);
  ros::Subscriber sub = nh.subscribe("/mobile_base/sensors/imu_data", 10, imuCallback);
  ros::Rate loop_rate(10);
  int count = 0;
  geometry_msgs::Twist vel;
  vel.linear.x = 0;//linear velocity(m/s)
  vel.angular.z = 1.0;//angular velocity(rad/s)
  while (ros::ok())
  {
    vel_pub.publish(vel);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
    if(count==20)
	vel.angular.z = 0;
  }
  return 0;
}
