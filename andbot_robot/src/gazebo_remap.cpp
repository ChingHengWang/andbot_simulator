#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

ros::Subscriber andbot_joint_states_sub,cmd_wheel_angularVel_sub;
ros::Publisher feedback_wheel_angularVel_pub,left_wheel_angularVel_pub,right_wheel_angularVel_pub;
ros::Time current_time, last_time;
double actual_omega_left=0.0;
double actual_omega_right=0.0;
double cmd_omega_left=0.0;
double cmd_omega_right=0.0;

void andbot_joint_states_Callback(const sensor_msgs::JointState &jointstate)
{
  actual_omega_left = jointstate.velocity[1];
  actual_omega_right = jointstate.velocity[2];
  current_time = ros::Time::now(); 
}
void cmd_wheel_angularVel_Callback(const geometry_msgs::Vector3 &cmd)
{
  cmd_omega_left = cmd.x;
  cmd_omega_right = cmd.y;
  current_time = ros::Time::now(); 
}
int main(int argc, char** argv){
  ros::init(argc, argv, "gazebo_remap");
  ros::NodeHandle n1;
  feedback_wheel_angularVel_pub = n1.advertise<geometry_msgs::Vector3>("feedback_wheel_angularVel", 50);
  left_wheel_angularVel_pub = n1.advertise<std_msgs::Float64>("/andbot/leftWheel_velocity_controller/command", 50);
  right_wheel_angularVel_pub = n1.advertise<std_msgs::Float64>("/andbot/rightWheel_velocity_controller/command", 50);

 cmd_wheel_angularVel_sub = n1.subscribe("cmd_wheel_angularVel", 10, cmd_wheel_angularVel_Callback);

  andbot_joint_states_sub = n1.subscribe("andbot/joint_states", 10, andbot_joint_states_Callback);
   ros::Rate r(10.0); // rate for publishing odom
  while(n1.ok()){
    std_msgs::Float64 left_cmd;
    std_msgs::Float64 right_cmd;
    left_cmd.data=cmd_omega_left;
    right_cmd.data=cmd_omega_right;
    left_wheel_angularVel_pub.publish(left_cmd);
    right_wheel_angularVel_pub.publish(right_cmd);

    geometry_msgs::Vector3 angularVel;
    angularVel.x=actual_omega_left;
    angularVel.y=actual_omega_right;
    feedback_wheel_angularVel_pub.publish(angularVel);
    ros::spinOnce();               // check for incoming messages
    r.sleep();
  }
}
