#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "debug_tf_tool");
  ros::NodeHandle node;
  ros::Rate rate(10.0);

  tf::TransformListener listener;
  tf::StampedTransform tfTransform;
  tf::Vector3 origin;
  tf::Quaternion rotation;
  tf::Vector3 axis;

  static ros::Publisher publisher = 
    node.advertise<geometry_msgs::TransformStamped>("/footprint2map_transform",10);

  geometry_msgs::TransformStamped geoTransform;

  int seq = 0;
  geoTransform.header.frame_id = "map";
  geoTransform.child_frame_id = "base_footprint";

  while(node.ok()) {
    try { 
      listener.lookupTransform(geoTransform.header.frame_id,
                   geoTransform.child_frame_id,       
                   ros::Time(0), 
                   tfTransform);
    }
    catch(tf::TransformException &exception) { 
      ROS_ERROR("%s", exception.what());
    }

    origin = tfTransform.getOrigin();
    rotation = tfTransform.getRotation();
    axis = rotation.getAxis();

    geoTransform.header.seq = seq;
    geoTransform.header.stamp = tfTransform.stamp_;

    geoTransform.transform.translation.x = origin.x();
    geoTransform.transform.translation.y = origin.y();
    geoTransform.transform.translation.z = origin.z();

    geoTransform.transform.rotation.x = axis.x();
    geoTransform.transform.rotation.y = axis.y();
    geoTransform.transform.rotation.z = axis.z();
    geoTransform.transform.rotation.w = rotation.getW();

    seq++;
    publisher.publish(geoTransform);
    rate.sleep();
  }

  return 0;
}
