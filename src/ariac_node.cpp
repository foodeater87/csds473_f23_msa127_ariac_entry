#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>
#include <vector>

std::vector<osrf_gear::Order> OrdersVector;
std::vector<osrf_gear::LogicalCameraImage> CameraData(10); //create vector containing all LogicalCameraImage variables

void orders(const osrf_gear::Order::ConstPtr& msg){
  OrdersVector.clear();
  osrf_gear::Order var;
  var.shipments = msg->shipments;
  var.order_id = msg->order_id;
  OrdersVector.push_back(var);
  ROS_INFO("We got the order.");
}
void logic_cam(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int cam_num) {
  CameraData[cam_num] = *msg;
}
void logic_agv1(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 0);
}

void logic_agv2(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 1);
}

void logic_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 2);
}

void logic_bin2(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 3);
}

void logic_bin3(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 4);
}

void logic_bin4(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 5);
}

void logic_bin5(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 6);
}

void logic_bin6(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 7);
}

void quality1(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 8);
}

void quality2(const osrf_gear::LogicalCameraImage::ConstPtr& msg){
  logic_cam(msg, 9);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "entry_node");
  ros::NodeHandle n;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
  std_srvs::Trigger begin_comp;
  ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
  int service_call_succeeded = begin_client.call(begin_comp);
  if (service_call_succeeded){
    if (begin_comp.response.success){
      ROS_INFO("Competition started successfully: %s", begin_comp.response.message.c_str());
    }
    else{
      ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
    }
  }
  else{
    ROS_ERROR("Competition service call failed! Goodness Gracious!!");
  }
  ros::ServiceClient MaterialLocations = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
  ros::Subscriber sub = n.subscribe("/ariac/orders", 1000, orders);
  ros::Subscriber suba1 = n.subscribe("/ariac/logical_camera_agv1", 1000, logic_agv1);
  ros::Subscriber suba2 = n.subscribe("/ariac/logical_camera_agv2", 1000, logic_agv2);
  ros::Subscriber subb1 = n.subscribe("/ariac/logical_camera_bin1", 1000, logic_bin1);
  ros::Subscriber subb2 = n.subscribe("/ariac/logical_camera_bin2", 1000, logic_bin2);
  ros::Subscriber subb3 = n.subscribe("/ariac/logical_camera_bin3", 1000, logic_bin3);
  ros::Subscriber subb4 = n.subscribe("/ariac/logical_camera_bin4", 1000, logic_bin4);
  ros::Subscriber subb5 = n.subscribe("/ariac/logical_camera_bin5", 1000, logic_bin5);
  ros::Subscriber subb6 = n.subscribe("/ariac/logical_camera_bin6", 1000, logic_bin6);
  ros::Subscriber subq1 = n.subscribe("/ariac/quality_control_sensor_1", 1000, quality1);
  ros::Subscriber subq2 = n.subscribe("/ariac/quality_control_sensor_2", 1000, quality2);
  while(ros::ok()){
    std::string OrderType;
    geometry_msgs::Pose pose;
    geometry_msgs::Pose Models;
    std::vector<std::string> CameraNames;
    std::string MaterialUnit;
    osrf_gear::GetMaterialLocations MaterialLocationType;
    if (begin_comp.response.success and not OrdersVector.empty()){
      OrderType = OrdersVector[0].shipments[0].products[0].type;
      MaterialLocationType.request.material_type = OrderType;
      if (MaterialLocations.call(MaterialLocationType)){
        MaterialUnit = MaterialLocationType.response.storage_units[0].unit_id;
        ROS_INFO("Storage unit: %s", MaterialUnit.c_str());
        CameraNames = {"agv1","agv2","bin1","bin2","bin3","bin4","bin5","bin6"};
        int size = CameraNames.size();
        for (int i = 0;i < size;i++){
          geometry_msgs::Pose CurrentCamera = CameraData[i].pose;
          if (MaterialUnit==CameraNames[i]){
            Models = CurrentCamera;
          }
        }
        pose = Models;
        //output the pose
        double poseow = pose.orientation.w;
        double poseox = pose.orientation.x;
        double poseoy = pose.orientation.y;
        double poseoz = pose.orientation.z;
        double posepx = pose.position.x;
        double posepy = pose.position.y;
        double posepz = pose.position.z;
        ROS_WARN("Part pose in camera's reference frame (w,x,y,z),(X,Y,Z): (%f,%f,%f,%f),(%f,%f,%f)",poseow,poseox,poseoy,poseoz,posepx,posepy,posepz);
        geometry_msgs::TransformStamped tfStamped;
        try {
          tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame",ros::Time(0.0), ros::Duration(1.0));
          ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),tfStamped.child_frame_id.c_str());
        } catch (tf2::TransformException &ex) {
          ROS_ERROR("%s", ex.what());
        }
        geometry_msgs::PoseStamped part_pose, goal_pose;
        part_pose.pose = pose;
        tf2::doTransform(part_pose, goal_pose, tfStamped);
        goal_pose.pose.position.z += 0.10;
        goal_pose.pose.orientation.w = 0.707;
        goal_pose.pose.orientation.x = 0.0;
        goal_pose.pose.orientation.y = 0.707;
        goal_pose.pose.orientation.z = 0.0;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
