#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "ik_service/PoseIK.h"
#include "osrf_gear/GetMaterialLocations.h"
#include "osrf_gear/LogicalCameraImage.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include <iostream>
#include <vector>
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "ur_kinematics/ur_kin.h"

ros::Publisher p_pub;

int x;
sensor_msgs::JointState joint_states;
std::vector<osrf_gear::Order> OrdersVector;
std::vector<osrf_gear::LogicalCameraImage> CameraData(10); //create vector containing all LogicalCameraImage variables
tf2_ros::Buffer tfBuffer;
std_srvs::Trigger begin_comp;
geometry_msgs::Pose endPos;
geometry_msgs::PoseStamped part_pose, goal_pose;
int count = 0;
int jt_name_size;
double T_pose[4][4], T_des[4][4];
double q_pose[6], q_des[8][6];
ik_service::PoseIK ik_pose;
trajectory_msgs::JointTrajectory joint_trajectory;

void JointState_Callback(const sensor_msgs::JointState::ConstPtr& msg){
  joint_states = *msg;
}

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

void MoveArm(ros::ServiceClient ik_client){
  endPos.position.x = goal_pose.pose.position.x;
  endPos.position.y = goal_pose.pose.position.y;
  endPos.position.z = goal_pose.pose.position.z;
  endPos.orientation.w = 0.707;
  endPos.orientation.x = 0.0;
  endPos.orientation.y = 0.707;
  endPos.orientation.z = 1;
  ik_pose.request.part_pose = endPos;
  bool success = false;
  if (ik_client.call(ik_pose)){
    success = true;
  }
  if(success and ik_pose.response.num_sols>0 and ik_pose.response.num_sols<=8){
    for(int i=0;i<8;i++){
      bool case1 = ik_pose.response.joint_solutions[i].joint_angles[0] >=  0;
      bool case2 = ik_pose.response.joint_solutions[i].joint_angles[0] <=  3.14 / 2;
      bool case3 = ik_pose.response.joint_solutions[i].joint_angles[1] >=  0;
      bool case4 = ik_pose.response.joint_solutions[i].joint_angles[1] <=  3.14;
      if(case1 and case2 and case3 and case4){
        x = i;
        break;
      }
    }
  }
  //ROS_INFO("Angle 1: %f radians",ik_pose.response.joint_solutions[x].joint_angles[1];
  //ROS_INFO("Angle 2: %f radians",ik_pose.response.joint_solutions[x].joint_angles[2];
  //ROS_INFO("Angle 3: %f radians",ik_pose.response.joint_solutions[x].joint_angles[3];
  //ROS_INFO("Angle 4: %f radians",ik_pose.response.joint_solutions[x].joint_angles[4];
  //ROS_INFO("Angle 5: %f radians",ik_pose.response.joint_solutions[x].joint_angles[5];
  //ROS_INFO("Angle 6: %f radians",ik_pose.response.joint_solutions[x].joint_angles[6];
  joint_trajectory.header.seq = count++;
  joint_trajectory.header.stamp = ros::Time::now();
  joint_trajectory.header.frame_id = "/world";
  joint_trajectory.joint_names.clear();
  joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
  joint_trajectory.joint_names.push_back("shoulder_pan_joint");
  joint_trajectory.joint_names.push_back("shoulder_lift_joint");
  joint_trajectory.joint_names.push_back("elbow_joint");
  joint_trajectory.joint_names.push_back("wrist_1_joint");
  joint_trajectory.joint_names.push_back("wrist_2_joint");
  joint_trajectory.joint_names.push_back("wrist_3_joint");
  joint_trajectory.points.resize(2);
  joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
  for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
    for (int indz = 0; indz < joint_states.name.size(); indz++) {
      if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
        joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
        break;
      }
    }
  }
  joint_trajectory.points[0].time_from_start = ros::Duration(0.0); 
  int sol_indx = 3;
  joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
  joint_trajectory.points[1].positions[0] = joint_states.position[1];
  for (int indy = 0; indy < 6; indy++) {
    joint_trajectory.points[1].positions[indy + 1] = ik_pose.response.joint_solutions[x].joint_angles[indy];
  }
  joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
  p_pub.publish(joint_trajectory);
}

void GetOrder(ros::ServiceClient MaterialLocations,ros::ServiceClient ik_client){
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
      double poseow = pose.orientation.w;
      double poseox = pose.orientation.x;
      double poseoy = pose.orientation.y;
      double poseoz = pose.orientation.z;
      double posepx = pose.position.x;
      double posepy = pose.position.y;
      double posepz = pose.position.z;
      ROS_WARN("Part position in camera's reference frame: (x,y,z) = (%f,%f,%f)", posepx,posepy,posepz);
      ROS_WARN("Part orientation in camera's reference frame: (w,x,y,z) = (%f,%f,%f,%f)",poseow,poseox,poseoy,poseoz);
      geometry_msgs::TransformStamped tfStamped;
      try {
        tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame",ros::Time(0.0), ros::Duration(1.0));
        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),tfStamped.child_frame_id.c_str());
      } catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
      }
      part_pose.pose = pose;
      goal_pose.pose.position.z += 0.10;
      goal_pose.pose.orientation.w = 0.707;
      goal_pose.pose.orientation.x = 0.0;
      goal_pose.pose.orientation.y = 0.707;
      goal_pose.pose.orientation.z = 0.0;
      double gppow = goal_pose.pose.orientation.w;
      double gppox = goal_pose.pose.orientation.x;
      double gppoy = goal_pose.pose.orientation.y;
      double gppoz = goal_pose.pose.orientation.z;
      double gpppx = goal_pose.pose.position.x;
      double gpppy = goal_pose.pose.position.y;
      double gpppz = goal_pose.pose.position.z;
      ROS_WARN("Part position in arm1_base_link's reference frame: (x,y,z) = (%f,%f,%f)", gpppx,gpppy,gpppz);
      ROS_WARN("Part orientation in arm1_base_link's reference frame: (w,x,y,z) = (%f,%f,%f,%f)", gppow,gppox,gppoy,gppoz);
      tf2::doTransform(part_pose, goal_pose, tfStamped);
      MoveArm(ik_client);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "entry_node");
  ros::NodeHandle n;
  p_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command", 1000);
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate loop_rate(10);
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
  ros::Subscriber joint_states = n.subscribe("ariac/arm1/joint_states", 10, JointState_Callback);
  ros::ServiceClient ik_client = n.serviceClient<ik_service::PoseIK>("/pose_ik");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  while(ros::ok()){
    loop_rate.sleep();
    GetOrder(MaterialLocations,ik_client);
  }
  return 0;
}

