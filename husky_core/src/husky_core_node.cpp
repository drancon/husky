#include <ros/ros.h>
/* ROS Message Header */
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/GoalStatus.h>
#include <nav_msgs/Odometry.h>
/* ROS Service Header */
#include <vehicle_msgs/Move.h>
/* ROS Packages */
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
/* C++ Header */
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <string>
#include <thread>

enum HuskyState {
  HUSKY_READY,
  HUSKY_NAVIGATION
};

class HuskyCore {
 public:
  HuskyCore(const ros::NodeHandle& nh);
 
 private:
  /* ROS message callback functions */
  void MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);
  void OdometryGlobalCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void ControllerReadyCallback(const nav_msgs::Odometry::ConstPtr& msg);
  /* ROS service callback functions */
  bool MoveServiceCallback(vehicle_msgs::Move::Request& req, vehicle_msgs::Move::Response& res);
  /* Task */
  void TaskThreadFunc(void);
  void TaskNavigationFunc(void);
  /* Functions */
  bool CheckMessageInitialization(void);

  /* ROS node handle */
  ros::NodeHandle nh_;

  /* ROS Subscriber */
  ros::Subscriber sub_move_base_status_;
  ros::Subscriber sub_odometry_global_;
  ros::Subscriber sub_controller_ready_;

  /* ROS subscribed messages */
  actionlib_msgs::GoalStatusArray::ConstPtr msg_sub_move_base_status_;
  nav_msgs::Odometry::ConstPtr msg_sub_odometry_global_;
  nav_msgs::Odometry::ConstPtr msg_sub_controller_ready_;

  /* ROS Publisher */
  ros::Publisher pub_goal_pose_;

  /* ROS Service Server */
  ros::ServiceServer srv_move_;

  /* variables for controlling this node */
  HuskyState state_;
  bool nav_success_;
  std::thread thread_task_;
  std::string param_pose_topic_;
  bool param_comp_yaw_;
  ros::Time last_stamp_;
};

/******************************************************************************/
/* Initialization                                                             */
/******************************************************************************/
HuskyCore::HuskyCore(const ros::NodeHandle& nh)
    : nh_(nh) {
  /* ROS parameters */
  nh_.getParam("pose_topic", param_pose_topic_);
  nh_.getParam("compensate_yaw", param_comp_yaw_);
  /* ROS Subscriber */
  sub_move_base_status_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1, &HuskyCore::MoveBaseStatusCallback, this);
  sub_odometry_global_ = nh_.subscribe<nav_msgs::Odometry>(param_pose_topic_, 1, &HuskyCore::OdometryGlobalCallback, this);
  sub_controller_ready_ = nh_.subscribe<nav_msgs::Odometry>("/husky_velocity_controller/odom", 1, &HuskyCore::ControllerReadyCallback, this);
  /* ROS publisher */
  pub_goal_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  /* ROS Service Server */
  srv_move_ = nh_.advertiseService("/husky_module/move", &HuskyCore::MoveServiceCallback, this);
  /* Main Thread */
  thread_task_ = std::thread(&HuskyCore::TaskThreadFunc, this);
  nav_success_ = false;
  // initialize the time stamp of last goal
  last_stamp_ = ros::Time::now();
}

/******************************************************************************/
/* Message callback functions                                                 */
/******************************************************************************/
void HuskyCore::MoveBaseStatusCallback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg) {
  msg_sub_move_base_status_ = msg;
}
void HuskyCore::OdometryGlobalCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  msg_sub_odometry_global_ = msg;
}
void HuskyCore::ControllerReadyCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  msg_sub_controller_ready_ = msg;
}

/******************************************************************************/
/* Task                                                                       */
/******************************************************************************/
void HuskyCore::TaskThreadFunc(void) {
  ros::Rate rate(10.0);

  while (!CheckMessageInitialization()) {
    rate.sleep();
  }

  state_ = HUSKY_READY;
  while (nh_.ok()) {
    TaskNavigationFunc();
    rate.sleep();
  }
}

/******************************************************************************/
/* Service callback functions                                                 */
/******************************************************************************/
bool HuskyCore::MoveServiceCallback(
    vehicle_msgs::Move::Request& req,
    vehicle_msgs::Move::Response& res) {

  if(!CheckMessageInitialization()) {
      ROS_INFO("Husky is not initialized");
      return false;
  }

  ros::Rate rate(30.0);

  /* ROS published messages */
  geometry_msgs::PoseStamped msg_goal_pose_;
  float yaw, dx, dy;

  // get goal pose
  msg_goal_pose_ = req.goal;
  // manipulate state value for smooth operation of mobile robot
  msg_goal_pose_.pose.position.z = 0.0;
  if (param_comp_yaw_) {
    // compute goal yaw
    dx = msg_goal_pose_.pose.position.x - msg_sub_odometry_global_->pose.pose.position.x;
    dy = msg_goal_pose_.pose.position.y - msg_sub_odometry_global_->pose.pose.position.y;
    yaw = atan2(dy, dx);
    // inpu goal yaw
    msg_goal_pose_.pose.orientation.z = sin(0.5*yaw);
    msg_goal_pose_.pose.orientation.w = cos(0.5*yaw);
  }
  
  // publish goal pose
  pub_goal_pose_.publish(msg_goal_pose_);
  // set the node's state as navigation mode
  state_ = HUSKY_NAVIGATION;

  while (state_ != HUSKY_READY) {
      ros::spinOnce();
      rate.sleep();
  }

  res.finished = nav_success_;
  return nav_success_;
}

void HuskyCore::TaskNavigationFunc(void) {
  switch (state_) {
    /***************************************************************/
    case HUSKY_NAVIGATION: {
      ROS_INFO("HUSKY_NAVIGATION");
      if (msg_sub_move_base_status_->status_list.size() > 0) {
        actionlib_msgs::GoalStatus current_status;
        current_status = msg_sub_move_base_status_->status_list.back();

        // check if the node is referencing the outdated goal
        if ( abs((current_status.goal_id.stamp - last_stamp_).toSec()) == 0.0 ) {
          break;
        }
          
        if (current_status.status == 3) {
          state_ = HUSKY_READY;
          nav_success_ = true;
          // store the time stamp of completed goal
          last_stamp_ = current_status.goal_id.stamp;
        } else if (current_status.status == 4) {
          state_ = HUSKY_READY;
          nav_success_ = false;
        }
      }
      break;
    }
    /***************************************************************/
    case HUSKY_READY:
      ROS_INFO("HUSKY_READY");
      break;
    /***************************************************************/
    default:
      ROS_INFO("ERROR");
  }
}

bool HuskyCore::CheckMessageInitialization(void) {
  if (msg_sub_move_base_status_ == nullptr) return false;
  if (msg_sub_odometry_global_ == nullptr) return false;
  if (msg_sub_controller_ready_ == nullptr) return false;
  return true;
}

/******************************************************************************/
/* Main                                                                       */
/******************************************************************************/
int main (int argc, char* argv[]) {
  ros::init(argc, argv, "husky_core");
  ros::NodeHandle nh("~");

  HuskyCore huskycore(nh);

  ros::Rate rate(100.0);
  while (nh.ok()) {
    ros::spinOnce();
    rate.sleep();
  }
  
  return 0;
}
