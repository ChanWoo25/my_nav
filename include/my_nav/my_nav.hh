// ros
#include <ros/ros.h>
// ros package to access package directory
#include <ros/package.h>
// move base
#include <move_base_msgs/MoveBaseAction.h>
// simple move action
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
// stamped point message
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
// tf2 matrix
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
// yaml file handling
#include <yaml-cpp/yaml.h>
//
#include <fstream>

#include "tf/transform_listener.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <thread>

namespace dbot {

enum NodeState {
  RUNNING, SUCCESS, FAILURE, IDLE
};

// parameter struct and typedef
struct MoveBaseParam {
  // waypoint filename
  std::string waypoints_filename;
  // reference frame. the robot wil move based on this frame
  std::string ref_frame;
  // number of runs
  int num_runs;
};

class MoveBase {
public:

  // Constructor & Bind 'action server' with ExecuteCB() and as_.start()
  // & Action client also starts with spin_thread=true
  MoveBase(std::string name);

  // Destructor
  ~MoveBase();

  // Get parameters & Connect with ActionServer
  void init();

  // Run waypoint navigation
  void run();

  inline double GoalDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  }

  // NodeState Setter (Manage mutex lock)
	void SetNodeState(const NodeState &nodestate);
  // NodeState Getter (Manage mutex lock)
	NodeState GetNodeState();

  // Start "&MoveBase::run" thread & Set NodeState RUNNING
	void StartThread();
  // Stop joinable thread & Set NodeState IDLE
	void StopThread();

  boost::thread as_movebase_thread_;
	boost::mutex state_mtx_;

  // contain robot's "/base_link" position according to "/map" coordinate
  tf::Vector3 current_pose_;

  std::string source_frameid = std::string("/map");
  std::string target_frameid = std::string("/base_link");


  // Instantiate a local listener
  tf::TransformListener echoListener;


 private:

  nav_msgs::Odometry odom_;
  ros::Subscriber odom_subscriber_;
  void OdomCB(const nav_msgs::Odometry& msg) {odom_=msg;}

  /// ros node handle
  ros::NodeHandle nh_;
  ros::NodeHandle ros_nh_;

  ///
  MoveBaseParam params_;
  /// input image file name including the path
  std::string waypoints_filename_;
  /// array of waypoint
  ros::Subscriber robot_status_sub_;

  // Tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> as_;


  void goal_callback(const move_base_msgs::MoveBaseGoalConstPtr &goal);
  void cancel_callback();

  std::string action_name_;

  NodeState node_state_;
  move_base_msgs::MoveBaseGoalConstPtr goal_;
};

}  // namespace dbot
