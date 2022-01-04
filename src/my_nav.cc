#include "my_nav.hh"

namespace dbot {

MoveBase::MoveBase(std::string name)
  : ac_("move_base", true), // SimpleActionClient(name, spin_thread)
    as_(nh_, name, boost::bind(&MoveBase::goal_callback, this, _1), false),
    action_name_(name)
{
  // To avoid race condtition,
  // start() should be called after construction of the server.
  as_.start();
  SetNodeState(NodeState::IDLE);
  ros_nh_ = ros::NodeHandle("~");
  odom_subscriber_ = ros_nh_.subscribe("/odom", 10, &MoveBase::OdomCB, this);
}

MoveBase::~MoveBase() {}

void MoveBase::init()
{
  // Temporary Private NodeHandle for getting parameters
  ros::NodeHandle tmp_nh("~");
  tmp_nh.param<int>("num_loops", params_.num_runs, 1);
  tmp_nh.param<std::string>("ref_frame", params_.ref_frame, "map");

  // Wait for Server connection
  while (!ac_.waitForServer(ros::Duration(3.0)))
    ROS_INFO("Waiting for the move_base action server to come up");
  ROS_INFO("Move_base Action Server is connected!!");
}

void MoveBase::goal_callback(const move_base_msgs::MoveBaseGoalConstPtr &goal)
{
  ros::NodeHandle nh("~");
  ros::Rate r(10);
  // Send a goal to the robot
  // move_base_msgs::MoveBaseGoal goal;
  // goal.target_pose.pose.position.x = 0.0;
  // goal.target_pose.pose.position.y = 0.0;
  // goal.target_pose.pose.position.z = -0.34;
  // goal.target_pose.pose.orientation.x = 0.0;
  // goal.target_pose.pose.orientation.y = 0.0;
  // goal.target_pose.pose.orientation.z = 0.0;
  // goal.target_pose.pose.orientation.w = 1.0;
  // goal_ = (*goal);
  // goal_.target_pose.header.frame_id = params_.ref_frame;
  // goal_.target_pose.header.stamp = ros::Time::now();

  // If Action server is idle, Run it. Otherwise, abort goal msg
  if (GetNodeState() == NodeState::IDLE)
  {
    ROS_INFO("StartThread()");
    StartThread();
  }
  else
  {
    ROS_INFO("Just ABORTED!!!!");
    as_.setAborted();
  }

  while (ros::ok()) // Start action loop
  {
    static unsigned long cnt = 0;
    if( (++cnt%20) == 0)
      ROS_INFO("Loop is ongoing ...... ");

    if (as_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      as_.setPreempted();
      StopThread();
      break;
    }

    if (GetNodeState() == NodeState::SUCCESS)
    {
      ROS_INFO("SUCCESS :: StopThread()");
      StopThread();
      as_.setSucceeded();
    }

    if (GetNodeState() == NodeState::FAILURE)
    {
      ROS_INFO("FAILURE :: StopThread()");
      StopThread();
      as_.setAborted();
      break;
    }

    r.sleep();
  }
}

void MoveBase::cancel_callback()
{
  ac_.cancelGoal();
}


void MoveBase::run()
{
  SetNodeState(NodeState::RUNNING);

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = params_.ref_frame;
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal_->target_pose.pose.position.x;
  goal.target_pose.pose.position.y = goal_->target_pose.pose.position.y;
  goal.target_pose.pose.position.z = 0.01;
  goal.target_pose.pose.orientation = goal_->target_pose.pose.orientation;
/// Debug Begin
  ROS_INFO("  Sent goal pose: \n\tP (%.3f, %.3f, %.3f),\n\tQ (%.3f, %.3f, %.3f, %.3f)",
          goal_->target_pose.pose.position.x,
          goal_->target_pose.pose.position.y,
          goal_->target_pose.pose.position.z,
          goal_->target_pose.pose.orientation.x,
          goal_->target_pose.pose.orientation.y,
          goal_->target_pose.pose.orientation.z,
          goal_->target_pose.pose.orientation.w );
  ac_.sendGoal(goal);
  ac_.waitForResult(ros::Duration(15.0));

  // Running Loop
  while (ros::ok() && GetNodeState() == NodeState::RUNNING)
  {
    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      SetNodeState(NodeState::SUCCESS);
      return;

    }
    else if ( ac_.getState() == actionlib::SimpleClientGoalState::ABORTED ||
              ac_.getState() == actionlib::SimpleClientGoalState::REJECTED)
    {
      SetNodeState(NodeState::FAILURE);
      return;
    }
    else
      continue;
  }
}

void MoveBase::SetNodeState(const NodeState &nodestate)
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  node_state_ = nodestate;
}

NodeState MoveBase::GetNodeState()
{
  boost::lock_guard<boost::mutex> guard(state_mtx_);
  return node_state_;
}

void MoveBase::StopThread()
{
  if (as_movebase_thread_.joinable())
    as_movebase_thread_.join();

  SetNodeState(NodeState::IDLE);
  ROS_INFO("Thread is stopped");
}

void MoveBase::StartThread()
{
  if (as_movebase_thread_.joinable())
    as_movebase_thread_.join();

  SetNodeState(NodeState::RUNNING);
  as_movebase_thread_ = boost::thread(&MoveBase::run, this);
  ROS_INFO("Thread starts");
}

};  // namespace dbot
