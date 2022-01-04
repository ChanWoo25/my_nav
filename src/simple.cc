#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/ModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


std::vector<std::pair<double, double>> traj;
std::vector<double> paths_length;

void doneCB(const actionlib::SimpleClientGoalState& state,
            const move_base_msgs::MoveBaseResultConstPtr &result)
{
  if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    std::cout << "[doneCB] -- Navigation fail" << std::endl;
  }
  else
  {
    if (traj.size() <= 1)
    {
      std::cout << "[doneCB] -- There is no trajectory" << std::endl;
    }
    else
    {
      double length = 0.0;
      for(int i=1; i<traj.size(); i++)
      {
        double x1 = traj[i-1].first;
        double y1 = traj[i-1].second;
        double x2 = traj[i].first;
        double y2 = traj[i].second;
        length += sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
      }
      std::cout << "[doneCB] -- Total length is " << length << std::endl;
      paths_length.push_back(length);
    }
  }
  std::cout << "[doneCB] -- traj.clear() " << std::endl;
  traj.clear();
}

void activeCB()
{
  std::cout << "Active Callback!" << std::endl;
}

void feedbackCB(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
{
  auto x = double(feedback->base_position.pose.position.x);
  auto y = double(feedback->base_position.pose.position.y);
  traj.push_back(std::make_pair(x, y));
}


int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  ros::NodeHandle nh;
  ros::Publisher model_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

  double goal_x = 0.5;
  double goal_y = 0.5;
  double goal_yaw = 3.14;

  int total_test = 10;
  int success = 0;
  int failure = 0;

  for(int i=0; ros::ok() && i < total_test; i++)
  {

    gazebo_msgs::ModelState model_state_msg;
    model_state_msg.model_name = "turtlebot3_waffle_pi";
    model_state_msg.reference_frame = "map";
    model_state_msg.pose.position.x = goal_x;
    model_state_msg.pose.position.y = goal_y;
    model_state_msg.pose.position.z = -0.001;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, goal_yaw);
    myQuaternion.normalize();
    model_state_msg.pose.orientation.x = myQuaternion.getX();
    model_state_msg.pose.orientation.y = myQuaternion.getY();
    model_state_msg.pose.orientation.z = myQuaternion.getZ();
    model_state_msg.pose.orientation.w = myQuaternion.getW();
    int cnt(10);
    while(ros::ok() && --cnt > 0)
    {
      model_state_pub.publish(model_state_msg);
      ros::Rate(5.0).sleep();
    }

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = -0.5;
    goal.target_pose.pose.position.y = -0.5;
    goal.target_pose.pose.position.z = 0.1;
    goal.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal, &doneCB, &activeCB, &feedbackCB);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Hooray, the base moved 1 meter forward");
      success++;
    }
    else
    {
      ROS_INFO("The base failed to move forward 1 meter for some reason");
      failure++;
    }
  }

  // Average Length
  assert(success == paths_length.size());
  double avg_len = 0.0;
  for(auto len: paths_length)
    avg_len += len;
  avg_len =avg_len / paths_length.size();

  std::cout << "ALL TEST IS OVER" << std::endl;
  std::cout << "total: " << total_test << std::endl;
  std::cout << "success rate: " << (double)success / (double)total_test << std::endl;
  std::cout << "failure rate: " << (double)failure / (double)total_test << std::endl;
  std::cout << "average traj length in success case: " << avg_len << std::endl;

  return 0;
}
