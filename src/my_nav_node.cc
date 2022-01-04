#include "my_nav.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_nav_node");

  dbot::MoveBase movebase("move_base");
  movebase.init();

  try {
    ros::spin();
  }
  catch (std::runtime_error &e) {
    ROS_ERROR("ros spin failed: %s", e.what());
    return -1;
  }

  // If MoveBase::run() is running, Let it join.
  movebase.as_movebase_thread_.join();
  return 0;
}
