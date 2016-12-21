#include "decision_process/decision_process_node.hpp"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "decision_process"); //Name of the node
  decision_process Node;

  //ros::Duration(2).sleep();

  while(Node.nh.ok())
  {
    ros::spin();
  }
  return 0;
}


