

#include "chibot_bridge.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, chibot::simu::NODE_NAME);
  chibot::simu::ChibotBridge _bridge_node;
  _bridge_node.init();

  ros::spin();
  return 0;
}
