

#include "mpc_bridge.h"

int main (int argc, char** argv) {
  ros::init(argc, argv, chibot::topologyplanning::NODE_NAME);
  chibot::topologyplanning::MpcBridge _bridge_node;
  _bridge_node.init();

  ros::spin();
  return 0;
}
