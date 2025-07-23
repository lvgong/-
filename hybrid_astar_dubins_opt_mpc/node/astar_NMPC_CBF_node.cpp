#include <csignal>
#include "astar_NMPC_CBF.h"

std::shared_ptr<chibot::topologyplanning::AstarNMPCCBF> astar_mpc_cbf_ptr;

void sigintHandler(int sig) {
  //对象已建立
  if (astar_mpc_cbf_ptr) {
    // hybrid_astar_dubins_opt_ptr->stop();//终止导航
    astar_mpc_cbf_ptr.reset();
  }
  ROS_INFO("hybrid astar dubins opt shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, chibot::topologyplanning::CBF_NODE_NAME);
  astar_mpc_cbf_ptr = std::make_shared<chibot::topologyplanning::AstarNMPCCBF>();
  signal(SIGINT, sigintHandler);//Ctrl+C触发
  astar_mpc_cbf_ptr->init();//初始化
  astar_mpc_cbf_ptr->run();//运行
  return 0;
}
