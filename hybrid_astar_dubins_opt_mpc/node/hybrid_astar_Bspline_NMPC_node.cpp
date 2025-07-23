#include <csignal>
#include "hybrid_astar_Bspline_NMPC.h"

std::shared_ptr<chibot::topologyplanning::HybridAstarBsplineNMPC> hybrid_astar_bspline_nmpc_ptr;

void sigintHandler(int sig) {
  //对象已建立
  if (hybrid_astar_bspline_nmpc_ptr) {
    // hybrid_astar_dubins_opt_ptr->stop();//终止导航
    hybrid_astar_bspline_nmpc_ptr.reset();
  }
  ROS_INFO("hybrid astar dubins opt shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, chibot::topologyplanning::NMPC_NODE_NAME);
  hybrid_astar_bspline_nmpc_ptr = std::make_shared<chibot::topologyplanning::HybridAstarBsplineNMPC>();
  signal(SIGINT, sigintHandler);//Ctrl+C触发
  hybrid_astar_bspline_nmpc_ptr->init();//初始化
  hybrid_astar_bspline_nmpc_ptr->run();//运行
  return 0;
}
