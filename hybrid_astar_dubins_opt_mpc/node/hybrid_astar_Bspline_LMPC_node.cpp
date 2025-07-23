#include <csignal>
#include "hybrid_astar_Bspline_LMPC.h"

std::shared_ptr<chibot::topologyplanning::HybridAstarBsplineLMPC> hybrid_astar_bspline_lmpc_ptr;

void sigintHandler(int sig) {
  //对象已建立
  if (hybrid_astar_bspline_lmpc_ptr) {
    // hybrid_astar_dubins_opt_ptr->stop();//终止导航
    hybrid_astar_bspline_lmpc_ptr.reset();
  }
  ROS_INFO("hybrid astar dubins opt shutting down!");
  ros::shutdown();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, chibot::topologyplanning::LMPC_NODE_NAME);
  hybrid_astar_bspline_lmpc_ptr = std::make_shared<chibot::topologyplanning::HybridAstarBsplineLMPC>();
  signal(SIGINT, sigintHandler);//Ctrl+C触发
  hybrid_astar_bspline_lmpc_ptr->init();//初始化
  hybrid_astar_bspline_lmpc_ptr->run();//运行
  return 0;
}
