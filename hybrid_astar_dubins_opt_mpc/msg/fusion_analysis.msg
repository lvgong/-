float32 v_k #线速度指令
float32 v_c #线速度反馈
float32 w_k #角速度指令
float32 w_c #角速度反馈
# float32 lwheel_control #左轮转速指令
# float32 lwheel_feedback #左轮转速反馈
# float32 lwheel_acc #左轮加速度
# float32 rwheel_control #右轮转速指令
# float32 rwheel_feedback #右轮转速反馈
# float32 rwheel_acc #右轮加速度

# 还需要计算耗时、第一步优化量uk，第一步参考量ur(0)

# mpc_pose odom_pose #odom位置
mpc_arg mpc #mpc计算的一些参数