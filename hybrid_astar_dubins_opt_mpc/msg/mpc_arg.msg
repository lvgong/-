# 还需要计算耗时、第一步优化量uk，第一步参考量ur(0)
# 各控制周期的第一步参考量 # ur(0):vr(0),wr(0)
float32 v_r #第一步的参考线速度
float32 w_r #
float32 x_r #第一步的参考x
float32 y_r
float32 theta_r
float32 time # 计算耗时，max，min，mean可以在PlotJuggler中直接得到

# 实际位姿,实际控制量uk由cmd_vel给出
float32 x_k
float32 y_k
float32 theta_k
# 当前控制量uk:v,w  由cmd_vel给出
# float32 vk #线速度指令
# float32 wk #角速度指令

# 偏差
float32 dv_k #线速度误差-控制量指令
float32 dv_c #线速度误差-实际控制量
float32 dw_k #角速度误差
float32 dw_c #角速度误差
float32 dx #横向位置误差
float32 dy #纵向位置误差
float32 dtheta #航向角误差

#x-v,y-w曲线可以在PlotJuggler中直接得到