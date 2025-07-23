开始录制:rosbag record -a -O 目标文件
rosbag record /tf_static /tf /cmd_vel /odom /fusion_analysis
rosbag record /fusion_analysis
先运行rosrun chibot_simu chibot_bridge_node

结束录制:使用 ctrl + c，在创建的目录中会生成bag文件

查看文件:rosbag info 文件名

回放文件:rosbag play 文件名

rqt_bag 文件名
