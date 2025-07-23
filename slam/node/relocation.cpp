#include <cmath>
#include <ros/ros.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "tf2/utils.h"

namespace chibot::slam {

  #define degree2rad (M_PI / 180.0)

  class ChibotRelo {
    public:
      ChibotRelo();
      ~ChibotRelo() = default;

    private:
      // 接收初始化位置，大概的一个位置，我们进行二次匹配
      void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
      // 接收地图
      void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);
      // 接收激光信息
      void laserReceived(const sensor_msgs::LaserScanConstPtr& msg);
      // 范围重定位（搜索范围2m，60度），分辨率0.05m（同地图分辨率）和0.5度（同雷达分辨率），将新位置赋给amcl
      auto rangeRelocate(geometry_msgs::PoseWithCovarianceStamped& best_pose,
                        double dist_range = 2.0, double angle_range = 60.0 * degree2rad,
                        double dist_reso = 0.05, double angle_reso = 0.5 * degree2rad) -> int;

    private:
      ros::Publisher initial_pose_pub_;
      ros::Subscriber initial_pose_sub_;
      ros::Subscriber map_sub_;
      ros::Subscriber laser_sub_;

      // 表示占据栅格地图
      nav_msgs::OccupancyGrid map_{};// {}列表初始化
      sensor_msgs::LaserScan laser_{};

      std::vector<std::pair<double, double>> cos_sin_table_{};

      std::atomic_bool on_going_{false};
      std::atomic_bool got_laser_info_{false};
  };

  ChibotRelo::ChibotRelo() {
    ros::NodeHandle nh;
    //将新的best_pose给amcl
    initial_pose_pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
    //接收用户给的pose，进行匹配计算
    initial_pose_sub_ = nh.subscribe("/initialpose_ori", 1, &ChibotRelo::initialPoseReceived, this);
    map_sub_ = nh.subscribe("/map", 1, &ChibotRelo::mapReceived, this);
    laser_sub_ = nh.subscribe("/scan", 1, &ChibotRelo::laserReceived, this);
  }

  void ChibotRelo::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    on_going_ = true;//接收到初始化位姿后，置1，表示开始进行匹配重定位
    auto best_pose = *msg;//接收原始pose
    ROS_INFO("Receive original initial pose for amcl node [%.3f, %.3f, %.3f]",
            msg->pose.pose.position.x, msg->pose.pose.position.y, tf2::getYaw(msg->pose.pose.orientation));
    auto start_time = ros::Time::now();
    auto score = rangeRelocate(best_pose);//进行二次匹配，得到新的bset_pose
    ROS_WARN("Get new best pose [%.3f, %.3f, %.3f] score [%d], time go %.3f",
            best_pose.pose.pose.position.x, best_pose.pose.pose.position.y, tf2::getYaw(best_pose.pose.pose.orientation),
            score, (ros::Time::now() - start_time).toSec());
    initial_pose_pub_.publish(best_pose);//分布匹配后的位姿
    on_going_ = false;
  }

  //接收地图信息
  void ChibotRelo::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg) {
    if (on_going_) return;
    map_ = *msg;
  }

  //接收雷达信息
  void ChibotRelo::laserReceived(const sensor_msgs::LaserScanConstPtr& msg) {
    if (on_going_) return;
    laser_ = *msg;
    if (got_laser_info_) return;
    auto start_time = ros::Time::now();
    got_laser_info_ = true;//表示接收到雷达信息了
    cos_sin_table_.resize(laser_.ranges.size());//雷达采样数
    for (auto i = 0; i < laser_.ranges.size(); ++i) {
      cos_sin_table_[i].first = std::cos(laser_.angle_min + i * laser_.angle_increment);//当前采样雷达线的cos值
      cos_sin_table_[i].second = std::sin(laser_.angle_min + i * laser_.angle_increment);
    }
    ROS_INFO("Calculate table size %lu time go %.3f",
            cos_sin_table_.size(), (ros::Time::now() - start_time).toSec());
  }

  //二次重定位
  auto ChibotRelo::rangeRelocate(geometry_msgs::PoseWithCovarianceStamped& best_pose,
                              double dist_range, double angle_range, double dist_reso, double angle_reso) -> int {
    //思路:在所给范围中寻找障碍点，将障碍点累加计算得分

// 使用Lambda表达式定义一个名为mapValid的函数对象。这个函数对象接受两个double类型的参数x和y，并根据一些地图信息判断给定的坐标(x, y)是否在地图范围内。
// auto mapValid = [&](double x, double y) { ... };：这里使用了 Lambda 表达式来定义一个函数对象 mapValid，它接受两个参数 x 和 y，并且捕获了所有外部变量（包括 map_）作为引用。捕获列表 [&] 表示捕获所有外部变量。
// auto i = std::floor((x - map_.info.origin.position.x) / map_.info.resolution + 0.5);：这一行计算出给定 x 坐标在地图中对应的列号 i。首先将 x 减去地图原点的 x 坐标，然后除以分辨率，再加上 0.5 后向下取整，得到离原点最近的列号。
// auto j = std::floor((y - map_.info.origin.position.y) / map_.info.resolution + 0.5);：类似地，这一行计算出给定 y 坐标在地图中对应的行号 j。
// return (i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height);：最后，函数返回一个布尔值，表示坐标 (x, y) 是否在地图的有效范围内。条件是 i 和 j 都大于等于 0，且都小于地图的宽度和高度。

    //判断所给pose是否超出地图范围
    //这里只是函数的定义，函数的使用在下文
    auto mapValid = [&](double x, double y) {
      auto i = std::floor((x - map_.info.origin.position.x) / map_.info.resolution + 0.5);
      auto j = std::floor((y - map_.info.origin.position.y) / map_.info.resolution + 0.5);
      return (i >= 0 && j >= 0 && i < map_.info.width && j < map_.info.height);
    };

    // 判断是否是障碍物
// auto idx = i + j * map_.info.width;：这一行计算出给定坐标在地图 data 数组中的下标 idx。由于地图数据存储方式通常是一维数组，所以需要将二维坐标转换为一维下标。具体计算方法是将 j 乘以地图宽度，再加上 i。
// return map_.data[idx] == 100;：最后，函数返回一个布尔值，表示给定坐标是否为障碍物。如果地图 data 数组中 idx 对应的值为 100，就认为该坐标是障碍物，返回 true；否则返回 false。
    auto mapObstacle = [&](double x, double y) {
      auto i = std::floor((x - map_.info.origin.position.x) / map_.info.resolution + 0.5);
      auto j = std::floor((y - map_.info.origin.position.y) / map_.info.resolution + 0.5);
      auto idx = i + j * map_.info.width;
      return map_.data[idx] == 100;//100为障碍物像素值
    };

    // 计算给定坐标 (x, y) 在指定角度下的得分
    auto calcuScore = [&](double x, double y, double cos, double sin) {
      const double laser2base = 0.0;//表示激光雷达相对于基准位置的偏移距离(平面距离)
      auto score = 0;//初始化得分为 0
      // transform to laser frame
      auto laser_x = x + laser2base * cos;//将给定坐标 (x, y) 按照指定角度 cos 平移到激光雷达坐标系中的 x 轴
      auto laser_y = y + laser2base * sin;
      //遍历激光雷达的测距数据，每隔 10 个数据点进行一次计算
      for (auto i = 0; i < laser_.ranges.size(); i += 10) {
        //判断当前测距值是否在有效范围内，若不在则跳过继续处理下一个数据点
        if (laser_.ranges[i] < laser_.range_min || laser_.ranges[i] >= laser_.range_max) continue;
        //根据指定角度 cos 和 sin，计算当前激光雷达数据点的 cos(pth) 值
        //cos(a+b)=cosa*cosb-sina*sinb
        //sin(a+b)=sina*cosb+cosa*sinb
        auto cos_pth = cos * cos_sin_table_[i].first - sin * cos_sin_table_[i].second;
        auto sin_pth = sin * cos_sin_table_[i].first + cos * cos_sin_table_[i].second;
        // 将当前激光雷达数据点按照 cos(pth) 方向上的距离进行平移，得到在激光雷达坐标系中的 x 坐标
        auto px = laser_x + laser_.ranges[i] * cos_pth;
        auto py = laser_y + laser_.ranges[i] * sin_pth;
        // 判断平移后的坐标是否在地图有效范围内，若不在则跳过继续处理下一个数据点
        if (!mapValid(px, py)) continue;
        // 调用之前定义的 mapObstacle 函数，判断平移后的坐标是否为障碍物，若是则增加得分
        if (mapObstacle(px, py)) ++score;
      }
      return score;
    };

    //在初始pose一定距离和朝向角度中匹配
    auto min_x = best_pose.pose.pose.position.x - dist_range / 2.0;
    auto max_x = best_pose.pose.pose.position.x + dist_range / 2.0;
    auto min_y = best_pose.pose.pose.position.y - dist_range / 2.0;
    auto max_y = best_pose.pose.pose.position.y + dist_range / 2.0;
    auto min_th = tf2::getYaw(best_pose.pose.pose.orientation) - angle_range / 2.0;
    auto max_th = tf2::getYaw(best_pose.pose.pose.orientation) + angle_range / 2.0;

    auto score = 0;
    double target_x, target_y, target_th;
    // 搜索在指定范围内的最佳目标位置和角度
    //遍历扫描角度（弧度制）
    for (auto th = min_th; th < max_th; th += angle_reso) {
      auto cos_th = std::cos(th);
      auto sin_th = std::sin(th);
      for (auto x = min_x; x <= max_x; x += dist_reso) {
        for (auto y = min_y; y <= max_y; y += dist_reso) {
          if (!mapValid(x, y) || mapObstacle(x, y)) continue;//判断当前坐标是否有效且不是障碍物
          auto temp = calcuScore(x, y, cos_th, sin_th);//计算当前位置和角度下的得分
          if (temp > score) {//判断当前得分是否大于之前的最高得分
            score = temp;//更新最高得分
            target_x = x;//更新目标位置的 x 坐标
            target_y = y;
            target_th = th;//更新目标角度
          }
        }
      }
    }
    //得分最高者说明匹配到的障碍物最多，就是最佳的位姿
    //最终，得到在指定范围内得分最高的目标位置和角度
    

    //得分最高者作为新的位置赋给amcl
    best_pose.pose.pose.position.x = target_x;
    best_pose.pose.pose.position.y = target_y;
    best_pose.pose.pose.orientation.z = std::sin(target_th / 2.0);
    best_pose.pose.pose.orientation.w = std::cos(target_th / 2.0);
    return score;
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "chibot_relocation");

  chibot::slam::ChibotRelo _chibot_relo;
  ros::spin();

  return 0;
}
