#pragma once

#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "lookup.h"
// #include "node2d.h"
#include "node3d.h"
#include "hybrid_astar_searcher/type_defs.h" 

namespace planning {
// namespace {

  /**
   * @brief Get the Configuration object of a node: node
   *        查询给定节点的configure (构型值)，分两种情况：
   *        - 若为Node2D类，theta分量恒为99
   *        - 若为Node3D类，theta分量为node的真实值
   * @param node: 节点指针
   * @param x 节点node的X分量
   * @param y 节点node的X分量
   * @param t 节点node的theta分量
   */
// void getConfiguration(const Node2D* node, float& x, float& y, float& t) {
//   x = node->getX();
//   y = node->getY();
//   // avoid 2D collision checking
//   t = 99; //2D的getConfiguration为多态函数，2D网格时统一将t=99
// }

// void getConfiguration(const std::shared_ptr<Node3d> node, float& x, float& y, float& theta) {
//   x = node->getX();
//   y = node->getY();
//   theta = node->getTheta();
// }
// }
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.
  CollisionDetection用于检测给定机器人的一个位姿下是否与环境发生碰撞
   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection {
 public:
    CollisionDetection(); //构造函数

  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  // 碰撞检测
  bool isTraversable(Vec3d pose) {
    //可通行性检验：1) 标准方法：使用空间占据计算检查；2) 其他：使用2D代价地图和导航器检查
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
   //  float cost = 0;
   //  float x = pose(0);
   //  float y = pose(1);
   //  float theta = pose(2);
    // assign values to the configuration
    // getConfiguration(node, x, y, theta);

    // 2D collision test: 2D网格就直接检测环境网格中的占据情况
    // if (t == 99) {//若为Node2D类，theta分量恒为99
    //   return !grid->data[node->getIdx()];
    // }

   //  if (true) {
      float cost = configurationTest(pose(0), pose(1), pose(2)) ? 0 : 1;//如果true，表明为自由网格，通行代价为0；有障碍则返回false，通行代价值为1
   //  } else {
      // cost = configurationCost(x, y, theta);
   //  }

    return cost <= 0;// 代价值最多为0时表示可通行
  }

  bool isTraversable(int X, int Y, float Theta) {
    //可通行性检验：1) 标准方法：使用空间占据计算检查；2) 其他：使用2D代价地图和导航器检查
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
   //  float cost = 0;
   //  float x = pose(0);
   //  float y = pose(1);
   //  float theta = pose(2);
    // assign values to the configuration
    // getConfiguration(node, x, y, theta);

    // 2D collision test: 2D网格就直接检测环境网格中的占据情况
    // if (t == 99) {//若为Node2D类，theta分量恒为99
    //   return !grid->data[node->getIdx()];
    // }

   //  if (true) {
      float cost = configurationTest_(X, Y, Theta) ? 0 : 1;//如果true，表明为自由网格，通行代价为0；有障碍则返回false，通行代价值为1
   //  } else {
      // cost = configurationCost(x, y, theta);
   //  }

    return cost <= 0;// 代价值最多为0时表示可通行
  }

  /*!
     \brief updates the grid with the world map 更新栅格地图，可以用来动态更新
  */
  void updateGrid(const nav_msgs::OccupancyGridConstPtr& map) {grid = map;}


   private:
  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
//   float configurationCost(float x, float y, float t) {return 0;}

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t);
  bool configurationTest_(int X, int Y, float Theta);


 private:
  // The occupancy grid
  // nav_msgs::OccupancyGrid::Ptr grid;
  nav_msgs::OccupancyGridConstPtr grid;

  // The collision lookup table
  // 这是个结构体(config)数组，数组数量是headings*positions
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};

}
#endif // COLLISIONDETECTION_H
