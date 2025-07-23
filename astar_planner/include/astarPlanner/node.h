#ifndef _NODE_H_
#define _NODE_H_

#include <ros/console.h>
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>

/*这行代码定义了一个名为 inf 的宏，其值为 1 >> 20
1 >> 20 是一个位运算表达式。在这个表达式中，1 被右移了 20 位。右移操作将二进制数向右移动指定的位数，并且在左侧用零填充。因此，1 >> 20 的结果是一个非常小的数字，接近于零*/
#define inf 1 >> 20
struct Node2D;
typedef Node2D* Node2DPtr;

//节点(2D)
/*这段代码定义了一个名为 Node2D 的结构体，它表示二维空间中的一个节点。
该结构体包含以下成员变量：
id：表示节点属于哪个集合，1 表示在 open set 中，-1 表示在 closed set 中。
coord：表示节点在二维空间中的坐标。
dir：表示从哪个方向到达该节点。
index：表示节点在栅格地图中的索引。
gScore：表示从起点到该节点的实际代价，即 g 值。
fScore：表示从起点到该节点的估计代价，即 f 值。
predecessor：指向该节点的前驱节点。
nodeMapIt：表示该节点在 open set 中的迭代器。

该结构体还包含以下函数：
Node2D(Eigen::Vector2i _index, Eigen::Vector2d _coord)：结构体的构造函数，用于初始化节点的属性。其中 _index 表示节点在栅格地图中的索引，_coord 表示节点在二维空间中的坐标。
Node2D(){}：结构体的默认构造函数。
~Node2D(){}：结构体的析构函数。
这个结构体主要用于 A* 算法中的路径搜索，节点的各个属性记录了在算法中所需的信息。*/
struct Node2D {
  int id;  // 1--> open set, -1 --> closed set
  Eigen::Vector2d coord;
  Eigen::Vector2i dir;  // direction of expanding
  Eigen::Vector2i index;

  double gScore, fScore;
  Node2DPtr predecessor;
  std::multimap<double, Node2DPtr>::iterator nodeMapIt;

  //构造函数
  Node2D(Eigen::Vector2i _index, Eigen::Vector2d _coord) {
    id = 0;
    index = _index;
    coord = _coord;
    dir = Eigen::Vector2i::Zero();

    gScore = inf;
    fScore = inf;
    predecessor = NULL;
  }
  Node2D(){};
  ~Node2D(){};
};

#endif
