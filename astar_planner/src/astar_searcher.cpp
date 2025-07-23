#include "astarPlanner/astar_searcher.h"


//类的构造函数是类的一种特殊的成员函数，它会在每次创建类的新对象时执行。
//构造函数的名称与类的名称是完全相同的，并且不会返回任何类型，也不会返回 void。构造函数可用于为某些成员变量设置初始值。
AstarPathFinder::AstarPathFinder() :
  total_length(0.0),
  node2com(nullptr),
  turn(0) {

  //接收膨胀（2格）后的地图-----改为接收添加障碍物信息后的
  // map_sub = nh.subscribe("/map_inflationdata_two", 1, &AstarPathFinder::setMapCallBack, this);//map_inflationdata
  map_sub = nh.subscribe("/local_grid_map", 10, &AstarPathFinder::setMapCallBack, this);
  //获取到小车位姿，作为起点
  // start_sub = nh.subscribe("/my_odom", 1,&AstarPathFinder::setStartCallback, this);
  //rviz中标记的目标点
  // goal_sub = nh.subscribe("/move_base_simple/goal", 1,&AstarPathFinder::setGoalCallback, this);
 

  //start_pub = nh.advertise<geometry_msgs::PoseStamped>("startInRviz", 1, true);
  //goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goalInRviz", 1, true);
  //_grid_path_vis_pub =nh.advertise<visualization_msgs::MarkerArray>("grid_path_vis", 1, true);
  //_visited_nodes_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visited_nodes_vis", 1, true);
 

  //A* path在rviz中显示
  // path_pub = nh.advertise<nav_msgs::Path>("nav_path", 1);
  //发布local_goal，DWA用
  // local_goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
  //smoothedPath_pub = nh.advertise<nav_msgs::Path>("smoothed_path", 1);

}


//接收膨胀map后的回调函数-----改为接收到变化的栅格地图
void AstarPathFinder::setMapCallBack(const nav_msgs::OccupancyGrid::Ptr map) {
  // ROS_INFO("111111");
  boost::mutex::scoped_lock lock(map_mutex_);
  map_ = *map;
  max_x_id = map_.info.width;//地图的宽
  max_y_id = map_.info.height;//地图的长
  X_SIZE = max_x_id;//X方向大小，实际上是对应方向的栅格数
  Y_SIZE = max_y_id;
  grid_number = X_SIZE * Y_SIZE;//总栅格数
  resolution = map_.info.resolution;//地图的分辨率,栅格地图分辨率对应栅格地图中一小格的长和宽,单位m/cell
  inv_resolution = 1.0 / resolution;//分辨率的倒数，除转乘
  originX = map_.info.origin.position.x;//地图原点x
  originY = map_.info.origin.position.y;//地图原点y

  // 动态分配数组
  data = new int8_t[grid_number];//存储栅格地图，障碍物的占用概率(1:占用 0:未占用)
  // 获取OccupancyGrid中data的指针
  int8_t* dataPtr = map_.data.data();
  // 将data数据拷贝至动态分配的一维数组中
  std::copy(dataPtr, dataPtr + grid_number, data);


  //膨胀栅格---碰撞检测算法不需要
  // const double chibot_rad = 0.3;/////
  // int inf_num = ceil(chibot_rad*inv_resolution);
  // // ROS_INFO("inf_num= %d" ,inf_num);
  // set<int> o={};
  // find_gezi(max_x_id, grid_number, data, inf_num, o);
  //  std::vector<signed char> p(a, a+counts);
  //   output.data=p;
  //   copy_map(output,carto_map_inflation);

  GridNodeMap = new Node2DPtr*[max_x_id];
  for (int i = 0; i < max_x_id; i++) {
    GridNodeMap[i] = new Node2DPtr[max_y_id];
    for (int k = 0; k < max_y_id; k++) {
      Vector2i tmpIdx(i, k);                      // i,k为栅格索引
      Vector2d pos = gridIndex2coord(tmpIdx);     //将栅格索引转换为空间真实坐标
      GridNodeMap[i][k] = new Node2D(tmpIdx, pos);//初始化每个栅格，设定索引与真实坐标的对应关系
    }
  }

  // initGridMap();//更新栅格地图
  //是否转换成功
  validMap = true;

}


/*
* @function     AstarPathFinder::initGridMap
* @brief        初始化每个栅格，设定索引与真实坐标的对应关系
* @param        double resolution          地图分辨率
                int max_x_id                X轴栅格的最多个数
                int max_y_id                Y轴栅格的最多个数
* @return       None
*/
void AstarPathFinder::initGridMap() {
  // ROS_INFO("grid map initialing");

  X_SIZE = max_x_id;//X方向大小，实际上是对应方向的栅格数
  Y_SIZE = max_y_id;
  grid_number = X_SIZE * Y_SIZE;//总栅格数
  inv_resolution = 1.0 / resolution;//分辨率的倒数，除转乘
  data = new int8_t[grid_number];//存储栅格地图，障碍物的占用概率(1:占用 0:未占用)
  //ROS_INFO("xl:%f ,yl,%f, xu,%f, yu,%f, XSIZE, %d, ySIZE, %d, reselution, %f",
  //         xl, yl, xu, yu, X_SIZE, Y_SIZE, resolution);
  memset(data, 0, grid_number * sizeof(int));//置零栅格地图  栅格总数乘每个栅格的占用空间
  // void *memset(void *buffer, int c, int count) 
  /*
  buffer：为指针或者数组
  c:为赋给buffer的值
  count：是buffer的长度
  */
  //data数值处理
  //map_为膨胀后的占用栅格：代价值变成了0（空闲）或100（障碍）
  if (map_.data.size() > 0) {
    for (int x = 0; x < X_SIZE; ++x) {//int x = 0; x < X_SIZE; ++x
      for (int y = 0; y < Y_SIZE; ++y) {//int y = 0; y < Y_SIZE; ++y
        //map_.data的代价值=100就赋1，等于0就赋0
        // data[y * X_SIZE + x] = map_.data[y * X_SIZE + x] ? 1 : 0;
          int index_ = y * X_SIZE + x;
          if (map_.data[index_] == 100 || map_.data[index_] == -1) {
              data[index_] = 1;//表示该位置是障碍物
          } 
          else {
              data[index_] = 0;
          }
        

        /*if(map_.data[y * X_SIZE + x]==0)
        {
        	data[y * X_SIZE + x]= 0;
        }
        else
        {
        	data[y * X_SIZE + x]= 1;
        }*/
      }
    }
  }
 /* for(int i =0;i<1000;i++)
  {
  	cout<<data[i]<<endl;
  }
  */
  //二维数组
/*创建一个名为 GridNodeMap 的二维指针数组，并初始化了其中的每个元素。
首先，通过 new Node2DPtr*[X_SIZE] 分配了一个长度为 X_SIZE 的指针数组。然后，使用一个循环遍历数组的每个元素，分别将它们初始化为指向 Node2DPtr 类型的指针。
在内部的循环中，对于每个 GridNodeMap[i]，又通过 new Node2DPtr[Y_SIZE] 分配了一个长度为 Y_SIZE 的指针数组。然后，再次使用一个循环遍历该数组的每个元素，为每个 GridNodeMap[i][k] 分配一个新的 Node2D 对象。
在 Node2D 对象的构造函数中，使用参数 tmpIdx 和 pos 创建了一个新的 Node2D 实例，并将它赋值给 GridNodeMap[i][k]。这样，GridNodeMap 数组中的每个元素都指向一个具有不同索引和位置的 Node2D 对象。
这段代码的目的是创建一个二维的网格地图，并将每个栅格与其索引和真实坐标进行关联。*/
  GridNodeMap = new Node2DPtr*[X_SIZE];
  for (int i = 0; i < X_SIZE; i++) {
    GridNodeMap[i] = new Node2DPtr[Y_SIZE];
    for (int k = 0; k < Y_SIZE; k++) {
      Vector2i tmpIdx(i, k);                      // i,k为栅格索引
      Vector2d pos = gridIndex2coord(tmpIdx);     //将栅格索引转换为空间真实坐标
      GridNodeMap[i][k] = new Node2D(tmpIdx, pos);//初始化每个栅格，设定索引与真实坐标的对应关系
    }
  }
  // ROS_INFO("grid map initialization is over");
}


/*
* @function     AstarPathFinder::resetGrid
* @brief        重置某个栅格
* @param        GridNodePtr ptr        指向一个栅格的指针
* @return       None
*/
void AstarPathFinder::resetGrid(Node2DPtr ptr) {
  ptr->id = 0;            //该节点是否被访问过，1 -> 未访问, -1 --> 已访问
  ptr->predecessor = NULL;//该节点来自哪，用于找父节点
  ptr->gScore = inf;      //g(n)的代价，开始设为无穷大  
  ptr->fScore = inf;      //f(n)的代价，开始设为无穷大
}


/*
* @function     AstarPathFinder::resetUsedGrids
* @brief        重置整张栅格地图
* @param        None
* @return       None
*/
void AstarPathFinder::resetUsedGrids() {
  for (int i = 0; i < X_SIZE; i++)
    for (int j = 0; j < Y_SIZE; j++) resetGrid(GridNodeMap[i][j]);//重置某个栅格
}


/*
* @function     AstarPathFinder::getVisitedNodes
* @brief        显示访问过的栅格
* @param        None
* @return       vector<Vector2d> visited_nodes

vector<Vector2d> AstarPathFinder::getVisitedNodes() {
  vector<Vector2d> visited_nodes;
  visited_nodes.clear();
  for (int i = 0; i < X_SIZE; i++)
    for (int j = 0; j < Y_SIZE; j++)
      // if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and
      // close list
      if (GridNodeMap[i][j]->id == -1)  // visualize nodes in close list only
        visited_nodes.push_back(GridNodeMap[i][j]->coord);
  //ROS_WARN("visited_nodes size : %d", visited_nodes.size());
  return visited_nodes;
}*/


/*
* @function     AstarPathFinder::gridIndex2coord
* @brief        将某个栅格的索引转化为欧式空间中的坐标(取栅格的中心位置)
* @param        Vector2i & index        栅格的索引向量
* @return       Vector2d pt             对应的欧式空间坐标向量
*/
/*这段代码定义了一个名为 gridIndex2coord 的函数，用于将栅格地图中的索引转换为二维空间中的坐标。
该函数接受一个 Vector2i 类型的参数 index，表示栅格地图中的索引。函数首先创建一个 Vector2d 类型的变量 pt，用于表示该索引对应的二维空间中的坐标。
接下来，函数根据栅格地图的分辨率和原点的坐标，计算出该栅格在实际地图中的位置。具体来说，对于栅格地图中的每个栅格，函数将其索引加上 0.5，然后乘以分辨率，最后加上原点的坐标，就可以得到该栅格在实际地图中的正中央的位置。
最后，函数返回计算得到的坐标 pt。
这个函数主要用于将栅格地图中的索引转换为实际地图中的坐标，以便进行路径规划。*/
Vector2d AstarPathFinder::gridIndex2coord(const Vector2i& index) {
  Vector2d pt;
  pt(0) = ((double)index(0) + 0.5) * resolution + originX;
  pt(1) = ((double)index(1) + 0.5) * resolution + originY;
  return pt;
  //栅格的索引转为栅格在实际地图小方块的正中央
  //设实际地图的方格4*4大小，那么索引为（1，1），假设分辨率为1，那么转为欧式空间坐标就为（1.5，1.5）
}


/*
* @function     AstarPathFinder::coord2gridIndex
* @brief        将某个欧式空间中的坐标转化为栅格的索引(向无穷小取整)
* @param        Vector2d & pt           对应的欧式空间坐标向量
* @return       Vector2i idx            栅格的索引向量
*/
Vector2i AstarPathFinder::coord2gridIndex(const Vector2d& pt) {
  Vector2i idx;
  idx << min(max(int((pt(0) - originX) * inv_resolution), 0), X_SIZE - 1),
      min(max(int((pt(1) - originY) * inv_resolution), 0), Y_SIZE - 1);

  return idx;
  //x，y分别 << 赋给idx向量，x_size-1为数组的最后一个元素，防止越界，int强制小数取整
}

/*
* @function 	AstarPathFinder::coordRounding
* @brief	    欧式空间坐标 -> 索引值 ->欧式空间坐标，为了显示
* @param	    Vector2d & coord       欧式空间坐标向量  
* @return	    Vector2d coord         欧式空间坐标向量 
*/
Eigen::Vector2d AstarPathFinder::coordRounding(const Eigen::Vector2d& coord) {
  return gridIndex2coord(coord2gridIndex(coord));
}


/*
* @function     AstarPathFinder::isOccupied
* @brief        判断某一栅格是否占用
* @param        const Vector2i &index   要查询的节点索引向量
* @return       bool isOccupied         是否占用
*/
inline bool AstarPathFinder::isOccupied(const Eigen::Vector2i& index) const {
  return isOccupied(index(0), index(1));
}

inline bool AstarPathFinder::isOccupied(const int& idx_x,
                                        const int& idx_y) const {
  return (idx_x >= 0 && idx_x < X_SIZE && idx_y >= 0 && idx_y < Y_SIZE &&
          (data[idx_x + idx_y * X_SIZE] == 100 || data[idx_x + idx_y * X_SIZE] == -1));
}

/*
* @function     AstarPathFinder::isFree
* @brief        判断某一栅格是否可行
* @param        const Vector2i &index   要查询的节点索引向量
* @return       bool isFree             是否空闲
*/
inline bool AstarPathFinder::isFree(const Eigen::Vector2i& index) const {
  return isFree(index(0), index(1));
}

inline bool AstarPathFinder::isFree(const int& idx_x, const int& idx_y) const {
  return (idx_x >= 0 && idx_x < X_SIZE && idx_y >= 0 && idx_y < Y_SIZE &&
          (data[idx_x + idx_y * X_SIZE] == 0));
}


/*
* @function     AstarPathFinder::AstarGetSucc
* @brief        获得当前节点周围的nerigbor,并计算每个nerigbor的fscore
* @param        Node2DPtr currentPtr                             当前节点
                vector<Node2DPtr>& neighborPtrSets               neighbor集合
* @return       vector<double>& edgeCostSets                     neighbor的cost集合
*/
inline void AstarPathFinder::AstarGetSucc(Node2DPtr currentPtr,
                                          vector<Node2DPtr>& neighborPtrSets,
                                          vector<double>& edgeCostSets) {
  neighborPtrSets.clear();//清空neighborPtrSets-> 邻居指针
  edgeCostSets.clear(); //清空 edgeCostSets->double
  Vector2i neighborIdx;//创一个邻居索引
  for (int dx = -1; dx < 2; dx++) {//遍历除自身的8个点
    for (int dy = -1; dy < 2; dy++) {
      if (dx == 0 && dy == 0) //如果遍历到自己，就继续
      	continue;
      neighborIdx(0) = (currentPtr->index)(0) + dx;
      neighborIdx(1) = (currentPtr->index)(1) + dy;
      if (neighborIdx(0) < 0 || neighborIdx(0) >= X_SIZE ||
          neighborIdx(1) < 0 || neighborIdx(1) >= Y_SIZE) {//判断当前nerigbor是否处在data内，即:(地图内)
        continue;
      }
      neighborPtrSets.push_back(GridNodeMap[neighborIdx(0)][neighborIdx(1)]);//map上的位置
      edgeCostSets.push_back(sqrt(dx * dx + dy * dy));//该点离cur点的的欧式距离代价
    }
  }
}


/*
* @function     AstarPathFinder::getHeu
* @brief        计算某个节点的启发函数值h(n)
* @param        Node2DPtr node1       节点1
                Node2DPtr node2       目标节点
* @return       double score            h(n)
*/
double AstarPathFinder::getHeu(Node2DPtr node1, Node2DPtr node2) {
  // choose possible heuristic function you want
  // Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
  double dx = abs(node1->index(0) - node2->index(0));
  double dy = abs(node1->index(1) - node2->index(1));
  double tie_breaker = 1 + 1 / 10000;

  double h = 0.0;
  int diag = min(dx, dy);
  dx -= diag;
  dy -= diag;

  if (dx == 0) {
    h = sqrt(2.0) * diag + 1.0 * abs(dy);
  }
  if (dy == 0) {
    h = sqrt(2.0) * diag + 1.0 * abs(dx);
  }

  return tie_breaker * h;
  //return 1.0*(dx+dy)+(sqrt(2.0)-2*1.0)*min(dx,dy);
}


/*
* @function     AstarPathFinder::AstarGraphSearch
* @brief        A*算法主流程
* @param        Vector2d start_pt       起点
                Vector2d end_pt         终点
* @return       None
*/
bool AstarPathFinder::AstarGraphSearch_Collision() {
  // ROS_INFO("searching");
  //ROS_INFO("start:[%f,%f]", start_pt(0), start_pt(1));
  //ROS_INFO("goal:[%f,%f]", goal_pt(0), goal_pt(1));

  ros::Time time_1 = ros::Time::now();

  //起点和终点的索引向量
  Vector2i start_idx = coord2gridIndex(start_pt);
  Vector2i end_idx = coord2gridIndex(goal_pt);
  //ROS_INFO("start id:[%d,%d]", start_idx(0), start_idx(1));
  //ROS_INFO("goal id:[%d,%d]", end_idx(0), end_idx(1));
  goalIdx = end_idx;
  if(isOccupied(goalIdx))//是否是障碍物点
  {
     ROS_WARN("The goal is Occupied, Please choose a new goal");//若设置的终点为障碍点（不可行点），则系统提示目标被占用，请重新选择终点
     return false;
  }
  //起点和终点的空间位置向量
  start_pt = gridIndex2coord(start_idx);
  goal_pt = gridIndex2coord(end_idx);

  //初始化起点和终点的节点(指针)
  Node2DPtr startPtr = new Node2D(start_idx, start_pt);
  // 用于记录上一个用于计算yaw的点位
  node2com = startPtr;//初值赋起始点
  turn=0;//复位
  Node2DPtr endPtr = new Node2D(end_idx, goal_pt);
  
  openSet.clear();//使用STL库中的multimap维护openset (std::multimap<double, GridNodePtr> openSet)
  //初始化currentPtr与, neighborPtr  (currentPtr代表openset中f(n)最小的节点)
  Node2DPtr currentPtr = NULL;
  Node2DPtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0; //起点g(n)设为0
  startPtr->fScore = getHeu(startPtr, endPtr)+startPtr->gScore;//起点的f(n)，启发式加上g，但是，此时g=0
  startPtr->id = 1;  // id is a flag, in open list or not
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));//将起点加入openset中，openset存储着对应节点的f(n) , make_pair实际上是一个创建二元组的便利函数模板

  double tentative_gScore;
  vector<Node2DPtr> neighborPtrSets;
  vector<double> edgeCostSets;

  // this is the main loop
  while (!openSet.empty()) {

    /*std::multimap<double, Node2DPtr>::iterator itFoundmin;
    for (itFoundmin = openSet.begin(); itFoundmin != openSet.end(); itFoundmin++)
    {
        if (itFoundmin->second->id == 1)    //说明该节点没被访问过
        {
                currentPtr = itFoundmin->second;
                currentPtr->id = -1;    //标记当前节点为已访问状态
                GridNodeMap[currentPtr->index(0)][currentPtr->index(1)]->id = -1;
                break;
        }
    }*/
    currentPtr = openSet.begin()->second;
    openSet.erase(openSet.begin());
    currentPtr->id = -1;
    //找到Goal,返回
    if (currentPtr->index == goalIdx) {
      ros::Time time_2 = ros::Time::now();
      terminatePtr = currentPtr;
      total_length = currentPtr->gScore * resolution; //m  A*路线长度
      // ROS_INFO("[A*]{sucess} Time in A* is %f s, path cost is %f m",
      //          (time_2 - time_1).toSec(),
      //          total_length);
      return true;
    }
   
    //expandedNodes.push_back(currentPtr);
    // get the successors 获得当前节点周围的nerigbor,并计算每个nerigbor的fscore
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

    for (int i = 0; i < (int)neighborPtrSets.size(); i++) {
      /*
      IMPORTANT NOTE!!!
      neighborPtrSets[i]->id = -1 : unexpanded
      neighborPtrSets[i]->id = 1 : expanded, equal to this node is in close set
      *
      */
      neighborPtr = neighborPtrSets[i];
      // neighborPtr根据上面的函数，始终在地图内，也就是这些节点肯定会在地图范围内

      // 碰撞检测版本
      if (!validityCheck(neighborPtr, currentPtr, i, (int)neighborPtrSets.size()) || neighborPtr->id == -1) 
      // 膨胀版本
      // if (isOccupied(neighborPtr->index) || neighborPtr->id == -1) 
      	continue;//被占用或者是被访问过就继续

      double edge_cost = edgeCostSets[i];
      tentative_gScore = currentPtr->gScore + edge_cost;//更新g值

      if (neighborPtr->id == 0) {  //发现新节点，即不是1，也不是-1

        neighborPtr->id = 1;//说明该节点没被访问过
        neighborPtr->predecessor = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr)); // put neighbor in open set and record it.
        continue;
      } else if (tentative_gScore <= neighborPtr->gScore) { // in open set and need update

        neighborPtr->predecessor = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
        openSet.erase(neighborPtr->nodeMapIt);
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr)); // put neighbor in open set and record it.
        continue;
      }
    }
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_INFO("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  return false;
}


bool AstarPathFinder::AstarGraphSearch() {
  // ROS_INFO("searching");
  //ROS_INFO("start:[%f,%f]", start_pt(0), start_pt(1));
  //ROS_INFO("goal:[%f,%f]", goal_pt(0), goal_pt(1));

  ros::Time time_1 = ros::Time::now();

  //起点和终点的索引向量
  Vector2i start_idx = coord2gridIndex(start_pt);
  Vector2i end_idx = coord2gridIndex(goal_pt);
  //ROS_INFO("start id:[%d,%d]", start_idx(0), start_idx(1));
  //ROS_INFO("goal id:[%d,%d]", end_idx(0), end_idx(1));
  goalIdx = end_idx;
  if(isOccupied(goalIdx))//是否是障碍物点
  {
     ROS_WARN("The goal is Occupied, Please choose a new goal");//若设置的终点为障碍点（不可行点），则系统提示目标被占用，请重新选择终点
     return false;
  }
  //起点和终点的空间位置向量
  start_pt = gridIndex2coord(start_idx);
  goal_pt = gridIndex2coord(end_idx);

  //初始化起点和终点的节点(指针)
  Node2DPtr startPtr = new Node2D(start_idx, start_pt);
  // 用于记录上一个用于计算yaw的点位
  node2com = startPtr;//初值赋起始点
  turn=0;//复位
  Node2DPtr endPtr = new Node2D(end_idx, goal_pt);
  
  openSet.clear();//使用STL库中的multimap维护openset (std::multimap<double, GridNodePtr> openSet)
  //初始化currentPtr与, neighborPtr  (currentPtr代表openset中f(n)最小的节点)
  Node2DPtr currentPtr = NULL;
  Node2DPtr neighborPtr = NULL;

  // put start node in open set
  startPtr->gScore = 0; //起点g(n)设为0
  startPtr->fScore = getHeu(startPtr, endPtr)+startPtr->gScore;//起点的f(n)，启发式加上g，但是，此时g=0
  startPtr->id = 1;  // id is a flag, in open list or not
  startPtr->coord = start_pt;
  openSet.insert(make_pair(startPtr->fScore, startPtr));//将起点加入openset中，openset存储着对应节点的f(n) , make_pair实际上是一个创建二元组的便利函数模板

  double tentative_gScore;
  vector<Node2DPtr> neighborPtrSets;
  vector<double> edgeCostSets;

  // this is the main loop
  while (!openSet.empty()) {

    /*std::multimap<double, Node2DPtr>::iterator itFoundmin;
    for (itFoundmin = openSet.begin(); itFoundmin != openSet.end(); itFoundmin++)
    {
        if (itFoundmin->second->id == 1)    //说明该节点没被访问过
        {
                currentPtr = itFoundmin->second;
                currentPtr->id = -1;    //标记当前节点为已访问状态
                GridNodeMap[currentPtr->index(0)][currentPtr->index(1)]->id = -1;
                break;
        }
    }*/
    currentPtr = openSet.begin()->second;
    openSet.erase(openSet.begin());
    currentPtr->id = -1;
    //找到Goal,返回
    if (currentPtr->index == goalIdx) {
      ros::Time time_2 = ros::Time::now();
      terminatePtr = currentPtr;
      total_length = currentPtr->gScore * resolution; //m  A*路线长度
      // ROS_INFO("[A*]{sucess} Time in A* is %f s, path cost is %f m",
      //          (time_2 - time_1).toSec(),
      //          total_length);
      return true;
    }
   
    //expandedNodes.push_back(currentPtr);
    // get the successors 获得当前节点周围的nerigbor,并计算每个nerigbor的fscore
    AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);

    for (int i = 0; i < (int)neighborPtrSets.size(); i++) {
      /*
      IMPORTANT NOTE!!!
      neighborPtrSets[i]->id = -1 : unexpanded
      neighborPtrSets[i]->id = 1 : expanded, equal to this node is in close set
      *
      */
      neighborPtr = neighborPtrSets[i];
      // neighborPtr根据上面的函数，始终在地图内，也就是这些节点肯定会在地图范围内

      // 碰撞检测版本
      // if (!validityCheck(neighborPtr, currentPtr, i, (int)neighborPtrSets.size()) || neighborPtr->id == -1) 
      // 膨胀版本
      if (isOccupied(neighborPtr->index) || neighborPtr->id == -1) 
      	continue;//被占用或者是被访问过就继续

      double edge_cost = edgeCostSets[i];
      tentative_gScore = currentPtr->gScore + edge_cost;//更新g值

      if (neighborPtr->id == 0) {  //发现新节点，即不是1，也不是-1

        neighborPtr->id = 1;//说明该节点没被访问过
        neighborPtr->predecessor = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = neighborPtr->gScore + getHeu(neighborPtr, endPtr);
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr)); // put neighbor in open set and record it.
        continue;
      } else if (tentative_gScore <= neighborPtr->gScore) { // in open set and need update

        neighborPtr->predecessor = currentPtr;
        neighborPtr->gScore = tentative_gScore;
        neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
        openSet.erase(neighborPtr->nodeMapIt);
        neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore,neighborPtr)); // put neighbor in open set and record it.
        continue;
      }
    }
  }

  // if search fails
  ros::Time time_2 = ros::Time::now();
  if ((time_2 - time_1).toSec() > 0.1)
    ROS_INFO("Time consume in Astar path finding is %f",
             (time_2 - time_1).toSec());
  return false;
}


//回溯得到path
vector<Vector2d> AstarPathFinder::getPath() {
  vector<Vector2d> path;
  vector<Node2DPtr> gridPath;
  // trace back from the curretnt nodePtr to get all nodes along the path
  gridPath.push_back(terminatePtr);//首先拿到目标点
  auto currentPtr = terminatePtr;
  while (currentPtr->predecessor != NULL) {
    currentPtr = currentPtr->predecessor;
    gridPath.push_back(currentPtr);//取每个节点的父节点
  }

  for (auto ptr : gridPath) path.push_back(ptr->coord);//取节点的坐标属性

  reverse(path.begin(), path.end());//翻转路径

  return path;
}


//接收到起点（小车位姿）后的回调函数
void AstarPathFinder::setStartCallback(
    const geometry_msgs::Pose::ConstPtr& begin) {
  /*ROS_INFO("enter set start callback function.");
  ROS_INFO("------------------------------");
  ROS_INFO("received a start pose from the Rviz.");*/
  start_cur.position = begin->position;
  //double y = begin->position.y;
  //start_cur.orientation = tf::getYaw(begin->orientation);
  // publish the start without covariance for rviz
  //cout<<"start x:"<<x<<"start y:"<<y<<endl;
  /*geometry_msgs::PoseStamped startRviz;
  startRviz.pose.position = begin->pose.pose.position;
  startRviz.pose.orientation = begin->pose.pose.orientation;
  startRviz.header.frame_id = "/map";
  startRviz.header.stamp = ros::Time::now();*/
  validStart = true;
  // publish start for RViz
  //start_pub.publish(startRviz);
  start = *begin;
  //ROS_INFO("[node] receive the planning start");

  makePlan();
}


//获取目标点后的回调函数
void AstarPathFinder::setGoalCallback(
    const geometry_msgs::PoseStamped::ConstPtr& end) {
  /*ROS_INFO("enter set goal callback function.");
  ROS_INFO("------------------------------");
  ROS_INFO("received a goa pose from the Rviz.");*/
  // double x = initial->pose.pose.position.x;
  // double y = initial->pose.pose.position.y;
  // double yaw = tf::getYaw(initial->pose.pose.orientation);
  // publish the start without covariance for rviz

  /*geometry_msgs::PoseStamped goalRviz;
  goalRviz.pose.position = end->pose.position;
  goalRviz.pose.orientation = end->pose.orientation;
  goalRviz.header.frame_id = "/map";
  goalRviz.header.stamp = ros::Time::now();*/
  targetPoint_cur.pose.orientation = end->pose.orientation;
  targetPoint_cur.pose.position = end->pose.position;
  validGoal = true;
  goal = *end;
  //goal_pub.publish(goalRviz);

  //ROS_INFO("[node] receive the planning target");
  makePlan();
}


//规划函数
void AstarPathFinder::makePlan() {
    //cout<<validGoal<<validStart<<validMap<<endl;
    // ROS_INFO("enter make plan function");

    //终点
    double x = goal.pose.position.x;
    double y = goal.pose.position.y;
    goal_pt(0) = x;
    goal_pt(1) = y;

    // _________________________
    // retrieving start position
    //起点
    float x1 = start.position.x;
    float y1 = start.position.y;
    start_pt(0) = x1;
    start_pt(1) = y1;
    // ROS_INFO("start:[%f,%f]", start_pt(0), start_pt(1));
    // ROS_INFO("goal:[%f,%f]", goal_pt(0), goal_pt(1));
    // Call A* to search for a path 利用A*找路径
    if(validMap && validStart && validGoal){//地图、起终点已经就绪
      //A*搜路  
      if(AstarGraphSearch()){

        // Retrieve the path
        gridPath_ = getPath();//回溯得到path
        //visitedNotes_ = getVisitedNodes();

        // Visualize the result
        //visVisitedNode();
        visGridPath();//path在rviz中显示
        //visGridSmoothPath();
        publocal();//发布local_goal供DWA用
        
        // Reset map for next call
        resetUsedGrids();//重置整张栅格地图
        deleteVariablesData();//清除、重置
      }

	
   }
   else
   {    
   	  ROS_WARN("validGoal: validStart: validMap: %d,%d,%d",validGoal,validStart,validMap);

   }
}


auto AstarPathFinder::makePlanandBezierOpt_Collision(Eigen::Vector3d start_pos, Eigen::Vector3d target_pos) -> nav_msgs::Path {
    //终点
    goal_pt(0) = target_pos[0];
    goal_pt(1) = target_pos[1];
    //起点
    start_pt(0) = start_pos[0];
    start_pt(1) = start_pos[1];
    // std::vector<Eigen::Vector2d> path={};
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.clear();
    // Call A* to search for a path 利用A*找路径
    //确保地图、起终点已经就绪
    while(!validMap);
    // {
      // ROS_WARN("validMap: %d",validMap);//0
    // }
    
    // if(validMap){

      //A*搜路
      if(!AstarGraphSearch_Collision()) return path;//空的path

      // Retrieve the path
      gridPath_ = getPath();//回溯得到path

      nav_msgs::Path path2opt;
      path2opt.header.stamp = ros::Time::now();
      path2opt.header.frame_id = "map";
      geometry_msgs::PoseStamped pathVehicle;
      pathVehicle.header.stamp = ros::Time::now();
      for (auto i = 0; i < gridPath_.size(); i++) {
        pathVehicle.header.frame_id = "map";
        //pathVehicle.id = id;
        pathVehicle.pose.position.x = gridPath_[i].x();
        pathVehicle.pose.position.y = gridPath_[i].y();
        pathVehicle.pose.position.z = 0;
        path2opt.poses.push_back(pathVehicle);
      }
        //试试先优化路径再等效
        //不先插值贝塞尔优化没有效果
      nav_msgs::Path path2inter;
      //栅格最大间距0.1414m
      // BezierOpt.deal_path(path2opt, path2inter, 5);//每x个点取1个
      BezierOpt.deal_path(path2opt, path, 5);//每x个点取1个

      // Reset map for next call
      resetUsedGrids();//重置整张栅格地图
      deleteVariablesData();//清除、重置
      validMap = false;

   return path;
}


//A*搜路+贝塞尔优化
auto AstarPathFinder::makePlanandBezierOpt(Eigen::Vector3d start_pos, Eigen::Vector3d target_pos) -> nav_msgs::Path {
    //终点
    goal_pt(0) = target_pos[0];
    goal_pt(1) = target_pos[1];
    //起点
    start_pt(0) = start_pos[0];
    start_pt(1) = start_pos[1];
    // std::vector<Eigen::Vector2d> path={};
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.poses.clear();
    // Call A* to search for a path 利用A*找路径
    //确保地图、起终点已经就绪
    while(!validMap);
    // {
      // ROS_WARN("validMap: %d",validMap);//0
    // }
    
    // if(validMap){

      //A*搜路
      if(!AstarGraphSearch()) return path;//空的path

      // Retrieve the path
      gridPath_ = getPath();//回溯得到path

      nav_msgs::Path path2opt;
      path2opt.header.stamp = ros::Time::now();
      path2opt.header.frame_id = "map";
      geometry_msgs::PoseStamped pathVehicle;
      pathVehicle.header.stamp = ros::Time::now();
      for (auto i = 0; i < gridPath_.size(); i++) {
        pathVehicle.header.frame_id = "map";
        //pathVehicle.id = id;
        pathVehicle.pose.position.x = gridPath_[i].x();
        pathVehicle.pose.position.y = gridPath_[i].y();
        pathVehicle.pose.position.z = 0;
        path2opt.poses.push_back(pathVehicle);
      }
        //试试先优化路径再等效
        //不先插值贝塞尔优化没有效果
      nav_msgs::Path path2inter;
      //栅格最大间距0.1414m
      // BezierOpt.deal_path(path2opt, path2inter, 5);//每x个点取1个
      BezierOpt.deal_path(path2opt, path, 5);//每x个点取1个

      //visitedNotes_ = getVisitedNodes(); //显示访问过的栅格
      // Visualize the result
      //visVisitedNode();

      // 贝塞尔优化（不需要，只要在外部用B-样条拟合）
      // nav_msgs::Path pathopted;
      // BezierOpt.createCurve(path2inter, pathopted);
      //再插值
      //总长度/step=所需路点个数
      //总路点个数/所需路点个数=dis
    // const double v_max = 0.3;//平均车速
    // const double step_time = 0.25;//时间间隔
    // const double step = v_max*step_time;   // = v_max * Ts
    // //total_length取的还是原A*长度,如果能用贝塞尔曲线实际长度代替应该更精确
    // //这里的total_length应该偏小
    // auto num = (double)total_length/step;
    //   // int dis = std::round(pathopted.poses.size()/num);
    //   // int dis = (int)pathopted.poses.size()/num;
    //   //由于num偏小,dis偏大,还是向下取整为妙
    //   // int dis = floor(pathopted.poses.size()/num);
    //   int dis = floor(path2opt.poses.size()/num);//直接对原始A*采样，初筛，减小B-样条拟合计算量
    //   // ROS_INFO("dis= %d" ,dis); //
    //   if(dis<1) dis=1;
    //   // BezierOpt.deal_path(pathopted, path, dis);//每dis个点取1个
    //   BezierOpt.deal_path(path2opt, path, dis);//每dis个点取1个
      // for(const auto& pose: path2inter.poses)
      // {
      //   Eigen::Vector2d pos;
      //   pos << pose.pose.position.x,pose.pose.position.y;
      //   path.push_back(pos);
      // }

      //visGridSmoothPath();//三次样条插值效果不如贝塞尔
      // publocal();//发布local_goal供DWA用
        
      // Reset map for next call
      resetUsedGrids();//重置整张栅格地图
      deleteVariablesData();//清除、重置
      validMap = false;
  //  }
  //  else //地图没准备好
  //  {    
  //  	  ROS_ERROR("validMap: %d",validMap);//0
  //  }
   
   return path;
}


//path在rviz中显示
void AstarPathFinder::visGridPath() {
   // path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  path.poses.clear();
  geometry_msgs::PoseStamped pathVehicle;
  int id = 0;
  for (auto i = 0; i < gridPath_.size(); i++) {
    pathVehicle.header.frame_id = "map";
    pathVehicle.header.stamp = ros::Time(0);
    //pathVehicle.id = id;
    pathVehicle.pose.position.x = gridPath_[i](0);
    pathVehicle.pose.position.y = gridPath_[i](1);
    pathVehicle.pose.position.z = 0;
    path.poses.push_back(pathVehicle);
  }

  path_pub.publish(path);
  //ROS_INFO("pub path in rviz");
  //ROS_INFO("path note number, %d", path.poses.size());
}


/*void AstarPathFinder::visGridSmoothPath(){
  nav_msgs::Path smoothedPath,pathcur;
  pathcur.header.frame_id = "map";
  geometry_msgs::PoseStamped posecur;
  posecur.header.frame_id = "map";

  pathcur.poses.clear();
  float yaw;
  geometry_msgs::Quaternion orientation_target;	
  int pointsPerUnit = 5;
  int skipPoints = 1;
  bool useEndConditions = false;
  bool useMiddleConditions = false;
  
  
  for (auto i = 0; i < gridPath_.size()-1; i++)
  {
  	float dx = gridPath_[i+5](0)-gridPath_[i](0);
    float dy = gridPath_[i+5](1)-gridPath_[i](1);
    yaw = atan2(dy, dx);
	  tf::Quaternion q;
	  q.setRPY(0, 0, yaw);
	  q.normalize();
	  orientation_target.x = q[0];
      orientation_target.y = q[1];
	  orientation_target.z = q[2];
	  orientation_target.w = q[3];
	  //orientation_target.normalize();
    //Vector2i curpose = coord2gridIndex(gridPath_[i]);
    //Vector2d curpose1 = gridIndex2coord(curpose);
	
       posecur.pose.position.x = gridPath_[i](0);//static_cast<double>(poseList[i]["x"])curpose1(0)
       posecur.pose.position.y = gridPath_[i](1);//static_cast<double>(poseList[i]["y"])curpose1(1)
       posecur.pose.orientation = orientation_target;/*tf::createQuaternionMsgFromYaw(poseList[i]["yaw"])//orientation_target
   
    	/*posecur.pose.position.x = gridPath_[i](0);
    	posecur.pose.position.y = gridPath_[i](1);
    	posecur.pose.orientation = targetPoint_cur.pose.orientation;
    */
   /* if(i==gridPath_.size()){
    	posecur.pose.position.x = targetPoint_cur.pose.position.x;
        posecur.pose.position.y = targetPoint_cur.pose.position.y;
        posecur.pose.orientation = targetPoint_cur.pose.orientation;
   }*/
/*    posecur.pose.position = targetPoint_cur.pose.position;
    posecur.pose.orientation = targetPoint_cur.pose.orientation;
  }
    pathcur.poses.push_back(posecur);
   */
 
  // create a cubic spline interpolator  创建一个三次样条插值器
/*  CubicSplineInterpolator csi("wenhao");
    // pointsPerUnit, skipPoints, useEndConditions, useMiddleConditions);
  csi.interpolatePath(pathcur, smoothedPath);
  smoothedPath_pub.publish(smoothedPath);
  //cout<<"success++++++++++++++++++++++"<<endl;
}*/


//发布local_goal
void AstarPathFinder::publocal(){
  float x;
  float y;
  int out_range_num = 15;
  float yaw;
  geometry_msgs::Quaternion orientation_target;
  if(gridPath_.size()>out_range_num+3){
    x = gridPath_[out_range_num](0);
    y = gridPath_[out_range_num](1); 
    float dx = gridPath_[out_range_num+3](0)-gridPath_[out_range_num](0);
    float dy = gridPath_[out_range_num+3](1)-gridPath_[out_range_num](1);
    yaw = atan2(dy, dx);
	  tf::Quaternion q;
	  q.setRPY(0, 0, yaw);
	  q.normalize();
	  orientation_target.x = q[0];
    orientation_target.y = q[1];
	  orientation_target.z = q[2];
	  orientation_target.w = q[3];
  }
  else if(gridPath_.size()<out_range_num+3){
    x = gridPath_.back()(0);
    y = gridPath_.back()(1);
    orientation_target = targetPoint_cur.pose.orientation;
  }
  if(hypot(start_cur.position.x-targetPoint_cur.pose.position.x,start_cur.position.y-targetPoint_cur.pose.position.y)>0.5)
  {
        geometry_msgs::PoseStamped local_goal_msg;
        local_goal_msg.header.seq = 0;
        local_goal_msg.header.stamp = ros::Time::now();//如果有问题就使用Time(0)获取时间戳，确保类型一致
        local_goal_msg.header.frame_id = "map";
        local_goal_msg.pose.position.x = x;
        local_goal_msg.pose.position.y = y;
        local_goal_msg.pose.position.z = 0.0;

        local_goal_msg.pose.orientation = orientation_target;
        // 发布消息
        local_goal_pub.publish(local_goal_msg);
  }
  else
  {
      geometry_msgs::PoseStamped local_goal_msg;
      local_goal_msg.header.seq = 0;
      local_goal_msg.header.stamp = ros::Time::now();//如果有问题就使用Time(0)获取时间戳，确保类型一致
      local_goal_msg.header.frame_id = "map";
      local_goal_msg.pose.position = targetPoint_cur.pose.position;

      local_goal_msg.pose.orientation = targetPoint_cur.pose.orientation;
      // 发布消息
      local_goal_pub.publish(local_goal_msg);
  }
}


/*void AstarPathFinder::visVisitedNode() {
  visualization_msgs::Marker pathVehicle;
  int id = 0;
  for (auto i = 0; i < visitedNotes_.size(); i++) {
    pathVehicle.header.frame_id = "/map";
    pathVehicle.header.stamp = ros::Time(0);
    pathVehicle.id = id;
    pathVehicle.type = visualization_msgs::Marker::CUBE;

    pathVehicle.scale.x = resolution;
    pathVehicle.scale.y = resolution;
    pathVehicle.scale.z = resolution;
    pathVehicle.color.a = 0.5;
    pathVehicle.color.r = 0.0 / 255.0;
    pathVehicle.color.g = 255.0 / 255.0;
    pathVehicle.color.b = 2550.0 / 255.0;

    pathVehicle.pose.position.x = visitedNotes_[i](0);
    pathVehicle.pose.position.y = visitedNotes_[i](1);
    pathVehicle.pose.position.z = 0;
    visited_notes_in_rviz_.markers.push_back(pathVehicle);
    id++;
  }
  _visited_nodes_vis_pub.publish(visited_notes_in_rviz_);
  ROS_INFO("pub visted note in rviz");
  ROS_INFO("visited note number, %d", visited_notes_in_rviz_.markers.size());
  // grids.markers.clear();
}
*/


//清除重置
void AstarPathFinder::deleteVariablesData() {
  //path.markers.clear();
  //visited_notes_in_rviz_.markers.clear();
  gridPath_.clear();
  //visitedNotes_.clear();
  // openSet is the open_list implemented through multimap in STL library
  openSet.clear();
  terminatePtr = NULL;
}


// 膨胀栅格:行数,总栅格数,a为膨胀前/后,膨胀格数,s为标记膨胀的栅格
void AstarPathFinder::find_gezi(const int hang,const int counts,int8_t* a,int floor,set<int>& s){
// void AstarPathFinder::find_gezi(const int hang, const int counts, std::vector<int8_t> a, int floor, set<int>& s){
  if (floor > 0 )
  {
    int f[8]={ };
    for (int i = 0; i < counts; ++i)
    {
      //代价值为1（障碍物占用）
      if (a[i]==100)
      {
        //标记需要膨胀的栅格
        f[0]=i-hang-1;
        f[3]=i-1;
        f[6]=i+hang-1;
        f[1]=i-hang;
        f[7]=i+hang;
        f[2]=i-hang+1;
        f[5]=i+1;
        f[8]=i+hang+1;
        // f[4]=i;
      }
      for (int j = 0; j < 8; j++ )
      { 
        //将所有要膨胀的栅格赋给s
        s.insert(f[j]);
      }
    }
    //给标记为膨胀的栅格膨胀，赋100，为障碍物
    for( auto beg=s.begin(),end=s.end(); beg!=end ; ++beg)
    {
      if (-1 < *beg && *beg < counts)
        a[*beg]=100;
    }
  }else {
    return;
  }
  find_gezi(hang,counts,a,floor-1,s);//递归，直到floor=0结束膨胀
}


// 碰撞检测
// 当前节点的邻居节点之一，当前节点，当前邻居节点序列号，邻居节点数量
bool AstarPathFinder::validityCheck(Node2DPtr nenode, Node2DPtr curnode, int j, int num){
    //每个节点都要是否占用检测
    //被占用的节点抛弃
    if (isOccupied(nenode->index)) {
        return false;//被占用，不合法
    }

    // 每间隔x个节点进行碰撞检测，减少计算量-------这个怎么实现？看currentPtr
    // 角度不使用邻近两个节点计算的朝向，使用上一次记录的节点和当前节点计算的朝向
    if(node2com != curnode){
      turn++;
    } 
    if(turn > 1){
      //栅格索引->真实索引
      Eigen::Vector3d pose;//当前邻居节点位姿
      Eigen::Vector2d pose2com;//历史节点位姿
      pose2com(0) = (node2com->index(0)+0.5) * resolution + originX;
      pose2com(1) = (node2com->index(1)+0.5) * resolution + originY;  
      
      pose(0) = (nenode->index(0)+0.5) * resolution + originX;
      pose(1) = (nenode->index(1)+0.5) * resolution + originY;    
      pose(2) = atan2(pose(1) - pose2com(1),
                      pose(0) - pose2com(0));//求夹角
      //首先需要当前节点的朝向:由当前点位和上一次点位得到yaw
      // std::pair<int, int> key = {nenode->index(1) - cunode->index(1), nenode->index(0) - cunode->index(0)};
      // 查找并获取值
      // auto iter = directionAngles.find(key);
      // if(iter != directionAngles.end()) {
      //     yaw = iter->second;
      // }
      // else {
      //   ROS_INFO("Yaw Invalid!");
      //   return false;
      // }

      // 最后一个当前节点的邻居节点才能复位
      if(j == num-1){
          turn=0;
          node2com = curnode;//赋上这次的节点
      }

      //生成车的四个顶点
      auto vertices = calculateCarBoundingBox(pose);
      //遍历四条边是否碰撞
      for (int i = 0; i < vertices.size(); ++i) {
          if (isLinecollision(vertices[i].x(), vertices[i].y(), 
                              vertices[(i+1) % 4].x(), vertices[(i+1) % 4].y())){
              return false;  //会碰撞，非法                  
          }
      }
    }
    return true;
}

//在当前路点生成车的四个顶点
vector<Eigen::Vector2d> AstarPathFinder::calculateCarBoundingBox(Eigen::Vector3d pose) {
    vector<Eigen::Vector2d> vertices;//由珊格索引表示的车轮廓
    double shift_distance = length_ / 2 - back_edge_to_center_;//后轴中心对车中心的偏移
    //栅格索引->真实坐标
    // Eigen::Vector3d pose;
    // pose(0) = (index(0)+0.5) * resolution + originX;
    // pose(1) = (index(1)+0.5) * resolution + originY;    
    // pose(2) = yaw;
    //小车中心
    Vector2d center = {pose(0) + shift_distance * std::cos(pose(2)),//pose位姿点在后轴中心
                    pose(1) + shift_distance * std::sin(pose(2))};
    //设置一个膨胀值inflation_value，放大轮廓（保守）
    const double dx1 = std::cos(pose(2)) * (length_ / 2 + inflation_value );
    const double dy1 = std::sin(pose(2)) * (length_ / 2 + inflation_value);
    const double dx2 = std::sin(pose(2)) * (width_ / 2 + inflation_value);
    const double dy2 = -std::cos(pose(2)) * (width_ / 2 + inflation_value);

    // const double dx1 = std::cos(pose(2)) * length_ / 2 ;
    // const double dy1 = std::sin(pose(2)) * length_ / 2 ;
    // const double dx2 = std::sin(pose(2)) * width_ / 2 ;
    // const double dy2 = -std::cos(pose(2)) * width_ / 2 ;

    //四个顶点
    vertices.emplace_back(center(0) + dx1 + dx2, center(1) + dy1 + dy2);
    vertices.emplace_back(center(0) + dx1 - dx2, center(1) + dy1 - dy2);
    vertices.emplace_back(center(0) - dx1 - dx2, center(1) - dy1 - dy2);
    vertices.emplace_back(center(0) - dx1 + dx2, center(1) - dy1 + dy2); //顺时针
    return vertices;
}

/* 查询一条线上的点是否在障碍物里（插值）*/
bool AstarPathFinder::isLinecollision(double x0, double y0, double x1, double y1) {
    // int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / xy_resolution_) + 1;
    // int check_point_num = static_cast<int>(max(abs(x1 - x0),abs(y1 - y0)) / 1) + 1;
    // const double resolution2 = 2*resolution;
    //间隔取点，减少计算量
    auto check_point_num = static_cast<int>(std::hypot(x1 - x0, y1 - y0) / resolution);

    const double delta_x = (x1 - x0) / check_point_num;
    const double delta_y = (y1 - y0) / check_point_num;

    double cur_x = x0;
    double cur_y = y0;

    // 判断是否是障碍物和超出边界
    // auto idx = i + j * map_.info.width;：这一行计算出给定坐标在地图 data 数组中的下标 idx。由于地图数据存储方式通常是一维数组，所以需要将二维坐标转换为一维下标。具体计算方法是将 j 乘以地图宽度，再加上 i。
    // return map_.data[idx] == 100;：最后，函数返回一个布尔值，表示给定坐标是否为障碍物。如果地图 data 数组中 idx 对应的值为 100，就认为该坐标是障碍物，返回 true；否则返回 false。
    auto mapObstacle = [&](double x, double y) {
        int i = std::floor((x - originX) / resolution + 0.5);
        int j = std::floor((y - originY) / resolution + 0.5);
        int idx = i + j * X_SIZE;
        // 目标：超出范围或是障碍物返回1
        return (i < 0 || i >= X_SIZE || j < 0 && j >= Y_SIZE ||
                data[idx] == 100 || data[idx] == -1 );
    };

    for (int i = 0; i < check_point_num; ++i) {
        //   if (!isInMap(cur_x, cur_y)) {
        // return true;
        // }
        //障碍区和未知区都不能通过，也不能超出地图范围（现在的车轮廓线是可能超出地图范围的）
        if(mapObstacle(cur_x, cur_y))  return true;//是障碍物

        cur_x += delta_x;
        cur_y += delta_y;
    }
    return false;//不是障碍物
}




