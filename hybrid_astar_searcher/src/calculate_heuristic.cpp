#include "hybrid_astar_searcher/calculate_heuristic.h"

using namespace std;

namespace planning {

// GridSearch::GridSearch(grid_map::GridMap map) {
//     gmap_ = map;
//     resolution_ = map.getResolution();
//     x_min_ = map.getStartIndex()(0) * resolution_ - 25;
//     x_max_ = x_min_ + map.getLength().x();
//     y_min_ = map.getStartIndex()(1) * resolution_ -25;
//     y_max_ = y_min_ + map.getLength().y();
    
//     map_size_x_ = map.getSize()(0); 
//     map_size_y_ = map.getSize()(1);
    
//     cout << "x_min and x_max:" << x_min_ << " " << x_max_ << endl;
//     cout << "y_min and y_max:" << y_min_ << " " << y_max_ << endl;

//     cout << "map_size_x_" << map_size_x_ << endl;
//     cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl;


// }


// 接受一个 CollisionDetection 对象作为参数
GridSearch::GridSearch(const nav_msgs::OccupancyGridConstPtr& map, CollisionDetection& configurationSpace) :
    map_(map),
    cloud(new pcl::PointCloud<pcl::PointXY>){
    // map_ = map;
    resolution_ = map->info.resolution;
    x_min_ = map->info.origin.position.x;//地图原点(第一个栅格点)在frame_id坐标系下("map")的值
    x_max_ = x_min_ + map->info.width*resolution_;//grid.info.width:宽度:在x方向的栅格数
    y_min_ = map->info.origin.position.y;
    y_max_ = y_min_ + map->info.height*resolution_;
    map_size_x_ = map->info.width; 
    map_size_y_ = map->info.height;
    
    configurationSpace_ptr_ = std::make_unique<CollisionDetection>(configurationSpace);

    //可视化优化前的曲线路点
    ros::NodeHandle nh("/");
    path_point_vis_pub = nh.advertise<visualization_msgs::Marker>("db_path_point_marker", 10);
    // cout << "x_min and x_max:" << x_min_ << " " << x_max_ << endl;
    // cout << "y_min and y_max:" << y_min_ << " " << y_max_ << endl;

    // cout << "map_size_x_" << map_size_x_ << endl;
    // cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl;
}


/**
 * @brief 通过A星的距离作为启发式函数，一般不用，因为每次扩展节点都用一次A星累积起来太多了
 * 修改：改成只在起终点调用一次，而后提取折线点，将A*路径转化为若干由折点表示的path
 * 
 */
std::vector<Point2D> GridSearch::calculateHeuByAstar(Vec3d& start_pos, Vec3d& goal_pos) {
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;
    unordered_map<int, shared_ptr<Node2d>> close_set;

    Vec2i start_idx = getIndexFromPosition(Vec2d{start_pos(0), start_pos(1)});
    Vec2i goal_idx = getIndexFromPosition(Vec2d{goal_pos(0), goal_pos(1)});

    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_pq.emplace(start_node->getIndex(), start_node->getF());
    open_set.emplace(start_node->getIndex(), start_node);

    // cout << "即将进入循环" << endl;
    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;//从起点开始，每次取代价值F最低的
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        if (cur_node->getIndex() == goal_node->getIndex()) {
            //取到目标点
            final_node_ = cur_node;
            break;
        }
        
        close_set.emplace(cur_index, cur_node);
        // 八邻域搜索
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        //cout << "得到邻居节点" << endl;
        // 得邻居节点
        for (auto &neighbor_node : neighbor_nodes) {
            //检测是否在地图以内
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            //cout << "在地图以内" << endl;

            //是否在障碍物上
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            //cout << "不在障碍物上" << endl;

            //占据栅格模板-线碰撞检测
            if(!configurationSpace_ptr_->isTraversable(neighbor_node->getPosXInMap(), neighbor_node->getPosYInMap(), neighbor_node->getDIR())){
                continue;
            }

            //是否在close set 里，表示是否已经探索过该节点
            if (close_set.find(neighbor_node->getIndex()) != close_set.end()) {
                continue;
            }
            //未探索过该节点
            // 未处在open_set中，新节点
            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                ++explored_node_num;
                //该节点至目标点的欧式距离：H值
                double heuristic = EuclidDistance(neighbor_node->getPosXInMap(),neighbor_node->getPosYInMap(),
                                                goal_node->getPosXInMap(), goal_node->getPosYInMap());
                neighbor_node->setH(heuristic);//H值
                neighbor_node->setParent(cur_node);//父节点
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);//加入open_set
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getF());//以F值为序存入open_pq
            } else {
                // 更新G值：走过的距离
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setDIR(neighbor_node->getDIR());//更新方向
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }
        }
    }

    //已经找到了一条路径，提取折角
    std::vector<Point2D> path={};

    if (final_node_ == nullptr) {
        cout << "没找到一条路径" << endl;
        return path;
    }

    // heu_astar_ = final_node_->getG() * resolution_;
    // cout << "Astar Heuristic:" << heu_astar_ << endl;
    cout << "探索的节点数是：" << explored_node_num << endl;

    shared_ptr<Node2d> cur_node = final_node_;
    double yaw0 = cur_node->getDIR();
    // ROS_INFO("yaw0=%f",yaw0);
    //检验终点朝向是否和A*扩展朝向处于同一半区
    auto dyaw = goal_pos(2)-yaw0;
    mod2Pi(dyaw);
    if(std::abs(dyaw)>M_PI_2) goal_pos(2)=goal_pos(2)+M_PI;
    // goal_pos(2) = (std::abs(goal_pos(2)-yaw0)>M_PI_2? goal_pos(2)+M_PI : goal_pos(2));
    mod2Pi(goal_pos(2));

    //目标点放入path
    Point2D pose = {cur_node->getPosXInMap(),cur_node->getPosYInMap()};//行列索引
    path.push_back(pose);
    // 核心的while循环，回溯过程
    while (cur_node->getParentNode() != nullptr) {
        if(yaw0 != cur_node->getDIR())
        {
            yaw0 = cur_node->getDIR();
            pose.x = cur_node->getPosXInMap();
            pose.y = cur_node->getPosYInMap();
            path.push_back(pose);
        }
        cur_node = cur_node->getParentNode();
    }
    //此时的cur_node为起点
    pose.x = cur_node->getPosXInMap();
    pose.y = cur_node->getPosYInMap();
    path.push_back(pose);
    // 反转,得到一条仅包含折点的path
    reverse(path.begin(), path.end()); 

    //检验起点朝向是否和A*扩展朝向处于同一半区
    dyaw = start_pos(2)-yaw0;
    mod2Pi(dyaw);
    start_pos(2) = (std::abs(dyaw)>M_PI_2? start_pos(2)+M_PI : start_pos(2));
    mod2Pi(start_pos(2));


    // 可视化所有A*路点
    // int id = 0;//标记id号
    // visualization_msgs::Marker path_point;
    // auto point_marker = [&](std::vector<Point2D> const& points) {
    //     path_point.id = id++;
    //     path_point.header.frame_id = "map";
    //     path_point.header.stamp = ros::Time::now();
    //     path_point.type = visualization_msgs::Marker::POINTS;
    //     path_point.scale.x = 0.1;
    //     path_point.scale.y = 0.1;
    //     path_point.scale.z = 0.1;
    //     path_point.color.a = 1;
    //     path_point.color.r = 1;
    //     path_point.color.g = 0;
    //     path_point.color.b = 0;
    //     path_point.pose.orientation.w = 1.0;
    //     for (size_t i = 0; i < points.size(); i++) {
    //         geometry_msgs::Point p;
    //         p.x = (points[i].x+0.5)* resolution_ + x_min_;
    //         p.y = (points[i].y+0.5)* resolution_ + y_min_;
    //         // p.z = 0.05;
    //         path_point.points.push_back(p);
    //     }
    // };
    // point_marker(path);
    // path_point_vis_pub.publish(path_point);

    return path;

}

inline void GridSearch::mod2Pi(double &angle) {
    if (angle >  M_PI)  angle -= 2 *M_PI;
    if (angle < - M_PI)  angle += 2 * M_PI;
}


/**
 * @brief 通过A星的距离作为启发式函数，一般不用，因为每次扩展节点都用一次A星累积起来太多了
 * 
 * @param start_pos 
 * @param goal_pos 
 * @return true 
 * @return false 
 */
double GridSearch::calculateHeuByAstar(Vec2d start_pos, Vec2d goal_pos) {
    // cout << "A star search" << endl;
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;
    unordered_map<int, shared_ptr<Node2d>> close_set;

    Vec2i start_idx = getIndexFromPosition(start_pos);
    Vec2i goal_idx = getIndexFromPosition(goal_pos);
    //cout << "start_idx:" << start_idx << endl;

    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_pq.emplace(start_node->getIndex(), start_node->getF());
    open_set.emplace(start_node->getIndex(), start_node);

    // cout << "即将进入循环" << endl;
    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        if (cur_node->getIndex() == goal_node->getIndex()) {
            final_node_ = cur_node;
            break;
        }
        
        close_set.emplace(cur_index, cur_node);
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        //cout << "得到邻居节点" << endl;
        for (auto &neighbor_node : neighbor_nodes) {
            //检测是否在地图以内
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            //cout << "在地图以内" << endl;

            //是否在障碍物上
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            //cout << "不在障碍物上" << endl;

            //是否在close set 里
            if (close_set.find(neighbor_node->getIndex()) != close_set.end()) {
                continue;
            }

            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                ++explored_node_num;
                double heuristic = 
                EuclidDistance(neighbor_node->getPosXInMap(),neighbor_node->getPosYInMap(),
                               goal_node->getPosXInMap(), goal_node->getPosYInMap());//欧式距离
                neighbor_node->setH(heuristic);
                neighbor_node->setParent(cur_node);
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getF());
            } else {
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }
        }
    }
    if (final_node_ == nullptr) {
        cout << "没找到一条路径" << endl;
        return false;
    }
    // double heu_astar_ = final_node_->getG() * resolution_;
    // cout << "Astar Heuristic:" << heu_astar_ << endl;
    explored_node_num_ += explored_node_num;
    num++;
    cout << "探索的节点数是：" << (int)(explored_node_num_/num) << endl;
    return final_node_->getG() * resolution_;
}

void GridSearch::clear(){
    explored_node_num_ = 0;
    num = 0;
}

/**
 * @brief 先生成一个到终点的距离表，每次扩展都直接查表即可，但是构建这个表也要花很多时间
 *  生成查表用的dp_map_，避免Heuristic的在线计算，加速求解过程
 * @param goal_pos 
 * @return true 
 * @return false 
 */
bool GridSearch::generateDpMap(Vec2d goal_pos, Vec2d start_pos) {
    // open_pq_队列根据节点cost由小到达的顺序排列
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;

    dp_map_.clear();

    Vec2i goal_idx = getIndexFromPosition(goal_pos);
    Vec2i start_idx = getIndexFromPosition(start_pos);
    //生成限定区域用于生成距离表，不然规划速度太慢了
    VecXd limit_zone(4);
    limit_zone(0) = std::min(goal_idx(0), start_idx(0))-100;
    limit_zone(1) = std::max(goal_idx(0), start_idx(0))+100;
    limit_zone(2) = std::min(goal_idx(1), start_idx(1))-100;
    limit_zone(3) = std::max(goal_idx(1), start_idx(1))+100;
    
    // 多个智能指针可以指向同一个对象，并且会自动在最后一个指针不再需要时释放内存
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_set.emplace(goal_node->getIndex(), goal_node);
    open_pq.emplace(goal_node->getIndex(), goal_node->getG());

    int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        dp_map_.emplace(cur_index, cur_node);//放入
        //从目标点开始拓展
        // std::move将获取到的邻居节点转移到了neighbor_nodes中，避免了不必要的拷贝操作，提高了性能
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        //cout << "得到邻居节点" << endl;
        // 用广度优先搜索记录每个节点到终点节点的最短遍历距离
        //neighbor_nodes存储了当前节点四周的邻节点
        for (auto &neighbor_node : neighbor_nodes) {

            //检测是否在地图以内
            if (!isInGridMap(neighbor_node)) {
                continue;
            }
            //cout << "在地图以内" << endl;

            //是否在限定范围内，不然规划太慢了
            if (!isInLimitedMap(neighbor_node, limit_zone)) {
                continue;
            }            

            //是否在障碍物上
            if (isOnObstacle(neighbor_node)) {
                continue;
            }
            //cout << "不在障碍物上" << endl;

            // 检查dp_map_中是否已经存在了neighbor_node->getIndex()对应的键值
            if (dp_map_.find(neighbor_node->getIndex()) != dp_map_.end()) {
                continue;
            }
/*这段代码实现了A*算法的核心逻辑。在处理邻居节点时，首先检查该节点是否已经被加入到开放集open_set中。如果没有，就将该节点加入到开放集中，并计算该节点的代价值（g值、f值），同时更新探索过的节点数量explored_node_num。
如果该节点已经在open_set中，说明之前已经探索过该节点，需要更新该节点的代价值和父节点信息。具体来说，首先比较该节点在open_set中保存的g值和当前计算的g值大小，如果后者更小，就更新g值和父节点信息。这一步保证了从起点到该节点的路径是最短的。
然后，不论是新加入的节点还是更新了代价值的节点，都需要将其加入到优先队列open_pq中。在A*算法中，通过优先队列来维护开放集，每次选择代价值最小的节点进行扩展。这里使用了自定义的比较函数，按照节点的f值进行比较，以实现代价值最小的节点在队列头部。
总之，这段代码的作用是实现A*算法的核心逻辑，将当前节点的邻居节点加入到开放集中，并更新它们的代价值和父节点信息。同时，使用优先队列来维护开放集，确保每次选择代价值最小的节点进行扩展。*/
            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                ++explored_node_num;
                neighbor_node->setParent(cur_node);
                //  节点没有遍历过，也没有发生碰撞，就加入到开放集open_set中，计算节点代价值。
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getG());
            } else {
                // 更新G值：走过的距离，因为是从终点往前走
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }

        }

    }

    cout << "搜索的节点数是：" << explored_node_num << endl; 
    return true;
}


bool GridSearch::generateLocalDpMap(Vec2d goal_pos, Vec2d start_pos) {
    // open_pq_队列根据节点cost由小到达的顺序排列
    priority_queue<pair<int, double>, vector<pair<int, double>>, cmp> open_pq;
    unordered_map<int, shared_ptr<Node2d>> open_set;

    dp_map_.clear();

    Vec2i goal_idx = getIndexFromPosition(goal_pos);
    Vec2i start_idx = getIndexFromPosition(start_pos);
    //生成限定区域用于生成距离表，不然规划速度太慢了
    VecXd limit_zone(4);
    limit_zone(0) = std::min(goal_idx(0), start_idx(0))-40;
    limit_zone(1) = std::max(goal_idx(0), start_idx(0))+40;
    limit_zone(2) = std::min(goal_idx(1), start_idx(1))-40;
    limit_zone(3) = std::max(goal_idx(1), start_idx(1))+40;
    
// 多个智能指针可以指向同一个对象，并且会自动在最后一个指针不再需要时释放内存
    shared_ptr<Node2d> goal_node = make_shared<Node2d>(goal_idx, map_size_x_);

    open_set.emplace(goal_node->getIndex(), goal_node);
    open_pq.emplace(goal_node->getIndex(), goal_node->getG());

    // int explored_node_num = 0;
    while (!open_pq.empty()) {
        int cur_index = open_pq.top().first;
        open_pq.pop();
        shared_ptr<Node2d> cur_node = open_set[cur_index];

        dp_map_.emplace(cur_index, cur_node);//放入
        //从目标点开始拓展
        // std::move将获取到的邻居节点转移到了neighbor_nodes中，避免了不必要的拷贝操作，提高了性能
        vector<shared_ptr<Node2d>> neighbor_nodes = move(getNeighborNodes(cur_node));
        //cout << "得到邻居节点" << endl;
        // 用广度优先搜索记录每个节点到终点节点的最短遍历距离
        //neighbor_nodes存储了当前节点四周的邻节点
        for (auto &neighbor_node : neighbor_nodes) {

            // //检测是否在地图以内
            // if (!isInGridMap(neighbor_node)) {
            //     continue;
            // }

    int index_x = neighbor_node->getPosXInMap();//列索引
    int index_y = neighbor_node->getPosYInMap();
    // cout << "到这了吗" << endl;
    // cout << "index_x: " << index_x << endl;
    // cout << "index_y: " << index_y << endl;
    // cout << "map_size_y_" << map_size_y_ << endl;

    if (index_x < 0 || index_x >= map_size_x_ || index_y < 0 || index_y >= map_size_y_
        //100为障碍物栅格值，-1为未知区域栅格值
        || map_->data[neighbor_node->getIndex()] == 100 || map_->data[neighbor_node->getIndex()] == -1) {
        continue;
    }

            //是否在限定范围内，不然规划太慢了
            if (!isInLimitedMap(neighbor_node, limit_zone)) {
                continue;
            }            

            //是否在障碍物上
            // if (isOnObstacle(neighbor_node)) {
            //     continue;
            // }
            //cout << "不在障碍物上" << endl;

// 检查dp_map_中是否已经存在了neighbor_node->getIndex()对应的键值
            if (dp_map_.find(neighbor_node->getIndex()) != dp_map_.end()) {
                continue;
            }
/*这段代码实现了A*算法的核心逻辑。在处理邻居节点时，首先检查该节点是否已经被加入到开放集open_set中。如果没有，就将该节点加入到开放集中，并计算该节点的代价值（g值、f值），同时更新探索过的节点数量explored_node_num。
如果该节点已经在open_set中，说明之前已经探索过该节点，需要更新该节点的代价值和父节点信息。具体来说，首先比较该节点在open_set中保存的g值和当前计算的g值大小，如果后者更小，就更新g值和父节点信息。这一步保证了从起点到该节点的路径是最短的。
然后，不论是新加入的节点还是更新了代价值的节点，都需要将其加入到优先队列open_pq中。在A*算法中，通过优先队列来维护开放集，每次选择代价值最小的节点进行扩展。这里使用了自定义的比较函数，按照节点的f值进行比较，以实现代价值最小的节点在队列头部。
总之，这段代码的作用是实现A*算法的核心逻辑，将当前节点的邻居节点加入到开放集中，并更新它们的代价值和父节点信息。同时，使用优先队列来维护开放集，确保每次选择代价值最小的节点进行扩展。*/
            if (open_set.find(neighbor_node->getIndex()) == open_set.end()) {
                // ++explored_node_num;
                neighbor_node->setParent(cur_node);
                //  节点没有遍历过，也没有发生碰撞，就加入到开放集open_set中，计算节点代价值。
                open_set.emplace(neighbor_node->getIndex(), neighbor_node);
                open_pq.emplace(neighbor_node->getIndex(), neighbor_node->getG());
            } else {
                if (open_set[neighbor_node->getIndex()]->getG() > neighbor_node->getG()) {
                    open_set[neighbor_node->getIndex()]->setG(neighbor_node->getG());
                    open_set[neighbor_node->getIndex()]->setParent(cur_node);
                }
            }

        }

    }

    // cout << "搜索的节点数是：" << explored_node_num << endl; 
    return true;
}


/**
 * @brief 查询到终点的距离
 * 
 * @param start_pos 
 * @return double 
 */
double GridSearch::lookupInDpMap(Vec2d start_pos) {
    Vec2i start_idx = getIndexFromPosition(start_pos);
    shared_ptr<Node2d> start_node = make_shared<Node2d>(start_idx, map_size_x_);

    if (dp_map_.find(start_node->getIndex()) != dp_map_.end()) {
        return dp_map_[start_node->getIndex()]->getG() * resolution_;
    } else {
        return numeric_limits<double>::infinity();
    }
}



//获得四周节点
vector<shared_ptr<Node2d>> GridSearch::getNeighborNodes(shared_ptr<Node2d> cur_node) {
    int cur_node_x = cur_node->getPosXInMap();//列索引
    int cur_node_y = cur_node->getPosYInMap();//行索引
    int cur_node_g = cur_node->getG();
    //栅格对角线长度，栅格长度
    double diagonal_distance = sqrt(2);//直接写值，运算快一点
    vector<shared_ptr<Node2d>> neighbors;
    // 每个节点可以向上、下、左、右，左上、右上、左下、右下八个方向遍历
    for (int i = -1; i <= 1; ++i) {
        for (int j = -1; j <= 1; ++j) {
            Vec2i neighbor_idx{cur_node_x + i, cur_node_y + j};
            shared_ptr<Node2d> neightbor = make_shared<Node2d>(neighbor_idx, map_size_x_);
            if (i ==0 && j == 0) continue;
            // 根据i，j可以知道当前扩展节点neightbor的朝向（相对于cur_node），朝向规定只要全局统一，就只会有8个方向的取值
            if (sqrt(i * i + j * j) > 1) {
                neightbor->setG(cur_node_g + diagonal_distance);//对角
            } else {
                neightbor->setG(cur_node_g + 1);
            }
            //赋上朝向,用于判断前后不同朝向的节点作为折线转折点
            std::pair<int, int> key = {i, j};
            // 查找并获取值
            auto iter = directionAngles.find(key);
            if(iter != directionAngles.end()) {
                neightbor->setDIR(iter->second);
            }
            neighbors.emplace_back(neightbor);
        }
    }
    return neighbors;
}


inline bool GridSearch::isInGridMap(shared_ptr<Node2d> node) {
    int index_x = node->getPosXInMap();//列索引
    int index_y = node->getPosYInMap();
    // cout << "到这了吗" << endl;
    // cout << "index_x: " << index_x << endl;
    // cout << "index_y: " << index_y << endl;
    // cout << "map_size_y_" << map_size_y_ << endl;

    if (index_x < 0 || index_x >= map_size_x_ || index_y < 0 || index_y >= map_size_y_) {
        return false;
    }
    
    return true;
}


//是否在限定范围内
inline bool GridSearch::isInLimitedMap(shared_ptr<Node2d> node, VecXd lzone) {
    int index_x = node->getPosXInMap();//列索引
    int index_y = node->getPosYInMap();
    // cout << "到这了吗" << endl;
    // cout << "index_x: " << index_x << endl;
    // cout << "index_y: " << index_y << endl;
    // cout << "map_size_y_" << map_size_y_ << endl;

    if (index_x < lzone(0) || index_x > lzone(1) || index_y < lzone(2) || index_y > lzone(3)) {
        return false;
    }
    
    return true;
}


//是否处于障碍物上
inline bool GridSearch::isOnObstacle(shared_ptr<Node2d> node) {
    // int index_x = node->getPosXInMap();//列索引
    // int index_y = node->getPosYInMap();
    //真实坐标
    // Vec2d pos = getPositionFromIndex({index_x, index_y});
    // cout << "index_x: " << pos(0) << endl;
    // cout << "index_y: " << pos(1) << endl;

    // auto idx = index_x + index_y * map_size_x;

    return (map_->data[node->getIndex()] == 100 || map_->data[node->getIndex()] == -1);//100为障碍物栅格值，-1为未知区域栅格值

    // if (map_.atPosition("elevation", pos) > 0) {
    //     return true;
    // }
    // return false;
}

/**
 * @brief 从终点回溯整条路径
 * 
 * @return vector<Vec2d> 
 */
vector<Vec2d> GridSearch::getAstartPath() {
    shared_ptr<Node2d> cur_node = final_node_;
    vector<shared_ptr<Node2d>> vec;
    vector<Vec2d> res;
    //cout << "回溯 " << endl;

    while (cur_node->getParentNode() != nullptr) {
        vec.emplace_back(cur_node);
        cur_node = cur_node->getParentNode();

    }
    reverse(vec.begin(), vec.end());
    //cout << "vec 大小：" << vec.size() << endl;

    for (auto &node : vec) {
        
        res.push_back({node->getPosXInMap() * resolution_ + x_min_,
                       node->getPosYInMap() * resolution_ + y_min_ });
        //cout << "cur_node->getPosXInMap():" << node->getPosXInMap() << endl;
    }
    return res;
}


inline double GridSearch::EuclidDistance(const double x1, const double y1,
                                  const double x2, const double y2) {
  return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}


inline Vec2i GridSearch::getIndexFromPosition(Vec2d position) {
    Vec2i index;
    index(0) = std::floor((position(0) - x_min_) / resolution_ + 0.5);
    index(1) = std::floor((position(1) - y_min_) / resolution_ + 0.5);
    return index;
}


inline Vec2d GridSearch::getPositionFromIndex(Vec2i index) {
    Vec2d pos;
    pos(0) = x_min_ + (index(0) + 0.5) * resolution_;
    pos(1) = y_min_ + (index(1) + 0.5) * resolution_;
    return pos;
}



/*前置步骤：提取A*中折角，构成多个直线段*/
/*这个函数会对A*的每一条直线路径，细分取路径点
edge:从某一个起点开始到各路径点的可通行连线，edge长为距离值
最后用Djsktra搜出最短路径*/
/*
@brief 将astar路径分段、拉直，生成路线长度最短的折线路径
@param mapPlan 栅格地图，10cm分辨率
@param path astar稀疏的路径点----这个已经是取好了的包含每条直线段的path了
@return 返回拉直的路径点，也是比较稀疏的/或者原始的astar路径（折线路径）*/
std::vector<Point2D> GridSearch::PathShorten(std::vector<Point2D> const& path)
{
    // 传入的是稀疏A*前后两个像素路径点，即前后两个栅格点
    auto TaceSegSparse = [&](Point2D point_one, Point2D point_two) -> std::vector<Point2D> {
        std::vector<Point2D> path_sparse;
        //存入起点
        path_sparse.push_back(point_one);
        const float k_seg_len = 1.0 / resolution_; //每一个小段的长度1.0m，对应10个像素距离
        // 计算两点之间的距离，单位：格，像素距离
        float dis             = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
        // 根据设定的小段长度 k_seg_len 将距离分割成多个小段
        int seg_num           = dis / k_seg_len + 0.5;
        if (seg_num < 2) {
            return path_sparse;
        }
        // 使用线性插值的方法，在每个小段之间生成新的路径点
        float delta_x = (point_two.x - point_one.x) / (1.f * seg_num);
        float delta_y = (point_two.y - point_one.y) / (1.f * seg_num);
        for (int i = 1; i < seg_num; i++) {
            //+ 0.5f：为了将计算出来的点坐标从实数空间取整到离该点最近的整数坐标，即四舍五入到最近的整数坐标。这种处理方式有时可以消除由于浮点数运算误差引起的偏差，确保得到的点在直线上更精确地位置。
            Point2D seg_point{static_cast<int>(point_one.x + delta_x * i + 0.5f), static_cast<int>(point_one.y + delta_y * i + 0.5f)};
            path_sparse.push_back(seg_point);
        }
        return path_sparse;
    };

    auto TacePathSparse = [=](std::vector<Point2D> basePoints) -> std::vector<Point2D> {
        if (basePoints.size() < 3) { //不超过2个点
            return basePoints;
        }
        std::vector<Point2D> ret;
        for (unsigned int i = 0; i < basePoints.size() - 1; i++) {
            // 插值，传入的是稀疏A*前后两个像素路径点,即前后两个栅格点
            // 返回插值后的路径点
            std::vector<Point2D> temp = TaceSegSparse(basePoints[i], basePoints[i + 1]);
            // ret.emplace_back(basePoints[i]);
            ret.insert(ret.end(), temp.begin(), temp.end());
        }
        ret.emplace_back(basePoints.back());//插入终点 
        return ret;
    };

    // 优化：拉直拉短
    auto path_points_sparse = TacePathSparse(path);//返回插值后的A*路径
    std::vector<Point2D> path_adjusted;
    bool is_path_adjust_work = false;
    do {
        PathAdjust pp(path_points_sparse.size());//传入插值后的A*路径点数，初始化图顶点
        if (path_points_sparse.size() < 3) {
            break;
        }
        // 将A*上相邻路点间的连线段作为边
        for (size_t i = 0; i < path_points_sparse.size() - 1; i++) {
            auto point_one = path_points_sparse[i];
            auto point_two = path_points_sparse[i + 1];
            // 计算相邻点之间的距离
            float dis      = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
            // 距离作为边的权重
            pp.AddEdge(i, i + 1, dis);
        }
        // 
        for (size_t i = 0; i < path_points_sparse.size() - 2; i++) {
            // linehit
            int meetHit = 0;
            for (size_t j = i + 2; j < path_points_sparse.size(); j++) {
                // int x1 = path_points_sparse[i].x;
                // int y1 = path_points_sparse[i].y;
                Point2D p1{path_points_sparse[i].x, path_points_sparse[i].y};
                // int x2 = path_points_sparse[j].x;
                // int y2 = path_points_sparse[j].y;
                Point2D p2{path_points_sparse[j].x, path_points_sparse[j].y};
                // int hitX;
                // int hitY;
                // 检查是否有障碍物，如果有则停止插入新点
                // 使用了画线算法----，判断是否经过了黑色像素
                // if (LineHit(hitX, hitY, 128, x1, y1, x2, y2,
                //              &mapPlan.map[0], mapPlan.mapParam.width, mapPlan.mapParam.height)) {
                    // LOGD("hitPoint = (%.2f, %.2f)", slamMap.idx2x(hitX), slamMap.idx2y(hitY));
                if (!LineHit(p1, p2)) {//返回 true，表示路径上没有障碍物
                    // 两点连线经过障碍
                    meetHit++;  // 连线发生hit最大允许向前搜索的次数
                    if (meetHit > 5) {
                        break;
                    } else {
                        continue;
                    }
                } else {
                    // 如果成功找到无障碍路径，则计算新边edge的长度
                    auto point_one = path_points_sparse[i];
                    auto point_two = path_points_sparse[j];
                    float dis      = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
                    pp.AddEdge(i, j, dis);
                }
            }
        }
        std::vector<int> ret = pp.GetShortestPath();//获取最短路径，返回顶点索引
        for (auto item : ret) {
            path_adjusted.push_back(path_points_sparse[item]);//保存路径点坐标
        }
        // 如果经过调整后的路径长度比原稀疏路径短
        if (path_adjusted.size() < path_points_sparse.size()) {
            // FLOGD("--PATH ADJUST WORKS.");
            is_path_adjust_work = true;
        }
    } while (0);
    
    if (is_path_adjust_work == true) {
        // 更新路点
        return path_adjusted;
    } else {
        // 保持原路径
        return path;
    }
}


// 直接对稀疏的原始A*插值，取得路点——————对照实验
std::vector<Point2D> GridSearch::PathInterpolation(std::vector<Point2D> const& path)
{
    // 传入的是稀疏A*前后两个像素路径点，即前后两个栅格点
    auto TaceSegSparse = [&](Point2D point_one, Point2D point_two) -> std::vector<Point2D> {
        std::vector<Point2D> path_sparse;
        //存入起点
        path_sparse.push_back(point_one);
        const float k_seg_len = 0.3 / resolution_; //每一个小段的长度0.3m，对应3个像素距离
        // 计算两点之间的距离，单位：格，像素距离
        float dis             = sqrt((point_one.x - point_two.x) * (point_one.x - point_two.x) + (point_one.y - point_two.y) * (point_one.y - point_two.y));
        // 根据设定的小段长度 k_seg_len 将距离分割成多个小段
        int seg_num           = dis / k_seg_len + 0.5;
        if (seg_num < 2) {
            return path_sparse;
        }
        // 使用线性插值的方法，在每个小段之间生成新的路径点
        float delta_x = (point_two.x - point_one.x) / (1.f * seg_num);
        float delta_y = (point_two.y - point_one.y) / (1.f * seg_num);
        for (int i = 1; i < seg_num; i++) {
            //+ 0.5f：为了将计算出来的点坐标从实数空间取整到离该点最近的整数坐标，即四舍五入到最近的整数坐标。这种处理方式有时可以消除由于浮点数运算误差引起的偏差，确保得到的点在直线上更精确地位置。
            Point2D seg_point{static_cast<int>(point_one.x + delta_x * i + 0.5f), static_cast<int>(point_one.y + delta_y * i + 0.5f)};
            path_sparse.push_back(seg_point);
        }
        return path_sparse;
    };

    auto TacePathSparse = [=](std::vector<Point2D> basePoints) -> std::vector<Point2D> {
        if (basePoints.size() < 3) { //不超过2个点
            return basePoints;
        }
        std::vector<Point2D> ret;
        for (unsigned int i = 0; i < basePoints.size() - 1; i++) {
            // 插值，传入的是稀疏A*前后两个像素路径点,即前后两个栅格点
            // 返回插值后的路径点
            std::vector<Point2D> temp = TaceSegSparse(basePoints[i], basePoints[i + 1]);
            // ret.emplace_back(basePoints[i]);
            ret.insert(ret.end(), temp.begin(), temp.end());
        }
        ret.emplace_back(basePoints.back());//插入终点 
        return ret;
    };

    return TacePathSparse(path);//返回插值后的A*路径
}


// 判断两点连线是否经过障碍物
bool GridSearch::LineHit(const Point2D pose1, const Point2D pose2)
{
  //栅格索引转真实坐标，索引->坐标
  std::pair<double, double> p1;
  p1.first = (pose1.x+0.5) * resolution_ + x_min_;
  p1.second = (pose1.y+0.5) * resolution_ + y_min_;

  std::pair<double, double> p2;
  p2.first = (pose2.x+0.5) * resolution_ + x_min_;
  p2.second = (pose2.y+0.5) * resolution_ + y_min_;

  std::pair<double, double> ptmp={};

//   计算两个位姿之间的距离
  double dist = sqrt( (p2.second-p1.second) * (p2.second-p1.second) +
                      (p2.first-p1.first) * (p2.first-p1.first) );
  if (dist < resolution_) return true;
  else{
// 如果距离大于等于分辨率，则根据分辨率将路径分为若干个点，依次检查这些点上是否存在障碍物
// 类似画线算法？
    int value = int(floor(dist/resolution_));//这个dis在这里是真实距离
    // 计算两个位姿之间的角度 theta
    double theta = atan2( (p2.second - p1.second), (p2.first - p1.first) );
    int n = 1;//步长
    // 循环遍历每个分辨率点，计算该点的坐标，并检查该点是否有障碍物
    for (int i = 0;i < value; i++)
    {
        // 计算当前点的坐标 ptmp
      ptmp.first = p1.first + resolution_*cos(theta) * n;
      ptmp.second = p1.second + resolution_*sin(theta) * n;
      // 调用 collision 函数检查该点是否有障碍物，如果有障碍物则返回 false
      if (collision(ptmp.first, ptmp.second)) return false;
      n++;
    }
    // 如果所有分辨率点上都没有障碍物，则返回 true，表示路径上没有障碍物----
    return true;
  }
}

inline bool GridSearch::collision(const double x, const double y)
{
  unsigned int mx,my,mxy;
  //真实坐标->索引
  mx = (int)((x - x_min_) / resolution_);
  my = (int)((y - y_min_) / resolution_);
  mxy = mx + my * map_size_x_;

 if (mx < 0 || mx >= map_size_x_ || my < 0 || my >= map_size_y_ || 
    map_->data[mxy] == 100 || map_->data[mxy] == -1) return true;

  return false;
}


// 求栅格距
inline double GridSearch::Distance(const Point2D& a, const Point2D& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

// 获取折线段上的点和它们到末端点的距离
void GridSearch::getPointsOnPolyline(const std::vector<Point2D>& polyline, float interval, std::vector<GridPoint>& points) {
    //首先计算最短折线路径的总栅格长度
    double totalLength = 0;
    for (u_int i = 0; i < polyline.size() - 1; ++i) {
        totalLength += Distance(polyline[i], polyline[i + 1]);
    }
    //按长度取间隔
    u_int numPoints = std::ceil(totalLength / interval);
    points.resize(numPoints);

    u_int pointIndex = 0;
    double lengthMoved = 0;
    // 从折线段的起始点开始，沿着折线段移动指定的距离来获取每个点的位置
    for (u_int i = 0; i < polyline.size() - 1 && pointIndex < numPoints; ++i) {
        // 计算相邻两大点长度
        double segmentLength = Distance(polyline[i], polyline[i + 1]);
        while (lengthMoved + interval <= segmentLength && pointIndex < numPoints) {
            //计算移动一小段后该小点占该段的比例
            double ratio = (lengthMoved + interval) / segmentLength;
            // 计算插入点的坐标
            points[pointIndex].x = polyline[i].x + std::round(ratio * (polyline[i + 1].x - polyline[i].x));
            points[pointIndex].y = polyline[i].y + std::round(ratio * (polyline[i + 1].y - polyline[i].y));
            // 通过减去已经移动过的距离来计算每个点到折线段末端点的路径距离
            points[pointIndex].distance_to_goal = totalLength - (lengthMoved + interval);
            // 累计已经移动过的距离
            lengthMoved += interval;
            // 点数+1
            ++pointIndex;
        }
        totalLength -= segmentLength;
        lengthMoved -= segmentLength;
    }
}

// 转换
void toPCLPointCloud(const std::vector<GridPoint>& points, pcl::PointCloud<pcl::PointXY>::Ptr cloud_) {
    for (const auto& point : points) {
        pcl::PointXY p = {static_cast<float>(point.x), static_cast<float>(point.y)};
        cloud_->push_back(p);
    }
}
/*计算当前格点到终点的近似astar路径:
-最短折线路径细分成多个路径点，存到一个kdtree里面，对当前的节点，找到最近的无碰撞路径点，计算最近的路径点后续的astar路径距离*/
/*在astar路径(稀疏之后的)上找到一个点，使得该点到当前节点最近且连线可通行，
将连线长度与这个点后续的astar路径长度之和(也可以直接使用这个点后续的astar路径长度)用来计算启发值H;*/
void GridSearch::InitLookUpTable(Vec3d& goal_pos, Vec3d& start_pos){
    // 原始A*,仅取了各直线段的首末点,对可视化来说不影响
    std::vector<Point2D> ret = calculateHeuByAstar(start_pos, goal_pos);

    const float k_seg_len = 0.3 / resolution_; //每一个小段的长度0.3m(同planning::step_size_)，对应3个像素距离
    
    #if 1
    // 被拉直的A*最短折线路线
    std::vector<Point2D> polyline = PathShorten(ret);
    // 获取最短折线段上的点和它们到终点的距离
    astar_points.clear();
    getPointsOnPolyline(polyline, k_seg_len, astar_points);
    
    #else
    // 对照实验——直接由原始A*路线作参考路径
    getPointsOnPolyline(ret, k_seg_len, astar_points);
    // std::vector<Point2D> polyline = PathInterpolation(ret);
    #endif

    ROS_INFO("astar_points=%ld",astar_points.size());

    //可视化astar_points 路点
    // int id = 0;//标记id号
    // visualization_msgs::Marker path_point;
    // auto point_marker = [&](std::vector<Point2D> const& points) {
    //     path_point.id = id++;
    //     path_point.header.frame_id = "map";
    //     path_point.header.stamp = ros::Time::now();
    //     path_point.type = visualization_msgs::Marker::POINTS;
    //     path_point.scale.x = 0.1;
    //     path_point.scale.y = 0.1;
    //     path_point.scale.z = 0.1;
    //     path_point.color.a = 1;
    //     path_point.color.r = 1;
    //     path_point.color.g = 0;
    //     path_point.color.b = 0;
    //     path_point.pose.orientation.w = 1.0;
    //     for (size_t i = 0; i < points.size(); i++) {
    //         geometry_msgs::Point p;
    //         p.x = (points[i].x+0.5)* resolution_ + x_min_;
    //         p.y = (points[i].y+0.5)* resolution_ + y_min_;
    //         // p.z = 0.05;
    //         path_point.points.push_back(p);
    //     }
    // };
    // point_marker(ret);
    // path_point_vis_pub.publish(path_point);

    // 存到一个kdtree里面
    // 把std::vector<GridPoint>转换为pcl::PointCloud<pcl::PointXY>
    cloud->clear();
    toPCLPointCloud(astar_points, cloud);
    // 创建kd-tree
    kdtree.setInputCloud(cloud);
    cout << "点云个数:" << cloud->size() << endl;
    

    //可视化原始A*和折线A*
    //可视化前需要将行列索引转真实坐标
    std::vector<geometry_msgs::PoseStamped> mret;
    std::vector<geometry_msgs::PoseStamped> mret_;
    for(auto const& idx : ret){
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = (idx.x+0.5) * resolution_ + x_min_;
        p.pose.position.y = (idx.y+0.5) * resolution_ + y_min_;
        mret.push_back(p);
    }
    vis.publishPrimitiveAstar(mret);
    for(auto const& idx : polyline){
        geometry_msgs::PoseStamped p;
        p.header.frame_id = "map";
        p.pose.position.x = (idx.x+0.5) * resolution_ + x_min_;
        p.pose.position.y = (idx.y+0.5) * resolution_ + y_min_;
        mret_.push_back(p);
    }
    vis.publishSimpleAstar(mret_);

}


/*在astar路径(稀疏之后的)上找到一个点，使得该点到当前节点最近且连线可通行，
将连线长度与这个点后续的astar路径长度之和(也可以直接使用这个点后续的astar路径长度)用来计算启发值H;*/
// 通过距离表查询距离值,pos是一个真实坐标
double GridSearch::CheckLookUpTable(Vec2d start_pos){
    Vec2i start_idx = getIndexFromPosition(start_pos);
    pcl::PointXY search_point = {static_cast<float>(start_idx(0)), static_cast<float>(start_idx(1))}; //搜索点
    int K = 1; // number of points to consider 暂时先设置为1
    std::unordered_set<int> excludedPoints;

    // 边界栅格平方距，气泡带？？ 4m*2带宽  减小搜索空间；
    // 只在有限指定区域进行节点扩展；减小搜索空间；----这个只在宽道路、少点障碍的时候用

    // 远距离点剔除
    const float dis2 =40. * 40.;

    // 使用kdtree.nearestKSearch或者kdtree.radiusSearch来查找最近的点
    // K=1，表示最近邻搜索
    while (true) {
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        // 参数：pointIdxNKNSearch这些点的索引，pointNKNSquaredDistance：它们到搜索点的平方距离
        if (kdtree.nearestKSearch(search_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            //有最近点
            for (int i = 0; i < pointIdxNKNSearch.size(); ++i) {
                if (excludedPoints.find(pointIdxNKNSearch[i]) == excludedPoints.end()) {
                    // 不在排除列表中：
                    // 最近的第i个点
                    pcl::PointXY nearestPoint = kdtree.getInputCloud()->points[pointIdxNKNSearch[i]];
                    // 先判断最近点和搜索点需要在一定范围内

                    float distanceSquared = pointNKNSquaredDistance[i];
                    if(distanceSquared>dis2) {
                        // excludedPoints.insert(pointIdxNKNSearch[i]);//表示不符合要求的点
                        return numeric_limits<double>::infinity();;//平方栅格距离
                    }

                    // 判断两点连线是否经过障碍物
                    Point2D pose1 = {start_idx(0), start_idx(1)};
                    Point2D pose2 = {static_cast<int>(nearestPoint.x), static_cast<int>(nearestPoint.y)};
                    if (LineHit(pose1, pose2)) {
                        // 不发生碰撞，可通行，返回从当前点到A*终点的距离
                        // nearestPoint.z: Distance to goal along A* path
                        return (std::sqrt(distanceSquared) + astar_points[pointIdxNKNSearch[i]].distance_to_goal) * resolution_;//返回真实距离
                    }
                    // 发生碰撞
                    excludedPoints.insert(pointIdxNKNSearch[i]);//表示不符合要求的点
                }
            }
            K++;
            if(K>7) return numeric_limits<double>::infinity();
        } else {
            throw std::runtime_error("No connectable point found");
            return numeric_limits<double>::infinity();
        }
    }
}





} // namespace planning

