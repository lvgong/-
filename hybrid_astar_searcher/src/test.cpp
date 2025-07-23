// 常规的扩展方式会导致频繁的转弯或者不必要的转弯；
// 首先计算出起点到终点的最短折线路径，作为参考路径；
// 在参考路径上等距选取多个子目标点位姿，每个子目标点的位姿朝向就是参考线在该点的直线方向；
// 使用hybrid进行搜索；额外的，在搜索过程中会优先尝试命中子目标点，如果命中了就继续命中后续的子目标点，如果无法命中就使用传统的hybrid扩展方法，直到成功到达终点位姿；

// 上述操作中，命中子目标点能够在空旷的区域/参考线的长直线段上有效减少节点扩展的数量，加快搜索到终点位姿的速度；

// 搜索过程中增加气泡带，限制搜索空间
// 绘制路径上的区域[局部走廊/气泡带BubbleZone]，5个车身宽度足迹，四十个像素，只考虑路径足迹范围，用于优化voronoi生成时间
        // TODO 这里改成动态分配，预防图片太大
        {
            cv::Mat mask = cv::Mat::zeros(planMapCv_raw.rows, planMapCv_raw.cols, CV_8UC1);
            planMapCv    = planMapCv_raw.clone();
            planMapCv.setTo(0);
            // cv::line(mask, cv::Point(333,6666), cv::Point(200,200),cv::Scalar(255,255,255),40,8,0);
            for (unsigned int i = 0; i < astarPath.size() - 1; i++) {
                cv::line(mask, cv::Point(astarPath[i].X, astarPath[i].Y),
                         cv::Point(astarPath[i + 1].X, astarPath[i + 1].Y),
                         cv::Scalar(255, 255, 255), 20 * 4, 8, 0); // 这里绘制的是圆头直线，相当于圆形；astarPath路径点是稀疏的
            }
            // todo 注意在边缘画上黑线
            // cv::imwrite("mask.bmp", mask); // 读写文件很耗时to delete
            planMapCv_raw.copyTo(planMapCv, mask);
            // cv::imwrite("planMapCv.bmp", planMapCv); // 读写文件很耗时to delete
        }


[key]多个子目标的生成
在最短折线段上挑选长直线段，并间隔采样；
在参考路径上等距选取多个子目标点位姿，每个子目标点的位姿朝向就是参考线在该点的直线方向；

// step 计算子目标点
        std::vector<Pose2D> sub_goals;
        do {
            // 对astarPath 中的长直线段采样
            const double k_subGoal_dis_interval =  0.5f; // 间隔0.5m采样一个路径点
            const double K_cut_segment_thresh = 1.f; // 长度至少为1m才采样
            std::vector<Pose2D> pose_sparse_list; // 原始路径点pose,物理坐标
            for (auto item : astarPath) {
                pose_sparse_list.push_back(Pose2D(mapData.mapParam.idx2x(item.X), mapData.mapParam.idx2y(item.Y), 0));
            }

            for (size_t i = 0; i < pose_sparse_list.size() - 1; i++) {
                float dist = pose_sparse_list[i].GetDistance(pose_sparse_list[i+1]);
                if (dist < K_cut_segment_thresh || dist < k_subGoal_dis_interval) {
                    continue;
                }
                LineSegment2d segment({pose_sparse_list[i].X, pose_sparse_list[i].Y}, {pose_sparse_list[i + 1].X, pose_sparse_list[i + 1].Y});
                Pose2D segment_start; // 线段上的起始位姿
                segment_start.X = segment.start().x();
                segment_start.Y = segment.start().y();
                segment_start.SetPhi(segment.heading());
                int num_inserts =  std::ceil(dist / k_subGoal_dis_interval) - 1;
                for (int j = 1; j <= num_inserts; ++j) {
                    float ratio = static_cast<float>(j) / (num_inserts + 1);
                    // new_poses.push_back(interpolate(path.poses[i], path.poses[i + 1], ratio));
                    sub_goals.push_back(segment_start + Pose2D(ratio * segment.length(), 0, 0)); // 计算线段上的子目标点
                }
            }
            // poseEnd
            sub_goals.push_back(poseEnd);
        } while (0);


单次命中单个子目标的代码实现

bool hybrid_a_star::DubinsShotSubGoal(Node *current_node, Pose2D subGoal, float real_dubins_radius, Node **newNode)
{
    Pose2D start = GetNodeSegEnd(current_node);
    Pose2D endPose = subGoal;
    // start
    double q0[] = {start.X, start.Y, start.Phi()};
    if (q0[2] < 0) {
        q0[2] = WrapTo2Pi(q0[2]);
    }
    // goal
    double q1[] = {endPose.X, endPose.Y, endPose.Phi()};
    if (q1[2] < 0) {
        q1[2] = WrapTo2Pi(q1[2]);
    }
    // initialize the path
    DubinsSimple::DubinsPath path;
    // calculate the path
    int res = DubinsSimple::dubins_init(q0, q1, real_dubins_radius, &path);
    if (res != 0) {
        return false;
    }
    std::vector<Pose2D> path_lists;
    float x = 0.f;
    // x += k_dubins_step_size;  // 防止路径点重叠，重叠了也无所谓，每个路径点的pose角度是单独算出来的，和下一个路径点没有关系
    float length = dubins_path_length(&path);
    if (length < 0.01f) {
        return false;
    }
    while (x <= length) {
        double q[3];
        dubins_path_sample(&path, x, q); // 在曲线上按照x长度取点
        Pose2D pose(q[0], q[1], WrapToPi(q[2]));
        // collision check
        if (CheckPoseCollisionSafe(pose) == false) {
            return false;
        }
        path_lists.push_back(pose);
        x += k_dubins_step_size;
    }
    {
        int index         = Pose2Index(path_lists.back());
        double added_cost = 0.0;
        added_cost += STEER_COST * fabs(0);                              // 转角，曲率代价，增大此项可减少尖锐的曲率 【转角与astar转角趋势相同时减小该项，顺应astar路径的转角趋势】
        // note todo 最好使用real_dubins_radius计算steer angle
        // added_cost += STEER_CHANGE_COST * fabs(WrapToPi(path_lists.back().Phi() - start.Phi())); // 转角变化代价，增大此项可减少频繁转弯【这里可以结合astar路径做调整，在astar路径长直段避免转角变化，在转弯处允许变化】
        double cost        = current_node->cost ;//+ added_cost ;//length * 10; // 这里arc_len最好乘上一个系数，arc_len用于优化路径长度
        auto neighbor_node = new Node(index, start, length, 0, current_node->state_grid_index, cost, true);
        // 记录子目标点命中时候的dubins路径
        *newNode = neighbor_node;
        mSubGoalPaths[neighbor_node] = path_lists;
    }
    return true;
}

【key】命中多个子目标
优先尝试命中子目标点，如果命中了就继续命中后续的子目标点，如果无法命中就使用传统的hybrid扩展方法，直到成功到达终点位姿；
使用多个半径命中，优先使用大的半径命中；
注意，每一次尝试命中都会做碰撞检测，会耗时；
为了优化耗时，子目标点在十米范围内，而且优先命中最远的那个；
// 命中子目标
        // todo 优化：如果起始终止位姿距离很近就只给一个终点位姿作为子目标点，不要命中中间的其他点。
        {
            valid_sub_goal = false;
            // 确定子目标点的范围；
            int index_fathest = mSubGoals.size() - 1; // 当前搜索的最远的子目标点的序号 Farthest
            {
                // 子目标点在十米范围内
                const float k_SubGoalDisInReach = 10.f; 
                int index = std::max(sub_goal_counter, 0);
                float dis_sum = 0;
                do {
                    if (index == mSubGoals.size() - 1) {
                        break;
                    }
                    index++;
                    if (index == mSubGoals.size() - 1) {
                        break;
                    }
                    dis_sum += mSubGoals[index].GetDistance(mSubGoals[index - 1]);
                    if (dis_sum > k_SubGoalDisInReach) {
                        break;
                    }
                } while (1);
                index_fathest = index;
            }
            // 子目标点在十米范围内
            for (int i = index_fathest /*mSubGoals.size() - 1*/; i > sub_goal_counter; --i) {
                Pose2D sub_goal = mSubGoals[i];
                // 使用dubins连接并判断是否命中
                do {
                    // bool hybrid_a_star::DubinsShotSubGoal(Node *current_node, Pose2D subGoal, float real_dubins_radius, Node *newNode)
                    Node *newNode = nullptr;
                    // 使用多个半径命中，优先使用大的半径命中
                    bool res = false;
                    for (float dubins_radius = k_dubins_radius_max; dubins_radius >= k_dubins_radius_min; dubins_radius -= k_dubins_radius_add_step) {
                        res = DubinsShotSubGoal(current_node_ptr, sub_goal, dubins_radius, &newNode); // k_dubins_radius_min
                        if (res == true) {
                            break; // 优先命中就结束
                        }
                    }
                    // 未命中则换一个目标点
                    if (res == false) {
                        break;
                    }
                    if (mCloseList.find(newNode->state_grid_index) != mCloseList.end()) {
                        // 已经搜索和扩展过的
                        delete newNode;
                        break;
                    }
                    if (mOpenList.find(newNode->state_grid_index) == mOpenList.end()) {
                        // 计算G+H
                        // 直接命中到终点，结束搜索
                        if (i == mSubGoals.size() - 1) {
                            m_terminal_node_ptr_ = newNode;
                            return true; // 命中终点
                        }
                        double f_cost = newNode->cost + ComputeH(newNode);
                        mOpenList.insert(std::make_pair(newNode->state_grid_index, newNode));
                        openset_.insert(std::make_pair((uint64_t)(f_cost * 1000), newNode->state_grid_index));
                    } else if (mOpenList.find(newNode->state_grid_index) != mOpenList.end()) {
                        delete newNode;
                        break;
                    }
                    sub_goal_counter = i;
                    valid_sub_goal   = true; // 能够直接用dubins命中voronoi节点
                } while(0);
            }
            if (valid_sub_goal) {
                continue;
            }
        } // end 命中子目标
// 扩展节点
        GetNeighborNodes(current_node_ptr, neighbor_nodes);
        for (unsigned int i = 0; i < neighbor_nodes.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes[i];
            int neighbor_node_index = neighbor_node_ptr->state_grid_index;// Pose2Index(GetNodeSegEnd(neighbor_node_ptr));
            if(mCloseList.find(neighbor_node_index) != mCloseList.end()) {
                // 已经搜索和扩展过的
                delete neighbor_node_ptr;
                continue;
            }
            if (mOpenList.find(neighbor_node_index) == mOpenList.end()  ) {
                // 计算G+H
                double f_cost = neighbor_node_ptr->cost + ComputeH(neighbor_node_ptr);
                mOpenList.insert(std::make_pair(neighbor_node_ptr->state_grid_index, neighbor_node_ptr));
                openset_.insert(std::make_pair((uint64_t)(f_cost * 1000), neighbor_node_ptr->state_grid_index));
            } else if (mOpenList.find(neighbor_node_index) != mOpenList.end()) {
                // 判断G的大小
//                 if (neighbor_node_ptr->cost < mOpenList[neighbor_node_index]->cost) {
//                     delete mOpenList[neighbor_node_index];
//                     double f_cost = neighbor_node_ptr->cost + ComputeH(neighbor_node_ptr);
////                     mOpenList.insert(std::make_pair(neighbor_node_ptr->state_grid_index, neighbor_node_ptr));
//                     mOpenList[neighbor_node_ptr->state_grid_index] = neighbor_node_ptr;
//                     openset_.insert(std::make_pair(f_cost*1000, neighbor_node_ptr->state_grid_index));
//                 } else {
                    delete neighbor_node_ptr;
//                 }
            }
        }



测试代码
选取多个随机位姿测试

// 选取多个随机位姿
void testHybridMany()
{
    auto copyFileWithIndex = [](int i) -> void {
        std::string sourceFilePath = "/tmp/debug_record/hybrid_canvas.png"; // hybrid_canvas.png
        std::string destinationFilePath;
        // 构建目标文件路径
        std::stringstream ss;
        ss << sourceFilePath.substr(0, sourceFilePath.find_last_of(".")) << "_" << i << sourceFilePath.substr(sourceFilePath.find_last_of("."));
        ss >> destinationFilePath;

        // 打开源文件和目标文件
        std::ifstream sourceFile(sourceFilePath, std::ios::binary);
        std::ofstream destinationFile(destinationFilePath, std::ios::binary);

        // 拷贝文件内容
        destinationFile << sourceFile.rdbuf();

        // 关闭文件
        sourceFile.close();
        destinationFile.close();
    };
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-2, 2);
    std::uniform_real_distribution<double> distribution_angle(-3, 3);
    for (int i = 0; i < 100; i++) {
        TestHybrid test;
        test.SetVisulization(true);
        TMapData mapData({-10, -10, 50 * 20, 50 * 20}, 255);
        for (float x = 0; x < 35; x += 6) {
            for (float y = 0; y < 35; y += 6) {
                int xInt = mapData.x2idx(x);
                int yInt = mapData.y2idx(y);
                LOCALCV::CCommonAlg::circleMat(0, xInt + RandomInt(10, 20), yInt + RandomInt(10, 20), 10 + RandomInt(10, 40), &mapData.map[0], mapData.mapParam.width, mapData.mapParam.height,
                                               0, mapData.mapParam.width, 0, mapData.mapParam.height);
            }
        }
        Pose2D startPose(-5 + distribution(generator), 0 + distribution(generator), 1.7 + distribution_angle(generator));
        Pose2D endPose(35 + distribution(generator), 32 + distribution(generator), -1.7 + distribution_angle(generator));
        bool res = test.test(mapData, startPose, endPose);
        FLOGD("test hybrid result is %d", res);
        if (res == false) {
            continue;
        }
        copyFileWithIndex(i); // 拷贝结果另存
    }
}


改进
额外的，反向的搜索能够解决终点在墙边或者墙角时命中终点困难的问题，提高效率，增加成功率；
额外的，如果从起点开始扩展，close set在一定半径距离 节点数目达到指定数目后无法扩展即认为起点被困；如果从终点开始扩展，close set在一定半径距离内节点数目达到指定数目无法扩展即认为终点被困；

传入起始位姿和终点位姿；注意，最好提前保证两个位姿有好的扩展性：起点位姿能够朝着左前方右前方正前方正常前进运动有限圆弧长度无碰撞，终点位姿能够朝着左后方右后方正后方正常前进运动有限圆弧长度无碰撞，不具有扩展性的起始位姿和终点位姿可能导致规划失败或者消耗大量资源；

传入障碍物二值地图，只有0和255像素；
可选的，可传入多个潜在的目标位姿；
返回：规划结果，起点被困或者终点被困或者成功；如果有多个潜在目标位姿会返回命中的那一个；返回路径；
额外的，可以返回哪一段路径存在窄通道(机器在某个路径点位姿下左右两侧20cm范围内都有障碍物认为是窄通道路径点)；



