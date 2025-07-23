#include "mpc_tracking/mpc.h"


Mpc::Mpc(int N, double dt) {
    N_ = N;//预测步
    dt_ = dt;//时间间隔
    u_max_ = 0.5;//最大线速度
    w_max_ = 0.5;//最大角速度
    vector<double> weights = {3.5,3.5,1.5,0.5,0.5}; //Q,R
    u_min_ = 0;
    w_min_ = - w_max_;
    
    Q_ = DM::zeros(3,3); //索引之前初始化size
    R_ = DM::zeros(2,2);
    

    setWeights(weights);
    //运动学方程
    kinematic_equation_ = setKinematicEquation();
}

Mpc::~Mpc() {}


void Mpc::setWeights(vector<double> weights) {
    //cout << "setweights" << endl;
    Q_(0, 0) = weights[0];//QR
    Q_(1, 1) = weights[1];
    Q_(2, 2) = weights[2];
    R_(0, 0) = weights[3];
    R_(1, 1) = weights[4];
    //R_(2, 2) = weights[5];
    //cout << "set weight finish" << endl;
}


//运动学方程
Function Mpc::setKinematicEquation() {
    //cout << "set kinematic" << endl;
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX theta = MX::sym("theta");//yaw
    MX state_vars = MX::vertcat({x, y, theta});//状态向量

    MX v = MX::sym("v");
    MX w = MX::sym("w");
    MX control_vars = MX::vertcat({v, w});//控制向量
    
    //rhs means right hand side
    //状态方程右侧项，差速运动学
    MX rhs = MX::vertcat({v * MX::cos(theta), v * MX::sin(theta), w});
    return Function("kinematic_equation", {state_vars, control_vars}, {rhs});
    // MX x = MX::sym("x");
    // MX y = MX::sym("y");
    // MX theta = MX::sym("theta");
    // MX state_vars = MX::vertcat({x, y, theta});

    // MX u = MX::sym("u");
    // MX v = MX::sym("v");
    // MX w = MX::sym("w");
    // MX control_vars = MX::vertcat({u, v, w});

    // MX rhs = u * MX::cos(theta) - v * MX::sin(theta);
    // rhs = MX::vertcat({rhs, u * MX::sin(theta) + v * MX::cos(theta), w});
    // return Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}


bool Mpc::solve(Eigen::Vector3d current_states, Eigen::MatrixXd desired_states) {
    
    //cout << "jinqiujiele" << endl;
    Opti opti = Opti();//创建优化器

    Slice all;//切片工具，用于选择矩阵的所有行或列

    MX cost = 0;//初始化代价函数
    X = opti.variable(3, N_ + 1);//声明状态变量矩阵
    U = opti.variable(2, N_);//声明控制变量矩阵
    MX x = X(0, all);//提取状态变量中的x坐标
    MX y = X(1, all);//提取状态变量中的y坐标
    MX theta = X(2, all);//提取状态变量中的航向角
    MX v = U(0, all);//提取控制变量中的线速度
    MX w = U(1, all);//提取控制变量中的角速度
    //MX w = U(2, all);


    MX X_ref = opti.parameter(3, N_ + 1);//声明参考状态参数矩阵
    MX X_cur = opti.parameter(3);//声明当前状态参数向量
    DM x_tmp1 = {current_states[0], current_states[1], current_states[2]};//当前状态向量

    opti.set_value(X_cur, x_tmp1);  //set current state 设置当前状态
    // cout << "set current state success" << endl;

    
    //将desired_states转换为一维向量，并设置参考状态
    vector<double> X_ref_v(desired_states.data(), desired_states.data() + desired_states.size());
    //auto tp1 = std::chrono::steady_clock::now();
    DM X_ref_d(X_ref_v);
    
    //X_ref_d.resize(3, N_ + 1);
    
    //cout << "desired_states:" << desired_states << endl;
    //cout << "X_ref_v:" << X_ref_v << endl;
    //cout << "X_ref_d:" << X_ref_d << endl;
    // DM x_tmp2(3, N_ + 1);
    // for (int i = 0; i < 3; ++i) {
    //     for (int j = 0; j <= N_; ++j) {
    //         x_tmp2(i, j) = desired_states(i, j);
    //     }
    // }
    X_ref = MX::reshape(X_ref_d, 3, N_ + 1);//将一维向量重塑为参考状态矩阵
    
    
    
    //opti.set_value(X_ref, X_ref_d);  //set reference traj

    // auto tp2 = std::chrono::steady_clock::now();
    // cout <<"set trajectory time:" 
    // << chrono::duration_cast<chrono::microseconds>(tp2 - tp1).count() 
    // << "microseconds" << endl;
    //cout << "set reference traj success" << endl;
    //cout << "x_ref:" << X_ref.size() << endl;

    //set costfunction 设置代价函数
    for (int i = 0; i < N_; ++i) {
        MX X_err = X(all, i) - X_ref(all, i); //计算状态误差
        MX U_0 = U(all, i);//提取当前控制输入
        //cout << "U_0 size:" << U_0.size() << endl;
        //cout << "cost size:" << cost_.size() << endl;
        // 将状态误差X_err通过权重矩阵Q_进行加权，并累加到成本函数cost中。
        cost += MX::mtimes({X_err.T(), Q_, X_err});
        //cout << "cost size:" << cost_.size() << endl; 
        // 将输入U_0通过权重矩阵R_进行加权，并累加到成本函数cost中。
        cost += MX::mtimes({U_0.T(), R_, U_0});
        //cout << "cost size:" << cost_.size() << endl;
    }
    //cout << "cost size:" << cost_.size() << endl;
    // 计算最后一个时间步的状态变量X的误差项（X(all, N_) - X_ref(all, N_)），然后将其通过权重矩阵Q_进行加权
    cost += MX::mtimes({(X(all, N_) - X_ref(all, N_)).T(), Q_,
                        X(all, N_) - X_ref(all, N_)});
    //cout << "cost:" << cost << endl;
    opti.minimize(cost);//最小化cost
    //cout << "set cost success" << endl;

    //kinematic constrains //表示系统的运动学方程，用于预测未来的状态
    for (int i = 0; i < N_; ++i) {
        vector<MX> input(2);
        input[0] = X(all, i);
        input[1] = U(all, i);
        MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i);//推测下N时刻的状态
        opti.subject_to(X_next == X(all, i + 1));//预测状态约束
    }

    //init value
    opti.subject_to(X(all, 0) == X_cur);//设置初始状态约束

    //speed angle_speed limit
    
    // 设置控制输入约束（例如输入限制）
    opti.subject_to(0 <= v <= u_max_);//线速度限制
    opti.subject_to(w_min_ <= w <= w_max_);//角速度限制

    //set solver
    casadi::Dict solver_opts;
    solver_opts["expand"] = true; //MX change to SX for speed up 将MX类型转换为SX类型，以提高计算速度
    solver_opts["ipopt.max_iter"] = 100; //指定了 IPOPT 求解器的最大迭代次数
    solver_opts["ipopt.print_level"] = 0; //关闭 IPOPT 求解器的详细输出信息
    solver_opts["print_time"] = 0; //关闭时间输出
    solver_opts["ipopt.acceptable_tol"] = 1e-6; //指定了 IPOPT 求解器接受的最终目标函数值的容差
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6; //指定了 IPOPT 求解器接受的目标函数变化的容差

    opti.solver("ipopt", solver_opts); //将设置好的优化器选项应用到优化问题实例 opti 中，并指定了使用 IPOPT 求解器进行求解

    //auto sol = opti.solve();
    // 调用 opti.solve() 函数来求解优化问题，该函数返回一个包含有关优化结果的对象
    solution_ = std::make_unique<casadi::OptiSol>(opti.solve());

    return true;
}
vector<double> Mpc::getFirstU() {
    vector<double> res;
    //首个控制量u
    auto first_v =  solution_->value(U)(0, 0);//线速度
    auto first_w = solution_->value(U)(1, 0);//角速度
    
    //cout << "first_u" << first_u << " " << "first_v" << first_v << endl;

    res.push_back(static_cast<double>(first_v));
    res.push_back(static_cast<double>(first_w));
    return res;
}

vector<double> Mpc::getPredictX() {
    vector<double> res;
    auto predict_x = solution_->value(X);
    // cout << "nomal" << endl;
    //cout << "predict_x size :" << predict_x.size() << endl;
    for (int i = 0; i <= N_; ++i) {
        res.push_back(static_cast<double>(predict_x(0, i)));
        res.push_back(static_cast<double>(predict_x(1, i)));
    }
    return res;
}
