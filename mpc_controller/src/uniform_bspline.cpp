#include "uniform_bspline.h"
#include <ros/ros.h>

namespace ego_planner
{

  UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                 const double &interval)
  {
    setUniformBspline(points, order, interval);
  }

  UniformBspline::~UniformBspline() {}

/*
输入参数为：点，阶数，时间间隔
功能：初始化成员变量：控制点矩阵、B样条次数、时间间隔、节点向量
*/
  void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                         const double &interval)
  {
    control_points_ = points;//控制点,控制点数=n+1
    p_ = order;//阶数=次数+1，这里这个order是次数
    interval_ = interval;//时间间隔

    n_ = points.cols() - 1;     //n=控制点数-1
    m_ = n_ + p_ + 1;           //m=节点数-1=n（控制点数-1）+ p（次数）+1  节点向量数=控制点数+阶数  将曲线分成m段：t0...tm——节点表，如等分的节点表[0,...，1]

    u_ = Eigen::VectorXd::Zero(m_ + 1);//节点向量，节点数=m+1

    // 这样小车不会过冲，终点附近也可以靠惯性。
    // for (int i = 0; i <= m_; ++i)//[u0,...um],共m+1个节点
    // {
    //   //我们取p=3，interval_=0.8
    //   if (i <= p_)//[u0,...,up]
    //   {
    //     u_(i) = double(-p_ + i) * interval_;//[-2.4，-1.6，-0.8，0],up=0
    //   }
    //   else if (i > p_ && i <= m_ - p_)//[u(p+1),...,u(m-p)]  m-p=n+1
    //   {
    //     u_(i) = u_(i - 1) + interval_;//[0.8,1.6,...,...]
    //   }
    //   else if (i > m_ - p_) //最后p个节点
    //   {
    //     u_(i) = u_(i - 1) + interval_;
    //   }
    // }
//    cout<<" Display U : "<<endl;
//    for(int i=0;i<u_.size();i++)
//    {
//        cout<<u_(i)<<" , ";
//        if(i%10==0&&i!=0) cout<<endl;
//    }

    //准均匀B样条曲线，严格过控制点的端点
    for (int i = 0; i <= m_; ++i)//[u0,...um],共m+1个节点
    {
      //假设取p=3（次数），interval_=0.8
      if (i <= p_)//[u0,...,up]
      {
        u_(i) = 0;
        // u_(i) = double(-p_ + i) * interval_;//[-2.4，-1.6，-0.8，0],up=0
      }
      else if (i > p_ && i <= m_ - p_)//[u(p+1),...,u(m-p)]  m-p=n+1
      {
        u_(i) = u_(i - 1) + interval_;//[0.8,1.6,...,...]
      }
      else if (i > m_ - p_) //最后p+1个节点
      {
        // u_(i) = u_(i - 1) + interval_;
        u_(i) = u_(i - 1);
      }
    }
  }


  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

  //得到轨迹的时长。从up=0到u(n+1)
  bool UniformBspline::getTimeSpan(double &um, double &um_p)
  {
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
      return false;

    um = u_(p_);
    um_p = u_(m_ - p_);

    return true;
  }

  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }


  // 首先定位到u在哪个区间内，然后使用deBoor递推公式求出u时刻B样条曲线的二维坐标。
  // 获得一个[t_p,t_m-p]作用域中的B样条函数值
  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {
    double ub = min(max(u_(p_), u), u_(m_ - p_));//在[up,u(n+1)]范围内

    // determine which [ui,ui+1] lay in
    // 找出输入的u是时间节点的第几个节点处
    int k = p_;//初始化k次
    while (true)
    {
      if (u_(k + 1) >= ub)//k就是deBoor's alg中的j
        break;
      ++k;
    }

    /* deBoor's alg */
    //通过k找到与当前节点相关的控制点是k-p_到k
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)//令i + k - p_=j后回代，j再改为i，发现就是deBoor's alg的递推式
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  //根据B样条曲线的性质，对于k阶B样条曲线，其导数的控制点为..,得到B样条曲线的导数的控制点
  //位置曲线->速度曲线
  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);//用于存储导数曲线的控制点
    for (int i = 0; i < ctp.cols(); ++i)
    {
      // 根据公式计算导数曲线的第i列控制点
      ctp.col(i) =
          //根据公式填充导数曲线的控制点
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  // 调用getDerivativeControlPoints()得到导数的控制点后，新初始化一个均匀B样条类，阶数为原来阶数-1，时间间隔一样，区间间隔为原来间隔的去头去尾，最终得到B样条曲线的导数曲线。
  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();//得到导数曲线的控制点
    UniformBspline derivative(ctp, p_ - 1, interval_);//阶数（次数）-1，我们改版之后还是准均匀

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);//去除首尾节点
    // 将节点向量u_从索引1开始，取长度为u_.rows() - 2的子向量赋值给knot，即去除了首尾节点
    knot = u_.segment(1, u_.rows() - 2);//还是准均匀
    derivative.setKnot(knot);//设置导数曲线的节点

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }


  void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
  {
    limit_vel_ = vel;//x、y速度极限、线速度极限
    limit_acc_ = acc;//加速度极限
    limit_ratio_ = 1.1;//时间区间放大比例极限  时间拉伸的比率
    feasibility_tolerance_ = tolerance;//容忍值
  }

  //验证轨迹的速度和加速度是否超过动力学限制（只检查各控制点处的大小），符合限制返回True，否则返回False并得到最大超过限制的比例ratio。
  //最好把速度、加速度适配线速度的
  bool UniformBspline::checkFeasibility(double &ratio, bool show)
  {
    bool fea = true;//可行性

    Eigen::MatrixXd P = control_points_;//控制点
    int dimension = control_points_.rows();//控制点的维度：2

    /* check vel feasibility and insert points 检查速度可行性*/
    double max_vel = -1.0;//初始化
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 1; ++i)//遍历控制点，计算每个控制点之间的速度
    {
      Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

      // if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
      //     fabs(vel(2)) > enlarged_vel_lim)
      // if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim 
      //     || vel.norm()  > enlarged_vel_lim)
      if (vel.norm()  > enlarged_vel_lim)
      {
        if (show)
          cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
        fea = false;
        // for (int j = 0; j < dimension; ++j){
        //   max_vel = max(max_vel, fabs(vel(j)));//记录最大速度值
        // }
         max_vel = max(max_vel, vel.norm());
      }
    }

    /* acc feasibility 检查加速度可行性*/
    double max_acc = -1.0;//初始化
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
    for (int i = 0; i < P.cols() - 2; ++i){
      Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

      // if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
      //     fabs(acc(2)) > enlarged_acc_lim)
      // if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim
      //   || acc.norm() > enlarged_acc_lim)
      if (acc.norm() > enlarged_acc_lim)
      {
        if (show)
          cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
        fea = false;
        // for (int j = 0; j < dimension; ++j){
        //   max_acc = max(max_acc, fabs(acc(j)));//记录最大加速度值
        // }
        max_acc = max(max_acc, acc.norm());
      }
    }
    // 计算最大速度与限制速度之间的比率，以及最大加速度的绝对值与限制加速度之间的平方根的比率
    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea;
  }

  // 把轨迹的时间延长为原来的ratio倍
  void UniformBspline::lengthenTime(const double &ratio)//时间拉伸的比率
  {
    int num1 = 5;//阶数=4的
    int num2 = getKnot().rows() - 1 - 5;

    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));//时间变化量
    double t_inc = delta_t / double(num2 - num1);//每个控制点时间参数的增量
    for (int i = num1 + 1; i <= num2; ++i)//利用循环逐一更新控制点的时间参数值
      u_(i) += double(i - num1) * t_inc;
    for (int i = num2 + 1; i < u_.rows(); ++i)
      u_(i) += delta_t;//后面的都加delta_t，准均匀
  }

  // void UniformBspline::recomputeInit() {}

/*
函数参数：ts：轨迹执行时间、point_set：原轨迹点集合、start_end_derivative：起点与终点的高阶约束
输出：更新ctrl_pts控制点矩阵的值
函数功能：将给定点集和起始/终止导数转换为 B-样条曲线的控制点矩阵，通过对原始轨迹的拟合得到B样条轨迹的控制点
*/
  void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                             const vector<Eigen::Vector3d> &start_end_derivative,
                                             Eigen::MatrixXd &ctrl_pts)
  {
    // 确保时间步长大于0
    if (ts <= 0){
      cout << "[B-spline]:time step error." << endl;
      return;
    }
    // 确保点集中至少有4个点
    if (point_set.size() <= 3){
      cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
      return;
    }
    //起点与终点的导数限制（即一阶和二阶导数，有此信息才可以确保得到一个具有足够信息的线性方程组，从而可以求解出 B-样条曲线的控制点。）
    if (start_end_derivative.size() != 4){
      cout << "[B-spline]:derivatives error." << endl;
    }
    //记录原曲线路经点个数
    int K = point_set.size();

  //该矩阵用来构建线性方程组， B-样条曲线参数化过程中求解控制点
  //一阶 B-样条基函数的系数向量、一阶 B-样条基函数的导数系数向量、二阶 B-样条基函数的系数向量
  //取该数值的原因是B样条曲线矩阵表示论文中提出的，以满足 B-样条曲线的一些良好性质。
    // write A
    Eigen::Vector3d prow(3), vrow(3), arow(3);//位置、速度、加速度
    prow << 1, 4, 1;
    vrow << -1, 0, 1;
    arow << 1, -2, 1;

    // 创建一个大小为(K+4)×(K+2)的零矩阵A
    //K+4是因为添加了四行导数约束信息
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    // 循环遍历点集中的每个点，将相应的值存储在向量bx、by和bz中
    for (int i = 0; i < K; ++i)
    // 根据公式填充矩阵A的第i行和第i到(i+2)列。公式为(1/6)*prow转置，其中prow为固定的系数向量
      A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();
    // 根据公式填充矩阵A的第K行和第1到3列。公式为(1/2/ts)*vrow转置，其中vrow为固定的系数向量
    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    // 根据公式填充矩阵A的第(K+2)行和第1到3列。公式为(1/ts/ts)*arow转置，其中arow为固定的系数向量
    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    //cout << "A" << endl << A << endl << endl;

    // 创建三个长度为(K+4)的向量bx、by和bz，用于存储点集和起始/结束导数的值
    // write b
    // Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
    Eigen::VectorXd bx(K + 4), by(K + 4);
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      // bz(i) = point_set[i](2);
    }

    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      // bz(K + i) = start_end_derivative[i](2);
    }

    // 使用QR分解求解线性方程组Ax=b，其中A为矩阵，x和b为向量。分别求解x、y和z方向的控制点
    // solve Ax = b
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    // Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // 调整控制点矩阵的大小为3×(K+2)
    // convert to control pts
    // ctrl_pts.resize(3, K + 2);
    ctrl_pts.resize(2, K + 2);
    // 将x方向的控制点存储在控制点矩阵的第一行
    ctrl_pts.row(0) = px.transpose();
    // 将y方向的控制点存储在控制点矩阵的第二行
    ctrl_pts.row(1) = py.transpose();
    // 将z方向的控制点存储在控制点矩阵的第三行
    // ctrl_pts.row(2) = pz.transpose();

    // cout << "[B-spline]: parameterization ok." << endl;
  }

  //其实就是u(n+1)
  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
