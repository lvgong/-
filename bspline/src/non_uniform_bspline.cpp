/**
* This file is part of Fast-Planner.
*
* Copyright 2019 Boyu Zhou, Aerial Robotics Group, Hong Kong University of Science and Technology, <uav.ust.hk>
* Developed by Boyu Zhou <bzhouai at connect dot ust dot hk>, <uv dot boyuzhou at gmail dot com>
* for more information see <https://github.com/HKUST-Aerial-Robotics/Fast-Planner>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* Fast-Planner is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Fast-Planner is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with Fast-Planner. If not, see <http://www.gnu.org/licenses/>.
*/



#include "bspline/non_uniform_bspline.h"
#include <ros/ros.h>

namespace fast_planner {

NonUniformBspline::NonUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                     const double& interval) {
  setUniformBspline(points, order, interval);
}

NonUniformBspline::~NonUniformBspline() {}

void NonUniformBspline::setUniformBspline(const Eigen::MatrixXd& points, const int& order,
                                          const double& interval) {
  control_points_ = points;//每行代表一个控制点，维度为N*3
  p_              = order;//B样条次数
  interval_       = interval;//节点之间的间隔，即delta_t

  n_ = points.rows() - 1;//控制点数为n+1
  m_ = n_ + p_ + 1;//时间间隔数=控制点数+次数

  u_ = Eigen::VectorXd::Zero(m_ + 1);//时间间隔数=控制点数+次数

  // for (int i = 0; i <= m_; ++i) {
  //   if (i <= p_) {
  //     u_(i) = double(-p_ + i) * interval_;
  //   } else if (i > p_ && i <= m_ - p_) {
  //     u_(i) = u_(i - 1) + interval_;
  //   } else if (i > m_ - p_) {
  //     u_(i) = u_(i - 1) + interval_;
  //   }
  // }

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

void NonUniformBspline::setKnot(const Eigen::VectorXd& knot) { this->u_ = knot; }

Eigen::VectorXd NonUniformBspline::getKnot() { return this->u_; }

//得到轨迹的时长。从up=0到u(n+1)
void NonUniformBspline::getTimeSpan(double& um, double& um_p) {
  um   = u_(p_);
  um_p = u_(m_ - p_);
}

Eigen::MatrixXd NonUniformBspline::getControlPoint() { return control_points_; }

pair<Eigen::VectorXd, Eigen::VectorXd> NonUniformBspline::getHeadTailPts() {
  Eigen::VectorXd head = evaluateDeBoor(u_(p_));
  Eigen::VectorXd tail = evaluateDeBoor(u_(m_ - p_));
  return make_pair(head, tail);
}

  // 首先定位到u在哪个区间内，然后使用deBoor递推公式求出u时刻B样条曲线的二维坐标。
  // 获得一个[t_p,t_m-p]作用域中的B样条函数值
Eigen::VectorXd NonUniformBspline::evaluateDeBoor(const double& u) {
  //取节点中间段为有效段
  double ub = min(max(u_(p_), u), u_(m_ - p_));

  // determine which [ui,ui+1] lay in
  // 找出输入的u是时间节点的第几个节点处
  int k = p_;//初始化k次
  while (true) {
    if (u_(k + 1) >= ub) break;//k就是deBoor's alg中的j
    ++k;
  }

  //通过k找到与当前节点相关的控制点是k-p_到k
  /* deBoor's alg */
  vector<Eigen::VectorXd> d;
  for (int i = 0; i <= p_; ++i) {
    d.push_back(control_points_.row(k - p_ + i));
    // cout << d[i].transpose() << endl;
  }


// De Boor递归计算控制点，计算过程中使用了混合参数 alpha，该参数用于混合现有控制点以生成新的控制点。
  for (int r = 1; r <= p_; ++r) {//外循环次数
    for (int i = p_; i >= r; --i) {//内循环控制节点，次数高一次，控制节点少一个
      double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
      // cout << "alpha: " << alpha << endl;
      d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
    }
  }

  //返回计算得到的 B-样条曲线在参数 u 处的点
  return d[p_];
}

Eigen::VectorXd NonUniformBspline::evaluateDeBoorT(const double& t) {
  return evaluateDeBoor(t + u_(p_));
}

  //根据B样条曲线的性质，对于k阶B样条曲线，其导数的控制点为..,得到B样条曲线的导数的控制点
  //位置曲线->速度曲线
Eigen::MatrixXd NonUniformBspline::getDerivativeControlPoints() {
  // The derivative of a b-spline is also a b-spline, its order become p_-1
  // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
  Eigen::MatrixXd ctp = Eigen::MatrixXd::Zero(control_points_.rows() - 1, control_points_.cols());
  for (int i = 0; i < ctp.rows(); ++i) {
    // 根据公式计算导数曲线的第i行控制点
    ctp.row(i) =
    //根据公式填充导数曲线的控制点
        p_ * (control_points_.row(i + 1) - control_points_.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
  }
  return ctp;
}

/*计算求导后的曲线，并且将新的控制点、次数、节点向量记录下来*/
NonUniformBspline NonUniformBspline::getDerivative() {
  Eigen::MatrixXd   ctp = getDerivativeControlPoints();//得到导数曲线的控制点
  NonUniformBspline derivative(ctp, p_ - 1, interval_);//阶数（次数）-1，我们改版之后还是

  /* cut the first and last knot */
  Eigen::VectorXd knot(u_.rows() - 2);//去除首尾节点
      // 将节点向量u_从索引1开始，取长度为u_.rows() - 2的子向量赋值给knot，即去除了首尾节点
  knot = u_.segment(1, u_.rows() - 2);//还是准均匀
  derivative.setKnot(knot);//设置导数曲线的节点

  return derivative;
}

double NonUniformBspline::getInterval() { return interval_; }


void NonUniformBspline::setPhysicalLimits(const double& vel, const double& acc, const double &tolerance) {
  limit_vel_   = vel;//线速度限幅
  limit_acc_   = acc;//线加速度限幅
  limit_ratio_ = 1.1;//时间区间放大比例极限  时间拉伸的比率
  feasibility_tolerance_ = tolerance;//容忍值  0.05=5%百分数
}

// 速度加速度这里计算的是控制点之间的值，因为B样条的特殊性质，限制控制点的速度与加速度可以间接控制B样条的速度与加速度
/*轨迹安全性检查，并更新超阈值比例*/
bool NonUniformBspline::checkFeasibility(bool show) {
  bool fea = true;
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;

  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  /* check vel feasibility and insert points */
  double max_vel = -1.0;
  double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;

  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));//根据控制点计算速度

    // if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||//判断是否有任一维度上的速度超过阈值
    //     fabs(vel(2)) > limit_vel_ + 1e-4) {
    if (vel.norm()  > enlarged_vel_lim)//判断是否有任一维度上的速度超过阈值
    {

      if (show) cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
      fea = false;

      // for (int j = 0; j < dimension; ++j) {
      //   max_vel = max(max_vel, fabs(vel(j)));//采用计算出来的速度更新最大速度
      // }

      max_vel = max(max_vel, vel.norm());
    }
  }

  /* acc feasibility */
  double max_acc = -1.0;
  double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
  for (int i = 0; i < P.rows() - 2; ++i) {

    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));//计算加速度

    // if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
    //     fabs(acc(2)) > limit_acc_ + 1e-4) {//判断是否有任一维度上的加速度超过阈值
    if (acc.norm() > enlarged_acc_lim)
    {
      if (show) cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
      fea = false;

      // for (int j = 0; j < dimension; ++j) {//记录更新最大加速度
      //   max_acc = max(max_acc, fabs(acc(j)));
      // }
      max_acc = max(max_acc, acc.norm());
    }
  }

  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));//计算最大速度或加速度超过约束的

  return fea;
}


/*获取当前控制点与时间向量节点下计算出来的超阈值比例*/
double NonUniformBspline::checkRatio() {
  Eigen::MatrixXd P         = control_points_;
  int             dimension = control_points_.cols();

  // find max vel
  double max_vel = -1.0;
  for (int i = 0; i < P.rows() - 1; ++i) {
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    for (int j = 0; j < dimension; ++j) {
      max_vel = max(max_vel, fabs(vel(j)));
    }
  }
  // find max acc
  double max_acc = -1.0;
  for (int i = 0; i < P.rows() - 2; ++i) {
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
    for (int j = 0; j < dimension; ++j) {
      max_acc = max(max_acc, fabs(acc(j)));
    }
  }
  double ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));
  ROS_ERROR_COND(ratio > 2.0, "max vel: %lf, max acc: %lf.", max_vel, max_acc);

  return ratio;
}


// 对于速度、加速度分配不合理的曲线的时间节点进行重分配，注意速度与加速度不合理时不仅仅要修改当前时间节点，因为与多节点有关都要修改。调整时间比例因子通过计算超过的比例系数获得。
/*检查可行性，重新分配时间节点*/
// 重新分配非均匀B样条的时间节点，以确保其速度和加速度在限制范围内
bool NonUniformBspline::reallocateTime(bool show) {
  // SETY << "[Bspline]: total points size: " << control_points_.rows() << endl;
  // cout << "origin knots:\n" << u_.transpose() << endl;
  bool fea = true;

  Eigen::MatrixXd P         = control_points_;// 控制点矩阵
  int             dimension = control_points_.cols();// 控制点的维度（列数）

  double max_vel, max_acc;

  double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
  /* check vel feasibility and insert points */
  // 检查速度的可行性
  for (int i = 0; i < P.rows() - 1; ++i) {
    // 计算当前段的速度
    Eigen::VectorXd vel = p_ * (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1));
    // 检查速度是否超出限制
    // if (fabs(vel(0)) > limit_vel_ + 1e-4 || fabs(vel(1)) > limit_vel_ + 1e-4 ||
    //     fabs(vel(2)) > limit_vel_ + 1e-4) {
    if (vel.norm()  > enlarged_vel_lim)//判断是否有任一维度上的速度超过阈值
    {
      fea = false;// 标记为不可行
      if (show) cout << "[Realloc]: Infeasible vel " << i << " :" << vel.transpose() << endl;

      max_vel = -1.0;
      // 找到速度的最大值
      // for (int j = 0; j < dimension; ++j) {
      //   max_vel = max(max_vel, fabs(vel(j)));
      // }
      max_vel = max(max_vel, vel.norm());

    /*
    在这段代码中，j = i + 2 处开始调整，是因为速度的计算涉及到两个相邻的控制点，即 P.row(i + 1) 和 P.row(i + 2)。
    为了调整速度，需要改变这两个控制点之间的时间间隔，即 u_(i + p_ + 1) - u_(i + 1)。因此，j 从 i + 2 开始，
    以便调整第 i + 1 和 i + 2 之间的时间节点。
    */
      // 计算新的时间比率
      double ratio = max_vel / limit_vel_ + 1e-4;
      if (ratio > limit_ratio_) ratio = limit_ratio_;
      // 计算新的时间间隔和增量
      double time_ori = u_(i + p_ + 1) - u_(i + 1);//原始时间间隔
      double time_new = ratio * time_ori;//新的时间间隔
      double delta_t  = time_new - time_ori;//计算需要增加的时间增量
      double t_inc    = delta_t / double(p_);//计算每个节点需要增加的时间

       // 调整从 i + 2 到 i + p_ + 1 的时间节点
      for (int j = i + 2; j <= i + p_ + 1; ++j) {
        u_(j) += double(j - i - 1) * t_inc;
        if (j <= 5 && j >= 1) {
          // cout << "vel j: " << j << endl;
        }
      }

      //  调整从 i + p_ + 2 到最后的所有时间节点
      for (int j = i + p_ + 2; j < u_.rows(); ++j) {
        u_(j) += delta_t;//后面的都加delta_t，相当于平移了，其实没有扩大时间跨距，维持准均匀
      }
    }
  }

  double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
  /* acc feasibility */
  // 检查加速度的可行性
  for (int i = 0; i < P.rows() - 2; ++i) {
    // 计算当前段的加速度
    Eigen::VectorXd acc = p_ * (p_ - 1) *
        ((P.row(i + 2) - P.row(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
         (P.row(i + 1) - P.row(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
        (u_(i + p_ + 1) - u_(i + 2));
     // 检查加速度是否超出限制
    // if (fabs(acc(0)) > limit_acc_ + 1e-4 || fabs(acc(1)) > limit_acc_ + 1e-4 ||
    //     fabs(acc(2)) > limit_acc_ + 1e-4) {
    if (acc.norm() > enlarged_acc_lim)
    {
      fea = false;// 标记为不可行
      if (show) cout << "[Realloc]: Infeasible acc " << i << " :" << acc.transpose() << endl;

      max_acc = -1.0;
      // 找到加速度的最大值
      // for (int j = 0; j < dimension; ++j) {
      //   max_acc = max(max_acc, fabs(acc(j)));
      // }
      max_acc = max(max_acc, acc.norm());
       // 计算新的时间比率
      double ratio = sqrt(max_acc / limit_acc_) + 1e-4;//使用平方根调整时间比率以控制加速度
      if (ratio > limit_ratio_) ratio = limit_ratio_;
      // cout << "ratio: " << ratio << endl;
       // 计算新的时间间隔和增量
      double time_ori = u_(i + p_ + 1) - u_(i + 2);//原始时间间隔
      double time_new = ratio * time_ori;//新的时间间隔 
      double delta_t  = time_new - time_ori;//时间增量
      double t_inc    = delta_t / double(p_ - 1);//每个时间节点的增量

      // 对于 i 为 1 或 2 的特殊情况，调整从 2 到 5 的时间节点，并在 6 之后的节点增加4倍的增量。
      if (i == 1 || i == 2) {
        // cout << "acc i: " << i << endl;
        for (int j = 2; j <= 5; ++j) {
          u_(j) += double(j - 1) * t_inc;
        }

        for (int j = 6; j < u_.rows(); ++j) {
          u_(j) += 4.0 * t_inc;
        }

      } else {
        // 对于一般情况，从 i + 3 开始调整到 i + p_ + 1 的时间节点，并调整之后的所有时间节点
        for (int j = i + 3; j <= i + p_ + 1; ++j) {
          u_(j) += double(j - i - 2) * t_inc;
          if (j <= 5 && j >= 1) {
            // cout << "acc j: " << j << endl;
          }
        }

        for (int j = i + p_ + 2; j < u_.rows(); ++j) {
          u_(j) += delta_t;
        }
      }
    }
  }

  return fea;
}

// 把轨迹的时间延长为原来的ratio倍
void NonUniformBspline::lengthenTime(const double& ratio) {
  int num1 = 5;
  int num2 = getKnot().rows() - 1 - 5;

  double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
  double t_inc   = delta_t / double(num2 - num1);//每个控制点时间参数的增量
  for (int i = num1 + 1; i <= num2; ++i) u_(i) += double(i - num1) * t_inc;//利用循环逐一更新控制点的时间参数值
  for (int i = num2 + 1; i < u_.rows(); ++i) u_(i) += delta_t;//后面的都加delta_t，准均匀
}

void NonUniformBspline::recomputeInit() {}

/*
  param：
    - ts:轨迹执行时间
    - point_set:原轨迹点集合
    - start_end_derivative:起点和终点的高阶约束
  output:
    - ctrl_pts:控制点矩阵
  fuction:
    - 将给定点集和起始/终止导数转换为B-Spline曲线的控制点矩阵，通过对原始轨迹的拟合得到B样条轨迹的控制点
*/
void NonUniformBspline::parameterizeToBspline(const double& ts, const vector<Eigen::Vector3d>& point_set,
                                              const vector<Eigen::Vector3d>& start_end_derivative,
                                              Eigen::MatrixXd&               ctrl_pts) {
  if (ts <= 0) {
    cout << "[B-spline]:time step error." << endl;
    return;
  }

  if (point_set.size() < 2) {
    cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
    return;
  }

  if (start_end_derivative.size() != 4) {
    cout << "[B-spline]:derivatives error." << endl;
  }

  //原轨迹点集合的个数
  int K = point_set.size();

  // write A
  // 该矩阵用来构建线性方程组，B-Spline曲线参数化过程中求解控制点
  // 一阶B-Spline基函数的系数向量、一阶B-Spline基函数的导数系数向量、二阶B-Spline基函数的系数向量
  Eigen::Vector3d prow(3), vrow(3), arow(3);
  prow << 1, 4, 1;
  vrow << -1, 0, 1;
  arow << 1, -2, 1;

  // K+4是等式约束个数，K+2是控制点个数
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

  for (int i = 0; i < K; ++i) A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

  A.block(K, 0, 1, 3)         = (1 / 2.0 / ts) * vrow.transpose();
  A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

  A.block(K + 2, 0, 1, 3)     = (1 / ts / ts) * arow.transpose();
  A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();
  // cout << "A:\n" << A << endl;

  // A.block(0, 0, K, K + 2) = (1 / 6.0) * A.block(0, 0, K, K + 2);
  // A.block(K, 0, 2, K + 2) = (1 / 2.0 / ts) * A.block(K, 0, 2, K + 2);
  // A.row(K + 4) = (1 / ts / ts) * A.row(K + 4);
  // A.row(K + 5) = (1 / ts / ts) * A.row(K + 5);

  // write b
  // b矩阵为K个坐标点和2个速度2个加速度组成的（K+4）*3的矩阵
  Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
  for (int i = 0; i < K; ++i) {
    bx(i) = point_set[i](0);
    by(i) = point_set[i](1);
    bz(i) = point_set[i](2);
  }

  for (int i = 0; i < 4; ++i) {
    bx(K + i) = start_end_derivative[i](0);
    by(K + i) = start_end_derivative[i](1);
    bz(K + i) = start_end_derivative[i](2);
  }

  // solve Ax = b
  Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
  Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
  Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

  // convert to control pts
  // 控制点赋值
  ctrl_pts.resize(K + 2, 3);
  ctrl_pts.col(0) = px;
  ctrl_pts.col(1) = py;
  ctrl_pts.col(2) = pz;

  // cout << "[B-spline]: parameterization ok." << endl;
}

//其实就是u(n+1)
double NonUniformBspline::getTimeSum() {
  double tm, tmp;
  getTimeSpan(tm, tmp);
  return tmp - tm;
}

double NonUniformBspline::getLength(const double& res) {
  double          length = 0.0;
  double          dur    = getTimeSum();
  Eigen::VectorXd p_l    = evaluateDeBoorT(0.0), p_n;
  for (double t = res; t <= dur + 1e-4; t += res) {
    p_n = evaluateDeBoorT(t);
    length += (p_n - p_l).norm();
    p_l = p_n;
  }
  return length;
}

double NonUniformBspline::getJerk() {
  NonUniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

  Eigen::VectorXd times     = jerk_traj.getKnot();
  Eigen::MatrixXd ctrl_pts  = jerk_traj.getControlPoint();
  int             dimension = ctrl_pts.cols();

  double jerk = 0.0;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    for (int j = 0; j < dimension; ++j) {
      jerk += (times(i + 1) - times(i)) * ctrl_pts(i, j) * ctrl_pts(i, j);
    }
  }

  return jerk;
}

void NonUniformBspline::getMeanAndMaxVel(double& mean_v, double& max_v) {
  NonUniformBspline vel = getDerivative();
  double            tm, tmp;
  vel.getTimeSpan(tm, tmp);

  double max_vel = -1.0, mean_vel = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
    double          vn  = vxd.norm();

    mean_vel += vn;
    ++num;
    if (vn > max_vel) {
      max_vel = vn;
    }
  }

  mean_vel = mean_vel / double(num);
  mean_v   = mean_vel;
  max_v    = max_vel;
}

void NonUniformBspline::getMeanAndMaxAcc(double& mean_a, double& max_a) {
  NonUniformBspline acc = getDerivative().getDerivative();
  double            tm, tmp;
  acc.getTimeSpan(tm, tmp);

  double max_acc = -1.0, mean_acc = 0.0;
  int    num = 0;
  for (double t = tm; t <= tmp; t += 0.01) {
    Eigen::VectorXd axd = acc.evaluateDeBoor(t);
    double          an  = axd.norm();

    mean_acc += an;
    ++num;
    if (an > max_acc) {
      max_acc = an;
    }
  }

  mean_acc = mean_acc / double(num);
  mean_a   = mean_acc;
  max_a    = max_acc;
}
}  // namespace fast_planner
