/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2016, George Kouros.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author:  George Kouros
*********************************************************************/

#include "AstarPlanner/cubic_spline_interpolator.h"
#include <tf/tf.h>

//三次样条插值

/*  CubicSplineInterpolator::CubicSplineInterpolator(
    double pointsPerUnit,
    unsigned int skipPoints,
    bool useEndConditions,
    bool useMiddleConditions)
    :
      pointsPerUnit_(pointsPerUnit),
      skipPoints_(skipPoints),
      useEndConditions_(useEndConditions),
      useMiddleConditions_(useMiddleConditions)
  {
  }*/

  CubicSplineInterpolator::CubicSplineInterpolator(std::string name)
  {
    ros::NodeHandle pnh("~/" + name);

    pnh.param("points_per_unit", pointsPerUnit_, 5.0);
    pnh.param<bool>("use_end_conditions", useEndConditions_, false);
    pnh.param<bool>("use_middle_conditions", useMiddleConditions_, true);

    int skipPoints;
    pnh.param("skip_points", skipPoints, 0);
    skipPoints_ = abs(skipPoints);
  }


  CubicSplineInterpolator::~CubicSplineInterpolator()
  {
  }


  void CubicSplineInterpolator::interpolatePath(
    const nav_msgs::Path& path,
    nav_msgs::Path& smoothedPath)
  {
    smoothedPath.header = path.header;
    interpolatePath(path.poses, smoothedPath.poses);
  }


  void CubicSplineInterpolator::interpolatePath(
    const std::vector<geometry_msgs::PoseStamped>& path,
    std::vector<geometry_msgs::PoseStamped>& smoothedPath)
  {
    // clear new smoothed path vector in case it's not empty 清除新的平滑路径向量，以防它不是空的
    smoothedPath.clear();

    // set skipPoints_ to 0 if the path contains has too few points 如果路径包含的点太少，则将skipPoints_设置为0
    unsigned int oldSkipPoints = skipPoints_;
    skipPoints_ = std::min<int>(path.size() - 2, skipPoints_);

    // create cummulative distances vector 创建累积距离向量
    std::vector<double> cummulativeDistances;
    calcCummulativeDistances(path, cummulativeDistances);

    // create temp pose 创建临时的点
    geometry_msgs::PoseStamped pose;
    pose.header = path[0].header;

    unsigned int numPoints = pointsPerUnit_ * calcTotalDistance(path);

    smoothedPath.resize(numPoints);

    // interpolate points on the smoothed path using the points in the original path 使用原始路径中的点在平滑的路径上插值点
    for (unsigned int i = 0; i < numPoints; i++)
    {
      double u = static_cast<double>(i) / (numPoints-1);
      interpolatePoint(path, cummulativeDistances, pose, u);

      if (std::isnan(pose.pose.position.x) || std::isnan(pose.pose.position.y))
        pose.pose = smoothedPath[std::max(static_cast<int>(i)-1, 0)].pose;
      smoothedPath[i] = pose;
    }

    // copy start and goal orientations to smoothed path 复制开始和目标方向到平滑的路径
    smoothedPath.front().pose.orientation = path.front().pose.orientation;
    smoothedPath.back().pose.orientation = path.back().pose.orientation;

    // interpolate orientations of intermediate poses 插值中间点的方向
    for (unsigned int i = 0; i < smoothedPath.size(); i++)
    {
      geometry_msgs::Quaternion orientation_target;  
      float dx = smoothedPath[i+1].pose.position.x - smoothedPath[i].pose.position.x;
      float dy = smoothedPath[i+1].pose.position.y - smoothedPath[i].pose.position.y;
      float th = atan2(dy, dx);
      tf::Quaternion q;
      q.setRPY(0, 0, th);
      q.normalize();
      orientation_target.x = q[0];
      orientation_target.y = q[1];
      orientation_target.z = q[2];
      orientation_target.w = q[3];
      smoothedPath[i].pose.orientation = orientation_target;
    }

    // revert skipPoints to original value 将跳转点还原为原始值
    skipPoints_ = oldSkipPoints;
  }

  //插入点
  void CubicSplineInterpolator::interpolatePoint(
    const std::vector<geometry_msgs::PoseStamped>& path,
    const std::vector<double>& cummulativeDistances,
    geometry_msgs::PoseStamped& point,
    double pointCummDist)
  {
    unsigned int group = findGroup(cummulativeDistances, pointCummDist);
    // ROS_INFO("u: %f, idx: %u", pointCummDist, group);

    double a = calcAlphaCoeff(path, cummulativeDistances, group, pointCummDist);
    double b = calcBetaCoeff(path, cummulativeDistances, group, pointCummDist);
    double c = calcGammaCoeff(path, cummulativeDistances, group, pointCummDist);
    double d = calcDeltaCoeff(path, cummulativeDistances, group, pointCummDist);

    std::vector<double> grad, nextGrad;
    calcPointGradient(path, cummulativeDistances, group, grad);
    calcPointGradient(path, cummulativeDistances, group+1, nextGrad);

    point.pose.position.x =
      + a * path[group*(skipPoints_+1)].pose.position.x
      + b * path[(group+1)*(skipPoints_+1)].pose.position.x
      + c * grad[0]
      + d * nextGrad[0];

    point.pose.position.y =
      + a * path[group*(skipPoints_+1)].pose.position.y
      + b * path[(group+1)*(skipPoints_+1)].pose.position.y
      + c * grad[1]
      + d * nextGrad[1];
  }

  //calc 累积距离
  void CubicSplineInterpolator::calcCummulativeDistances(
    const std::vector<geometry_msgs::PoseStamped> path,
    std::vector<double>& cummulativeDistances)
  {
    cummulativeDistances.clear();
    cummulativeDistances.push_back(0);

    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
      cummulativeDistances.push_back(
        cummulativeDistances.back()
        + calcDistance(path, i) / calcTotalDistance(path));
  }

  //calc总距离
  double CubicSplineInterpolator::calcTotalDistance(
    const std::vector<geometry_msgs::PoseStamped>& path)
  {
    double totalDist = 0;

    for (unsigned int i = skipPoints_+1; i < path.size(); i += skipPoints_+1)
      totalDist += calcDistance(path, i);

    return totalDist;
  }

  //calc距离
  double CubicSplineInterpolator::calcDistance(
    const std::vector<geometry_msgs::PoseStamped>& path,
    unsigned int idx)
  {
    if (idx <= 0 || idx >=path.size())
      return 0;

    double dist =
      hypot(
        path[idx].pose.position.x - path[idx-skipPoints_-1].pose.position.x,
        path[idx].pose.position.y - path[idx-skipPoints_-1].pose.position.y);

    return dist;
  }

  //calc α多项式系数
  double CubicSplineInterpolator::calcAlphaCoeff(
    const std::vector<geometry_msgs::PoseStamped> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double alpha =
      + 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
      - 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2)
      + 1;

    return alpha;
  }

  //calc β多项式系数
  double CubicSplineInterpolator::calcBetaCoeff(
    const std::vector<geometry_msgs::PoseStamped> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double beta =
      - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
      + 3 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2);

    return beta;
  }

  //calc伽马多项式系数
  double CubicSplineInterpolator::calcGammaCoeff(
    const std::vector<geometry_msgs::PoseStamped> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double gamma =
      (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
       - 2 * pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
      * (cummulativeDistances[idx+1] - cummulativeDistances[idx])
      + input
      - cummulativeDistances[idx];

    return gamma;
  }

  //calc三角多项式系数
  double CubicSplineInterpolator::calcDeltaCoeff(
    const std::vector<geometry_msgs::PoseStamped> path,
    const std::vector<double> cummulativeDistances,
    unsigned int idx,
    double input)
  {
    double delta =
      (pow(calcRelativeDistance(cummulativeDistances, idx, input), 3)
       - pow(calcRelativeDistance(cummulativeDistances, idx, input), 2))
      * (cummulativeDistances[idx+1] - cummulativeDistances[idx]);

      return delta;
  }

  //calc相对距离
  double CubicSplineInterpolator::calcRelativeDistance(
    const std::vector<double>& cummulativeDistances,
    const unsigned int idx,
    const double input)
  {
    double relDist =
      (input - cummulativeDistances[idx])
      / (cummulativeDistances[idx+1] - cummulativeDistances[idx]);
    return relDist;
  }

  //calc点梯度
  void CubicSplineInterpolator::calcPointGradient(
    const std::vector<geometry_msgs::PoseStamped>& path,
    const std::vector<double>& cummulativeDistances,
    unsigned int idx,
    std::vector<double>& gradient)
  {
    double dx, dy, du;
    gradient.assign(2, 0);

    // use either pose.yaw or interpolation to find gradient of points
    if ((useEndConditions_ && (idx == 0 || idx == cummulativeDistances.size()-1))
      || useMiddleConditions_)
    {
      double th = tf::getYaw(path[idx*(skipPoints_+1)].pose.orientation);
      int sign = (fabs(th) < M_PI / 2) ? 1 : -1;

      gradient[0] = sign * calcTotalDistance(path)
        * sqrt(1 + pow(tan(th),2)) / (1 + pow(tan(th), 2));
      gradient[1] = tan(th) * gradient[0];
    }
    else  // gradient interpolation using original points
    {
      if (idx == 0 || idx == cummulativeDistances.size()-1)
        return;

      dx = path[(idx)*(skipPoints_+1)].pose.position.x - path[(idx-1)*(skipPoints_+1)].pose.position.x;
      dy = path[(idx)*(skipPoints_+1)].pose.position.y - path[(idx-1)*(skipPoints_+1)].pose.position.y;
      du = cummulativeDistances[idx] - cummulativeDistances[idx-1];

      gradient[0] =  dx / du;
      gradient[1] =  dy / du;
    }
  }


  unsigned int CubicSplineInterpolator::findGroup(
    const std::vector<double>& cummulativeDistances,
    double pointCummDist)
  {
    unsigned int i;
    for (i = 0; i < cummulativeDistances.size()-1; i++)
    {
      if (pointCummDist <= cummulativeDistances[i+1])
        return i;
    }
    return i;
  }

  // namespace path_smoothing
