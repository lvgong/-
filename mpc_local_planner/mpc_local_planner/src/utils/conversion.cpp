/*********************************************************************
 *
 *  Software License Agreement
 *
 *  Copyright (c) 2020, Christoph Rösmann, All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 *  Authors: Christoph Rösmann
 *********************************************************************/

#include <mpc_local_planner/utils/conversion.h>

#include <tf/tf.h>

namespace mpc_local_planner {

/*用于把时间序列转化成pose数组。
此处的时间序列，指的是control_box_rst中corbo中的类TimeSeries；
pose数组，指的是std::vector< geometry_msgs::PoseStamped > 。
主要包含了函数convert，输入time_series、dynamics（机器人动力学接口，systems部分会详细介绍）、frame_id，输出pose_stamped。*/
void convert(const corbo::TimeSeries& time_series, const RobotDynamicsInterface& dynamics, std::vector<geometry_msgs::PoseStamped>& poses_stamped,
             const std::string& frame_id)
{
    poses_stamped.clear();

    if (time_series.isEmpty()) return;

    for (int i = 0; i < time_series.getTimeDimension(); ++i)
    {
        poses_stamped.emplace_back();

        double theta = 0;
        // 函数getPoseSE2FromState作用就是把状态向量转化成三个状态量，后边再把pose的header填上去。
        dynamics.getPoseSE2FromState(time_series.getValuesMap(i), poses_stamped.back().pose.position.x, poses_stamped.back().pose.position.y, theta);
        poses_stamped.back().pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        poses_stamped.back().header.frame_id  = frame_id;
        poses_stamped.back().header.stamp     = ros::Time::now();  // TODO(roesmann) should we use now()?
    }
}

}  // namespace mpc_local_planner
