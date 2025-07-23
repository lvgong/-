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

#ifndef QUADRATIC_COST_SE2_H_
#define QUADRATIC_COST_SE2_H_

#include <corbo-optimal-control/functions/quadratic_cost.h>
#include <corbo-optimal-control/functions/quadratic_state_cost.h>

#include <cmath>
#include <memory>

namespace mpc_local_planner {

/**
 * @brief Quadratic form running cost (specialized for SE2)
 *
 * This class implements quadratic form running costs
 * \f[
 *    l(x, u) = (x_{ref} - x)^T Q (x_{xref} - x) + (u_{ref} - u)^T R (u_{ref} - u)
 * \f]
 * However, this class ensures that the state distance (x_{ref} - x) is
 * computed properly in SO(2) for the third state component.
 *
 * Note, the class also provides a least squares form if not in integral form which is:
 * \f$ l_{lsq} = \begin{bmatrix} \sqrt{Q} (x_{ref} - x) \\ \sqrt{R} (u_{ref} - u) \end{bmatrix} \f$
 * \f$ \sqrt{Q} \f$  \f$ \sqrt{R} \f$ are computed once using the cholesky decomposition.
 * This form is later squared by the solver.
 *
 * @see corbo::QuadraticFinalStateCost
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
/*QuadraticFormCostSE2 是一个二次形式的代价函数，用于计算在最优控制问题中每个阶段的代价。它是针对在平面上的二维运动系统（SE(2)）设计的。
这个代价函数的计算方式如下：
J = x'Qx + u'Ru
其中，x 是状态向量，u 是控制向量，Q 是状态权重矩阵，R 是控制权重矩阵。x' 表示 x 的转置。
代价函数的目标是最小化 J，也就是尽量减小状态和控制的加权和。状态权重矩阵 Q 用于定义状态变量在代价函数中的重要性，控制权重矩阵 R 用于定义控制变量在代价函数中的重要性。
在实际应用中，通过调整状态权重和控制权重的大小，可以控制不同变量对最优控制问题的影响程度。较大的权重值会使得对应的变量在代价函数中的影响更大，而较小的权重值则会降低其影响。
QuadraticFormCostSE2 是一种常用的代价函数形式，在许多最优控制问题中都可以使用。它能够灵活地建模各种目标，例如最小化位移、速度、能耗等。通过调整权重矩阵 Q 和 R，可以根据具体问题的需求来定义代价函数，从而得到满足特定目标的最优控制策略。*/
class QuadraticFormCostSE2 : public corbo::QuadraticFormCost
{
 public:
    using Ptr = std::shared_ptr<QuadraticFormCostSE2>;

    //! Default constructor
    QuadraticFormCostSE2() : corbo::QuadraticFormCost() {}

    /**
     * @brief Construct with weight matrices
     *
     * @param[in] Q                Positive definite state weighting matrix (must match dimensions!)
     * @param[in] R                Positive definite control weighting matrix (must match dimensions!)
     * @param[in] integral_form    If true, the running costs are later integrated over the grid interval,
     *                             otherwise, the values are just summed up as in sampled-data MPC.
     * @param[in] lsq_form         Set to true in order to prefer the least-squares form
     */
    QuadraticFormCostSE2(const Eigen::Ref<const Eigen::MatrixXd>& Q, const Eigen::Ref<const Eigen::MatrixXd>& R, bool integral_form = false,
                         bool lsq_form = false)
        : corbo::QuadraticFormCost(Q, R, integral_form, lsq_form)
    {
    }

    // implements interface method
    corbo::StageCost::Ptr getInstance() const override { return std::make_shared<QuadraticFormCostSE2>(); }

    // implements interface method
    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    // implements interface method
    void computeIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                         Eigen::Ref<Eigen::VectorXd> cost) const override;
};

/**
 * @brief Quadratic state running cost (specialized for SE2)
 *
 * This class implements quadratic state running costs
 * \f[
 *    l(x) = (x_{ref} - x)^T Q (x_{xref} - x)
 * \f]
 * However, this class ensures that the state distance (x_{ref} - x) is
 * computed properly in SO(2) for the third state component.
 *
 * Note, the class also provides a least squares form if not in integral form which is:
 * \f$ l_{lsq} = \sqrt{Q} (x_{ref} - x) \f$
 * \f$ \sqrt{Q} \f$ is computed once using the cholesky decomposition.
 * This form is later squared by the solver.
 *
 * @see corbo::QuadraticFinalStateCost
 *
 * @author Christoph Rösmann (christoph.roesmann@tu-dortmund.de)
 */
class QuadraticStateCostSE2 : public corbo::QuadraticStateCost
{
 public:
    using Ptr = std::shared_ptr<QuadraticStateCostSE2>;

    //! Default constructor
    QuadraticStateCostSE2() : corbo::QuadraticStateCost() {}

    /**
     * @brief Construct with weight matrices
     *
     * @param[in] Q                Positive definite state weighting matrix (must match dimensions!)
     * @param[in] integral_form    If true, the running costs are later integrated over the grid interval,
     *                             otherwise, the values are just summed up as in sampled-data MPC.
     * @param[in] lsq_form         Set to true in order to prefer the least-squares form
     */
    QuadraticStateCostSE2(const Eigen::Ref<const Eigen::MatrixXd>& Q, bool integral_form = false, bool lsq_form = false)
        : corbo::QuadraticStateCost(Q, integral_form, lsq_form)
    {
    }

    // implements interface method
    corbo::StageCost::Ptr getInstance() const override { return std::make_shared<QuadraticStateCostSE2>(); }

    // implements interface method
    void computeNonIntegralStateTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, Eigen::Ref<Eigen::VectorXd> cost) const override;

    // implements interface method
    void computeIntegralStateControlTerm(int k, const Eigen::Ref<const Eigen::VectorXd>& x_k, const Eigen::Ref<const Eigen::VectorXd>& u_k,
                                         Eigen::Ref<Eigen::VectorXd> cost) const override;
};

}  // namespace mpc_local_planner

#endif  // QUADRATIC_COST_SE2_H_
