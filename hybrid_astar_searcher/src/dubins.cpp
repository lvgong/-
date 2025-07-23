
#include "hybrid_astar_searcher/dubins.h"

#include <queue>
#include <boost/math/constants/constants.hpp>

namespace planning{

    inline double mod2pi(double x) {
        if (x < 0 && x > DUBINS_ZERO)
            return 0;
        double xm = x - twopi * floor(x / twopi);
        if (twopi - xm < .5 * DUBINS_EPS) xm = 0.;
        return xm;
    }

    DubinsStateSpace::DubinsPath DubinsStateSpace::dubinsLSL(double d, double alpha, double beta) {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sa - sb));
        if (tmp >= DUBINS_ZERO) {
            double theta = atan2(cb - ca, d + sa - sb);
            double t = mod2pi(-alpha + theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha + t) - sa + sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(dubinsPathType[0], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath DubinsStateSpace::dubinsRSR(double d, double alpha, double beta) {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = 2. + d * d - 2. * (ca * cb + sa * sb - d * (sb - sa));
        if (tmp >= DUBINS_ZERO) {
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta);
            double p = sqrt(std::max(tmp, 0.));
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha - t) + sa - sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(dubinsPathType[1], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath DubinsStateSpace::dubinsRSL(double d, double alpha, double beta) {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = d * d - 2. + 2. * (ca * cb + sa * sb - d * (sa + sb));
        if (tmp >= DUBINS_ZERO) {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(ca + cb, d - sa - sb) - atan2(2., p);
            double t = mod2pi(alpha - theta);
            double q = mod2pi(beta - theta);
            assert(fabs(p * cos(alpha - t) - 2. * sin(alpha - t) + sa + sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha - t) + 2. * cos(alpha - t) - ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(dubinsPathType[2], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath DubinsStateSpace::dubinsLSR(double d, double alpha, double beta) {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = -2. + d * d + 2. * (ca * cb + sa * sb + d * (sa + sb));
        if (tmp >= DUBINS_ZERO) {
            double p = sqrt(std::max(tmp, 0.));
            double theta = atan2(-ca - cb, d + sa + sb) - atan2(-2., p);
            double t = mod2pi(-alpha + theta);
            double q = mod2pi(-beta + theta);
            assert(fabs(p * cos(alpha + t) + 2. * sin(alpha + t) - sa - sb - d) < 2 * DUBINS_EPS);
            assert(fabs(p * sin(alpha + t) - 2. * cos(alpha + t) + ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(dubinsPathType[3], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath DubinsStateSpace::dubinsRLR(double d, double alpha, double beta) {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb + d * (sa - sb)));
        if (fabs(tmp) < 1.) {
            double p = twopi - acos(tmp);
            double theta = atan2(ca - cb, d - sa + sb);
            double t = mod2pi(alpha - theta + .5 * p);
            double q = mod2pi(alpha - beta - t + p);
            assert(fabs(2. * sin(alpha - t + p) - 2. * sin(alpha - t) - d + sa - sb) < 2 * DUBINS_EPS);
            assert(fabs(-2. * cos(alpha - t + p) + 2. * cos(alpha - t) - ca + cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha - t + p - q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(dubinsPathType[4], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }

    DubinsStateSpace::DubinsPath DubinsStateSpace::dubinsLRL(double d, double alpha, double beta) {
        double ca = cos(alpha), sa = sin(alpha), cb = cos(beta), sb = sin(beta);
        double tmp = .125 * (6. - d * d + 2. * (ca * cb + sa * sb - d * (sa - sb)));
        if (fabs(tmp) < 1.) {
            double p = twopi - acos(tmp);
            double theta = atan2(-ca + cb, d + sa - sb);
            double t = mod2pi(-alpha + theta + .5 * p);
            double q = mod2pi(beta - alpha - t + p);
            assert(fabs(-2. * sin(alpha + t - p) + 2. * sin(alpha + t) - d - sa + sb) < 2 * DUBINS_EPS);
            assert(fabs(2. * cos(alpha + t - p) - 2. * cos(alpha + t) + ca - cb) < 2 * DUBINS_EPS);
            assert(mod2pi(alpha + t - p + q - beta + .5 * DUBINS_EPS) < DUBINS_EPS);
            return DubinsStateSpace::DubinsPath(dubinsPathType[5], t, p, q);
        }
        return DubinsStateSpace::DubinsPath();
    }


    //生成最短路径的DB曲线
    DubinsStateSpace::DubinsPath DubinsStateSpace::dubins(double d, double alpha, double beta) {
        if (d < DUBINS_EPS && fabs(alpha - beta) < DUBINS_EPS)
            return DubinsStateSpace::DubinsPath(dubinsPathType[0], 0, d, 0);//直线路线

        // 选择长度最小的路径作为最优路径
        DubinsStateSpace::DubinsPath path(dubinsLSL(d, alpha, beta)), tmp(dubinsRSR(d, alpha, beta));
        double len, minLength = path.length();

        if ((len = tmp.length()) < minLength) {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRSL(d, alpha, beta);
        if ((len = tmp.length()) < minLength) {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLSR(d, alpha, beta);
        if ((len = tmp.length()) < minLength) {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsRLR(d, alpha, beta);
        if ((len = tmp.length()) < minLength) {
            minLength = len;
            path = tmp;
        }
        tmp = dubinsLRL(d, alpha, beta);
        if ((len = tmp.length()) < minLength)
            path = tmp;
        return path;
    }


    double DubinsStateSpace::distance(const std::shared_ptr<Node3d> start_node, const std::shared_ptr<Node3d> end_node) {
        return rho_ * GenerateDBP(start_node, end_node).length();
    }


    DubinsStateSpace::DubinsPath DubinsStateSpace::GenerateDBP(const std::shared_ptr<Node3d> start_node, 
                                const std::shared_ptr<Node3d> end_node) {
        double dx = end_node->getX() - start_node->getX();
        double dy = end_node->getY() - start_node->getY();
        double d = sqrt(dx * dx + dy * dy) / rho_, th = atan2(dy, dx);
        double dphi = mod2pi(start_node->getTheta() - th), beta = mod2pi(end_node->getTheta() - th);
        return dubins(d, dphi, beta);
    }


    //  插值，生成路点
    void DubinsStateSpace::interpolate(const std::shared_ptr<Node3d> start_node, DubinsPath &path, double seg, double s[3]) {

        if (seg < 0.0) seg = 0.0;
        if (seg > path.length()) seg = path.length();

        double phi, v;

        s[0] = s[1] = 0.0;
        s[2] = start_node->getTheta();

        for (unsigned int i = 0; i < 3 && seg > 0; ++i) {
            v = std::min(seg, path.length_[i]);
            seg -= v;
            phi = s[2];
            switch (path.type_[i]) {
                case DUBINS_LEFT:
                    s[0] += ( sin(phi+v) - sin(phi));
                    s[1] += (-cos(phi+v) + cos(phi));
                    s[2] = phi + v;
                    break;
                case DUBINS_RIGHT:
                    s[0] += (-sin(phi-v) + sin(phi));
                    s[1] += ( cos(phi-v) - cos(phi));
                    s[2] = phi - v;
                    break;
                case DUBINS_STRAIGHT:
                    s[0] += (v * cos(phi));
                    s[1] += (v * sin(phi));
                    break;
            }
        }

        s[0] = s[0] * rho_ + start_node->getX();
        s[1] = s[1] * rho_ + start_node->getY();
    }


    //生成最短DB曲线
    void DubinsStateSpace::ShortestDBP(const std::shared_ptr<Node3d> start_node,
                                    const std::shared_ptr<Node3d> end_node,
                                    DubinsPath_& optimal_path){
        // 计算起始点到目标点的最短Dubins路径
        DubinsPath path = GenerateDBP(start_node, end_node);

        // 路径的实际长度
        double length = rho_ * path.length();
        optimal_path.total_length = length;

        std::vector<std::vector<double> > db_path;
        // 通过循环从路径的起始点到终点按照步长step_size进行等间隔采样
        // length+step_size确保涵盖终点
        for (double seg=0.0; seg<=(length+step_size); seg+=step_size){
            double qnew[3] = {};
            // 在每个采样点上，调用interpolate(q0, path, seg/rho_, qnew)函数进行插值，计算出对应于采样点位置的状态qnew
            interpolate(start_node, path, seg/rho_, qnew);
            // 通过sizeof qnew / sizeof qnew[0]可以计算出数组qnew中的元素个数。然后，将数组qnew的起始地址作为第一个参数，结束地址作为第二个参数传递给std::vector的构造函数，即v(qnew, qnew + sizeof qnew / sizeof qnew[0])。这样就创建了一个包含了qnew数组元素的std::vector<double>向量v
            std::vector<double> v(qnew, qnew + sizeof qnew / sizeof qnew[0]);
            db_path.push_back(v);
        }

        for (auto &point_itr : db_path) {
            optimal_path.x.push_back(point_itr[0]);
            optimal_path.y.push_back(point_itr[1]);
              
            if (point_itr[2] >  M_PI)  point_itr[2] -= 2 *M_PI;
            if (point_itr[2] < - M_PI)  point_itr[2] += 2 * M_PI;
    
            optimal_path.phi.push_back(point_itr[2]);
        }
        
        // 检查最优路径的终点是否与目标节点位置和航向角相一致
        // if (std::abs(optimal_path.x.back() -
        //             end_node->getX()) > 1e-3 ||
        //     std::abs(optimal_path.y.back() -
        //             end_node->getY()) > 1e-3 ||
        //     std::fmod(std::abs(optimal_path.phi.back() -
        //                         end_node->getTheta()),
        //                 2 * M_PI) > 1e-3) {
        //     return false;
        // }

        return;
    }


    //生成最短DB曲线
    void DubinsStateSpace::ShortestDBP_(const std::shared_ptr<Node3d> start_node,
                                    const std::shared_ptr<Node3d> end_node,
                                    DubinsPath_& optimal_path){
        // 计算起始点到目标点的最短Dubins路径
        DubinsPath path = GenerateDBP(start_node, end_node);

        // 路径的实际长度
        double length = rho_ * path.length();
        optimal_path.total_length = length;

        return;
    }


}
