#pragma once

#include <Eigen/Core>
/*
    Eigen 是一个用于线性代数、矩阵运算和数值计算的 C++ 模板库，提供了丰富的类型来处理不同维度和数据类型的数学对象。
*/
namespace planning {
    typedef Eigen::Vector2d Vec2d;//2维双精度向量（double）
    typedef Eigen::Vector3d Vec3d;//3维双精度向量（double）
    typedef Eigen::VectorXd VecXd;//动态大小的双精度向量
    typedef Eigen::Vector2i Vec2i;//2维整形向量（int）
    typedef Eigen::Vector3i Vec3i;//3维整形向量（int）

    typedef Eigen::Array2i Arr2i;//2维整形数组
    typedef Eigen::Array3i Arr3i;//3维整形数组

    typedef Eigen::Matrix2d Mat2d;//2x2双精度矩阵
    typedef Eigen::Matrix3d Mat3d;//3x3双精度矩阵
    typedef Eigen::MatrixXd MatXd;//动态大小的双精度矩阵
    
}