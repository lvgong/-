#include "MPC.hpp"
#include "math.h"
#include "iostream"

// #define PI 3.1415926
#define T 0.25  //控制周期
// #define w0 1.0  //权重
// #define w1 0.5
#define Ku 1
#define Kl 1

using namespace Eigen;
using namespace std;
USING_NAMESPACE_QPOASES


//MPC核心步骤
MatrixXd MPC_controller::MPC_Solve_qp(Eigen::Vector3d X_k,std::vector<Eigen::Vector3d >X_r,std::vector<Eigen::Vector2d >U_r,const int N)
{
    //cout<<"current w : "<<w_max<<endl;
    ////根据参考输入计算出的系数矩阵
    //  都有 N 个 MatrixXd 对象
    vector<MatrixXd> A_r(N),B_r(N),A_multiply1(N);
    MatrixXd O_r(3*N,1);//
    MatrixXd A_bar(3*N,3);//
    MatrixXd X_ref(3*N,1);
    MatrixXd A_multiply2;
    MatrixXd B_bar = MatrixXd::Zero(3*N,2*N);//初始化为3N*2N的0矩阵
    MatrixXd C_bar = MatrixXd::Identity(3*N,3*N);//
    MatrixXd A_r_init = MatrixXd::Zero(3,3);//初始化为3*3的0矩阵
    MatrixXd B_r_init = MatrixXd::Zero(3,2);
    MatrixXd eye_3 = MatrixXd::Identity(3,3);//单位阵
    MatrixXd Q = MatrixXd::Identity(3*N,3*N)*omega0;//对角权重矩阵，用于预测误差在优化时的比重
    MatrixXd R = MatrixXd::Identity(2*N,2*N)*omega1;//对角权重矩阵，用于运动控制在优化时的比重
//    cout<<"Ur 1 : "<<endl<<U_r[0]<<endl;
//    cout<<"Xr 1 : "<<endl<<X_r[0]<<endl;
//    cout<<"Xk : "<<endl<<X_k<<endl;
    for(int k=0;k<N;k++)//未来 N 个周期
    {
        A_r[k] = A_r_init;//A_r向量中的每个元素都初始化为3*3的0矩阵
        B_r[k] = B_r_init;
        //用到了差速模型
        //A
        A_r[k](0,2) = -U_r[k](0)*sin(X_r[k](2));//X_r：参考状态  U_r：参考输入
        A_r[k](1,2) = U_r[k](0)*cos(X_r[k](2));
        //O
        Vector3d temp_vec = -T*A_r[k]*X_r[k];
        O_r.block<3,1>(k*3,0) = temp_vec;

        A_r[k] = eye_3+T*A_r[k];//A_hat
        //B
        B_r[k](0,0) = cos(X_r[k](2))*T;
        B_r[k](1,0) = sin(X_r[k](2))*T;
        B_r[k](2,1) = T;
        X_ref.block<3,1>(k*3,0) = X_r[k];
//        cout<<"B_r "<<k+1<<" : "<<endl<<B_r[k]<<endl;

        if(k==0) A_multiply1[k] = A_r[k];
        else A_multiply1[k] = A_multiply1[k-1]*A_r[k];//A_multiply1[k]：关于A的连乘积
        //A_bar
        A_bar.block<3,3>(3*k,0) = A_multiply1[k];
    }
//    cout<<"Ar 1 : "<<endl<<A_r[0]<<endl;
//    cout<<"Br 1 : "<<endl<<B_r[0]<<endl;
//    cout<<"Or 1 : "<<endl<<O_r.block<3,1>(0,0)<<endl;
//    cout<<"O_r : "<<endl<<O_r<<endl;

    for(int k=0;k<N;k++)
    {
        // 将向量 B_r[k] 的值复制到矩阵 B_bar 的特定块中
        // block<3,2>(3*k,2*k) 表示从矩阵 B_bar 中选择一个大小为 3x2 的块，起始于行索引 3*k 和列索引 2*k
        B_bar.block<3,2>(3*k,2*k) = B_r[k];
        A_multiply2 = eye_3;//初始化为单位阵 3*3
        for(int i=0;i<k;i++)
        {
            A_multiply2 = A_multiply2*A_r[k-i];
            C_bar.block<3,3>(3*k,3*(k-1-i)) = A_multiply2;
            //B_bar最后一行块
            B_bar.block<3,2>(3*k,2*(k-1-i)) = A_multiply2*B_r[k-1-i];
        }
    }

    //cout<<"C bar : "<<endl<<C_bar<<endl;

    //minJ有关系数
    MatrixXd E =A_bar*X_k+C_bar*O_r-X_ref;//X_k：当前状态
    MatrixXd Hesse = 2*(B_bar.transpose()*Q*B_bar+R);   //Hesse矩阵
    VectorXd gradient = 2*B_bar.transpose()*Q*E;   //一次项系数

    real_t H[2*N*2*N],g[2*N],A[2*N],lb[2*N],ub[2*N],lbA[1],ubA[1];
    lbA[0] = N*(v_min+w_min)/Ku;
    ubA[0] = N*(v_max+w_max)/Kl;
    for(int i=0;i<2*N;i++)
    {
        g[i] = gradient(i);
        A[i] = 1;
        if(i%2==0)
        {
            lb[i] = v_min;//硬约束 v
            ub[i] = v_max;
        }
        else
        {
            lb[i] = w_min;//w
            ub[i] = w_max;
        }
        for(int j=0;j<2*N;j++)
        {
            H[i*2*N+j] = Hesse(i,j);
        }
    }

    int_t nWSR = 800;

    QProblem mpc_qp_solver(2*N,1);
    mpc_qp_solver.init(H,g,A,lb,ub,lbA,ubA,nWSR);

    real_t x_solution[2*N];
    mpc_qp_solver.getPrimalSolution(x_solution);

    Vector2d u_k;
    MatrixXd U_result = MatrixXd::Zero(2,N);
    for(int i=0;i<N;i++)
    {
        u_k(0) = x_solution[2*i];//v线速度
        u_k(1) = x_solution[2*i+1];//w角速度
        U_result.col(i) = u_k;
//        std::cout<<"U "<<i+1<<" : "<<endl<<u_k<<endl;
    }
    //cout<<"N : "<<N<<endl;
    return U_result;
}


void MPC_controller::MPC_init(ros::NodeHandle &nh)
{
    nh.getParam("/chibot_hybrid_astar_dubins_opt/MPC/v_max", v_max);
    nh.getParam("/chibot_hybrid_astar_dubins_opt/MPC/w_max", w_max);
    nh.getParam("/chibot_hybrid_astar_dubins_opt/MPC/omega0", omega0);
    nh.getParam("/chibot_hybrid_astar_dubins_opt/MPC/omega1", omega1);

    // nh.param("/ego_planner_node/MPC/v_max",v_max,0.7);
    // nh.param("/ego_planner_node/MPC/w_max",w_max,0.3);
    ROS_INFO("v_max: %f , w_max: %f",v_max,w_max);
    w_min = -w_max;
    v_min=0;
}
