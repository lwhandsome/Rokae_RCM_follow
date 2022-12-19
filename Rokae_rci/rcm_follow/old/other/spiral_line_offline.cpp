/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 螺旋线
 */

#include <cmath>
#include <iostream>
#include <fstream>
#include <functional>
#include <unistd.h>
#include <vector>
#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "move.h"
#include "model.h"
#include <ctime>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/SVD>

using namespace Eigen;

using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;

double distance(Vector3d p_1, Vector3d p_2, Vector3d p_RCM)
{
    Vector3d p12 = p_1 - p_2;
    Vector3d p1RCM = p_1 - p_RCM;
    Vector3d p_cross = p12.cross(p1RCM);
    double d = p_cross.norm() / p12.norm();
    return d;
}

// 利用Eigen库，采用SVD分解的方法求解矩阵伪逆，默认误差er为0
Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd &origin, const float er = 0)
{
    // 进行svd分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin,
                                                 Eigen::ComputeThinU |
                                                     Eigen::ComputeThinV);
    // 构建SVD分解结果
    Eigen::MatrixXd U = svd_holder.matrixU();
    Eigen::MatrixXd V = svd_holder.matrixV();
    Eigen::MatrixXd D = svd_holder.singularValues();

    // 构建S矩阵
    Eigen::MatrixXd S(V.cols(), U.cols());
    S.setZero();

    for (unsigned int i = 0; i < D.size(); ++i)
    {

        if (D(i, 0) > er)
        {
            S(i, i) = 1 / D(i, 0);
        }
        else
        {
            S(i, i) = 0;
        }
    }

    // pinv_matrix = V * S * U^T
    return V * S * U.transpose();
}


// std::array<double, 7> Get_joint_position_lamda(std::array<double, 7>q_last, Vector3d p_goal, xmate::Robot robot, XmateModel xmatemodel, double lamda)
// {
//     const double PI = 3.14159;
//     std::array<double, 7> q_init, q_init_plus, q_init_minus;
//     // std::array<double,7> q_drag = {{0,PI/6,PI/2,PI/2,PI/2,PI/2,PI/2}};
//     std::array<double, 7> q_drag = {{0, 0, 0, 0, 0, 0, 0}};
//     std::array<double, 42> jocobian1, jocobian2;
//     std::array<double, 16> pose1, pose2;
//     std::array<double, 16> pose1_plus, pose2_plus;
//     std::array<double, 16> pose1_minus, pose2_minus;
//     Vector3d p_RCM, p_1, p_1_plus, p_1_minus, p_2, p_2_plus, p_2_minus, error, last_error;
//     Vector4d p_goal_ext;
//     p_RCM << 0.56, 0, 0.35;
//     // p_goal << 0.3, 0.1, 0.6;
//     double d, d_plus, d_minus;
//     MatrixXd J_ext(4, 7);
//     MatrixXd J_ext_pinv;
//     MatrixXd delta_q;

//     std::array<double, 16> coord_1 = {{1, 0, 0, 0,
//                                        0, 1, 0, 0,
//                                        0, 0, 1, 0.28,
//                                        0, 0, 0, 1}};
//     std::array<double, 16> coord_2 = {{1, 0, 0, 0,
//                                        0, 1, 0, 0,
//                                        0, 0, 1, 0,
//                                        0, 0, 0, 1}};
//     double eps = 0.001;
//     std::array<double, 7> derivative;
//     q_init = robot.receiveRobotState().q;
//     //q_init = q_last;
//     last_error << 0.0, 0.0, 0.0;
//     while (1)
//     {
//         // q_init = robot.receiveRobotState().q;
//         jocobian1 = xmatemodel.Jacobian(q_init, coord_1, coord_2, SegmentFrame::kEndEffector);
//         // jocobian2 = xmatemodel.Jacobian(q_init, SegmentFrame::kJoint6);
//         pose1 = xmatemodel.GetCartPose(q_init, coord_1, coord_2, SegmentFrame::kEndEffector);
//         pose2 = xmatemodel.GetCartPose(q_init, SegmentFrame::kJoint7);
//         for (int i = 0; i < 3; i++)
//         {
//             p_1(i) = pose1.at(4 * i + 3);
//             p_2(i) = pose2.at(4 * i + 3);
//         }
//         // std::cout << q_init << std::endl;
//         // int i;
//         // std::cin >> i;
//         // std::cout << "p_1:" << p_1 << std::endl
//         //           << "    " << std::endl;
//         // std::cout << "p_2:" << p_2 << std::endl
//         //           << "    " << std::endl;
//         d = distance(p_1, p_2, p_RCM);
//         // std::cout << d << std::endl
//         //           << "    " << std::endl;
//         // Derivative of d
//         for (int i = 0; i < 7; i++)
//         {
//             q_init_plus = q_init;
//             q_init_plus.at(i) += eps;
//             q_init_minus = q_init;
//             q_init_minus.at(i) -= eps;
//             pose1_plus = xmatemodel.GetCartPose(q_init_plus, coord_1, coord_2, SegmentFrame::kEndEffector);
//             pose2_plus = xmatemodel.GetCartPose(q_init_plus, SegmentFrame::kJoint7);
//             pose1_minus = xmatemodel.GetCartPose(q_init_minus, coord_1, coord_2, SegmentFrame::kEndEffector);
//             pose2_minus = xmatemodel.GetCartPose(q_init_minus, SegmentFrame::kJoint7);
//             for (int j = 0; j < 3; j++)
//             {
//                 p_1_plus(j) = pose1_plus.at(4 * j + 3);
//                 p_1_minus(j) = pose1_minus.at(4 * j + 3);
//                 p_2_plus(j) = pose2_plus.at(4 * j + 3);
//                 p_2_minus(j) = pose2_minus.at(4 * j + 3);
//             }
//             d_plus = distance(p_1_plus, p_2_plus, p_RCM);
//             d_minus = distance(p_1_minus, p_2_minus, p_RCM);
//             derivative.at(i) = (d_plus - d_minus) / (2 * eps);
//         }

//         for (int i = 0; i < 3; i++)
//         {
//             for (int j = 0; j < 7; j++)
//             {
//                 J_ext(i, j) = jocobian1.at(7 * i + j);
//             }
//         }

//         for (int i = 3; i < 4; i++)
//         {
//             for (int j = 0; j < 7; j++)
//             {
//                 J_ext(i, j) = derivative.at(j);
//             }
//         }

//         J_ext_pinv = pinv_eigen_based(J_ext);

//         error = p_goal - p_1;
//         for (int i = 0; i < 3; i++)
//         {
//             p_goal_ext(i) = error(i) * 0.05;
//         }
//         last_error = error;
//         p_goal_ext(3) = (0 - d) * 0.1;
//         if (p_goal_ext.norm() < 0.0002)
//         {
//             // std::cout << "Motion Planning Over!" << std::endl;
//             break;
//         }

//         delta_q = J_ext_pinv * p_goal_ext;

//         // std::cout << p_goal_ext << std::endl
//         //           << "  " << std::endl;

//         for (int i = 0; i < 7; i++)
//         {
//             if (delta_q(i) > 0.2)
//             {
//                 q_drag.at(i) = q_init.at(i) + 0.2;
//             }
//             else if (delta_q(i) < -0.2)
//             {
//                 q_drag.at(i) = q_init.at(i) - 0.2;
//             }
//             else
//             {
//                 q_drag.at(i) = q_init.at(i) + delta_q(i);
//             }
//         }
//         // p1 = pose1.at();
//         q_init = q_drag;
//         // std::cout << q_drag << std::endl;
//     }
//     return q_init;
// }

std::array<double, 7> Get_joint_position(std::array<double, 7>q_last, Vector3d p_goal, xmate::Robot robot, XmateModel xmatemodel, double &d)
{
    const double PI = 3.14159;
    std::array<double, 7> q_init, q_init_plus, q_init_minus;
    // std::array<double,7> q_drag = {{0,PI/6,PI/2,PI/2,PI/2,PI/2,PI/2}};
    std::array<double, 7> q_drag = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 42> jocobian1, jocobian2;
    std::array<double, 16> pose1, pose2;
    std::array<double, 16> pose1_plus, pose2_plus;
    std::array<double, 16> pose1_minus, pose2_minus;
    Vector3d p_RCM, p_1, p_1_plus, p_1_minus, p_2, p_2_plus, p_2_minus, error, last_error;
    Vector4d p_goal_ext;
    p_RCM << 0.56, 0, 0.35;
    // p_goal << 0.3, 0.1, 0.6;
    double d_plus, d_minus;
    MatrixXd J_ext(4, 7);
    MatrixXd J_ext_pinv;
    MatrixXd delta_q;

    std::array<double, 16> coord_1 = {{1, 0, 0, 0,
                                       0, 1, 0, 0,
                                       0, 0, 1, 0.28,
                                       0, 0, 0, 1}};
    std::array<double, 16> coord_2 = {{1, 0, 0, 0,
                                       0, 1, 0, 0,
                                       0, 0, 1, 0,
                                       0, 0, 0, 1}};
    double eps = 0.001;
    std::array<double, 7> derivative;
    //q_init = robot.receiveRobotState().q;
    q_init = q_last;
    last_error << 0.0, 0.0, 0.0;
    while (1)
    {
        // q_init = robot.receiveRobotState().q;
        jocobian1 = xmatemodel.Jacobian(q_init, coord_1, coord_2, SegmentFrame::kEndEffector);
        // jocobian2 = xmatemodel.Jacobian(q_init, SegmentFrame::kJoint6);
        pose1 = xmatemodel.GetCartPose(q_init, coord_1, coord_2, SegmentFrame::kEndEffector);
        pose2 = xmatemodel.GetCartPose(q_init, SegmentFrame::kJoint7);
        for (int i = 0; i < 3; i++)
        {
            p_1(i) = pose1.at(4 * i + 3);
            p_2(i) = pose2.at(4 * i + 3);
        }
        // std::cout << q_init << std::endl;
        // int i;
        // std::cin >> i;
        // std::cout << "p_1:" << p_1 << std::endl
        //           << "    " << std::endl;
        // std::cout << "p_2:" << p_2 << std::endl
        //           << "    " << std::endl;
        d = distance(p_1, p_2, p_RCM);
        // std::cout << d << std::endl
        //           << "    " << std::endl;
        // Derivative of d
        for (int i = 0; i < 7; i++)
        {
            q_init_plus = q_init;
            q_init_plus.at(i) += eps;
            q_init_minus = q_init;
            q_init_minus.at(i) -= eps;
            pose1_plus = xmatemodel.GetCartPose(q_init_plus, coord_1, coord_2, SegmentFrame::kEndEffector);
            pose2_plus = xmatemodel.GetCartPose(q_init_plus, SegmentFrame::kJoint7);
            pose1_minus = xmatemodel.GetCartPose(q_init_minus, coord_1, coord_2, SegmentFrame::kEndEffector);
            pose2_minus = xmatemodel.GetCartPose(q_init_minus, SegmentFrame::kJoint7);
            for (int j = 0; j < 3; j++)
            {
                p_1_plus(j) = pose1_plus.at(4 * j + 3);
                p_1_minus(j) = pose1_minus.at(4 * j + 3);
                p_2_plus(j) = pose2_plus.at(4 * j + 3);
                p_2_minus(j) = pose2_minus.at(4 * j + 3);
            }
            d_plus = distance(p_1_plus, p_2_plus, p_RCM);
            d_minus = distance(p_1_minus, p_2_minus, p_RCM);
            derivative.at(i) = (d_plus - d_minus) / (2 * eps);
        }
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 7; j++)
            {
                J_ext(i, j) = jocobian1.at(7 * i + j);
            }
        }
        for (int i = 3; i < 4; i++)
        {
            for (int j = 0; j < 7; j++)
            {
                J_ext(i, j) = derivative.at(j);
            }
        }

        J_ext_pinv = pinv_eigen_based(J_ext);
        error = p_goal - p_1;
        for (int i = 0; i < 3; i++)
        {
            p_goal_ext(i) = error(i) * 0.8;
        }
        last_error = error;
        p_goal_ext(3) = (0 - d) * 0.08;
        if (p_goal_ext.norm() < 0.0001)
        {
            // std::cout << "Motion Planning Over!" << std::endl;
            break;
        }
        delta_q = J_ext_pinv * p_goal_ext;
        // std::cout << p_goal_ext << std::endl
        //           << "  " << std::endl;

        for (int i = 0; i < 7; i++)
        {
            if (delta_q(i) > 0.2)
            {
                q_drag.at(i) = q_init.at(i) + 0.2;
            }
            else if (delta_q(i) < -0.2)
            {
                q_drag.at(i) = q_init.at(i) - 0.2;
            }
            else
            {
                q_drag.at(i) = q_init.at(i) + delta_q(i);
            }
        }
        // p1 = pose1.at();
        q_init = q_drag;
        // std::cout << q_drag << std::endl;
    }
    return q_init;
}

Vector3d trajPlanning(double time, std::array<double, 3> k, Vector3d initPoint)
{
    double theta = k[0] * time;
    double r = k[1] * time;
    double high = k[2] * time;

    Vector3d goal;
    goal[0] = initPoint[0] + r * std::cos(theta);
    goal[1] = initPoint[1] + r * std::sin(theta);
    goal[2] = initPoint[2] - high;

    return goal;
}


int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.3.41";
    std::string name = "callback";
    uint16_t port = 1337;
    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port,XmateType::XMATE7_PRO);
    XmateModel xmatemodel(&robot, XmateType::XMATE7_PRO);
    //防止网络连接失败
    sleep(1);

    robot.setMotorPower(1);
    const double PI=3.14159;
    std::array<double,7> q_init, q_next, q_t;
    Vector3d p_init, p_next;
    double d;

    q_init = robot.receiveRobotState().q;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    MOVEJ(0.2,q_init,q_drag,robot);
    // std::cout << xmatemodel.GetCartPose(q_drag) << std::endl;

    p_init << 0.56, 0, 0.15;
    q_init = robot.receiveRobotState().q;
    q_next = Get_joint_position(q_init, p_init, robot, xmatemodel, d);
    q_next.at(6) = 0.000001;
    MOVEJ(0.1,q_init,q_next,robot);

    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    std::array<double, 7> init_position;
    std::array<double, 7> joint_delta;
    double time = 0;
    JointPositions output;
    std::array<double, 3> k {PI, 0.005, 0.01};

    // std::ofstream ofs;
    // ofs.open("data004.txt", std::ios::out);
    // if (!ofs)
    // {
    //     std::cout << "file open error!" << std::endl;
    // }

    std::array<std::array<double, 7>, 9000> q_array, q_now;
    Matrix<double, 4, 4> T;
    for (int i = 0; i < 4000; i++)
    {
        time += 0.001;
        q_init = q_next;
        p_next = trajPlanning(time, k, p_init);
        q_next = Get_joint_position(q_init, p_next, robot, xmatemodel, d); //计算下一个点的轴角度
        q_next.at(6) = 0.000001;
        
        T = Map<Matrix<double, 4, 4, RowMajor>>(xmatemodel.GetCartPose(q_next).data());
        T = T * MatrixXd{{1, 0, 0, 0},
                         {0, 1, 0, 0},
                         {0, 0, 1, 0.28},
                         {0, 0, 0, 1}};

        q_array[i] = q_next;

        std::cout << "time: " << time << std::endl;
        std::cout << "next position: " << std::endl << p_next << std::endl;
        std::cout << "joint angle: " << q_next << std::endl;
        // ofs << time << ' ' << p_next[0] << ' ' << p_next[1] << ' ' << p_next[2] << ' ' << T(0, 3) << ' ' << T(1, 3) << ' ' << T(2, 3) << ' ';
        // ofs << q_next[0] << ' ' << q_next[1] << ' ' << q_next[2] << ' ' << q_next[3] << ' ' << q_next[4] << ' ' << q_next[5] << ' ' << q_next[6] << ' ' << d << std::endl;
        
        for (int i = 0; i < 7; i++)
        {
            joint_delta[i] = q_next[i] - q_init[i];
            if (q_next.at(i) >= 2.0)
            {
                std::cout << "out of range!!!" << std::endl;    //关节角超限，直接退出回调
                return 0;
            }
        }
        std::cout << "-------------------" << std::endl;
    }
    std::cout << "calculate complete!" << std::endl;

    // ofs.close();

    sleep(1);

    int count = 0;
    
    std::string joint_callback = "joint_callback";
    JointControl joint_position_callback;

    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }
        
        q_t = q_array[count];

        // q_now[count] = robot_state.q;
        std::cout << count << std::endl;
        count++;

        if (count < 4000) {
            output = {{q_t[0], q_t[1], q_t[2], q_t[3], q_t[4], q_t[5], q_t[6]}};
        }
        else {
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }

        return output;        
    };

    robot.Control(joint_position_callback);
    
    // ofs.open("error11.txt", std::ios::out);
    // if (!ofs)
    // {
    //     std::cout << "file open error!" << std::endl;
    // }

    // for (int i = 0; i < 4000; i++)
    // {
    //     ofs << count << ' ' << q_now[i][0] << ' ' << q_now[i][1] << ' ' << q_now[i][2] << ' ' << q_now[i][3] << ' ' << q_now[i][4] << ' ' << q_now[i][5] << ' ' << q_now[i][6] << ' ' << std::endl;
    // }
    
    // ofs.close();
    robot.setMotorPower(0);

    return 0;
}
