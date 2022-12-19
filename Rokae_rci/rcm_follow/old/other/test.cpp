/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * 开启拖动示例
 */

#include <cmath>
#include <iostream>
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
#include "network1.h"
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

std::array<double, 7> Get_joint_position(std::array<double, 7>q_last, Vector3d p_goal, xmate::Robot robot, XmateModel xmatemodel)
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
    p_RCM << 0.6, 0, 0.3;
    // p_goal << 0.3, 0.1, 0.6;
    double d, d_plus, d_minus;
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
            p_goal_ext(i) = error(i) * 0.05;
        }
        last_error = error;
        p_goal_ext(3) = (0 - d) * 0.1;
        if (p_goal_ext.norm() < 0.0002)
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

void Joint_motion_control(std::vector<std::array<double, 7>> joint_motion_vector, xmate::Robot robot, XmateModel xmatemodel)
{
    std::array<double, 7> init_position, delta_position, q_init;
    static bool init = true;
    double time = 0;

    q_init = robot.receiveRobotState().q;
    MOVEJ(1,q_init,joint_motion_vector[0],robot);

    robot.setJointImpedance({{1000, 1000, 1000, 1000, 100, 100, 100}});
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointImpedance, RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    JointControl joint_position_callback;

    int count=0;
    int total_step;
    total_step = joint_motion_vector.size();

    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }

        //double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

        JointPositions output = {{joint_motion_vector[count].at(0), joint_motion_vector[count].at(1),
                                  joint_motion_vector[count].at(2), joint_motion_vector[count].at(3),
                                  joint_motion_vector[count].at(4), joint_motion_vector[count].at(5),
                                  joint_motion_vector[count].at(6)}}; 
        time += 0.001; 
        count++;
        if(count>=total_step)
        {
            std::cout<<"运动结束count"<<std::endl;
            return MotionFinished(output);
        }
        if(time>=2){
            time = 0;
            std::cout<<"运动结束time"<<std::endl;
            return MotionFinished(output);
        }
        //std::cout << delta_angle << std::endl;
        return output;        
    };

    robot.Control(joint_position_callback);
}

Vector3d Get_now_position(xmate::Robot robot, XmateModel xmatemodel)
{
    std::array<double, 16> coord_1 = {{1, 0, 0, 0,
                                       0, 1, 0, 0,
                                       0, 0, 1, 0.28,
                                       0, 0, 0, 1}};
    std::array<double, 16> coord_2 = {{0.86603, 0, -0.5, 0,
                                       0, 1, 0, 0,
                                       0.5, 0, 0.86603, 0,
                                       0, 0, 0, 1}};
    std::array<double, 16> pose1;
    std::array<double, 7> q_init;
    q_init = robot.receiveRobotState().q;
    Vector3d now_position;
    pose1 = xmatemodel.GetCartPose(q_init, coord_1, coord_2, SegmentFrame::kEndEffector);
    for (int i = 0; i < 3; i++)
    {
        now_position(i) = pose1.at(4 * i + 3);
    }
    return now_position;
}
Vector3d Get_next_position(Vector3d new_position, xmate::Robot robot, XmateModel xmatemodel)
{
    std::array<double, 16> coord_1 = {{1, 0, 0, 0,
                                       0, 1, 0, 0,
                                       0, 0, 1, 0.28,
                                       0, 0, 0, 1}};
    std::array<double, 16> coord_2 = {{0.86603, 0, -0.5, 0,
                                       0, 1, 0, 0,
                                       0.5, 0, 0.86603, 0,
                                       0, 0, 0, 1}};
    std::array<double, 16> pose1;
    std::array<double, 7> q_init;
    q_init = robot.receiveRobotState().q;
    Vector3d last_position, next_position;
    pose1 = xmatemodel.GetCartPose(q_init, coord_1, coord_2, SegmentFrame::kEndEffector);
    MatrixXd transform(3, 3);
    for (int i = 0; i < 3; i++)
    {
        last_position(i) = pose1.at(4 * i + 3);
        transform(i, 0) = pose1.at(4 * i);
        transform(i, 1) = pose1.at(4 * i + 1);
        transform(i, 2) = pose1.at(4 * i + 2);
    }
    next_position = transform * new_position + last_position;
    return next_position;
}

int main(int argc, char *argv[])
{

    std::string ipaddr = "192.168.3.41";
    uint16_t port = 1337;

    std::string file = "../../xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file))
    {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    const double PI=3.14159;
    // RCI连接机器人
    xmate::Robot robot(ipaddr, port, XmateType::XMATE7_PRO);
    XmateModel xmatemodel(&robot, XmateType::XMATE7_PRO);
    //防止网络连接失败
    sleep(2);
    int res = robot.getMotorState();
    std::cout << "机器人上电状态：" << res << std::endl;
    int power_state = 1;
    robot.setMotorPower(power_state);

    std::array<double,7> q_init, q_t, q_last;
    std::vector<std::array<double, 7>> joint_motion_vector;
    double x_t, y_t, h_t, theta_t, t=0;

    p_t << 0.65, 0, 0.213;
    q_init = robot.receiveRobotState().q;
    q_t = Get_joint_position(q_init, p_t, robot, xmatemodel);
    q_t.at(6) = 0.000001;
    MOVEJ(0.5,q_init,q_t,robot);

    CtrlParam param;
    Recver rec;
    Vector3d next_position, new_vector, now_position;
    if (!rec.setup())
    {
        cout << "FAIL to setup socket!" << endl;
        return 0;
    }
    int out_of_range_flag = 0;
    
    robot.setJointImpedance({{1000, 1000, 1000, 1000, 100, 100, 100}});
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointImpedance, RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    JointControl joint_position_callback;
    std::array<double, 7> joint_delta;

    // int count=0;
    // int total_step;
    // total_step = joint_motion_vector.size();

    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        
        if(init==true){
            init_position = robot_state.q;
            init=false;
        }
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }

        //接受相机信息！！！！！！！！！！！！！！！！！！！
        if(!rec.getvec(param))
        {
            std::cout << "VectorRec ERROR!" << std::endl;
            rec.reset();
            rec.setup();
            break;
        }
        else
        {
            new_vector(0) = param.y;
            new_vector(1) = -param.x;
            new_vector(2) = 0.0;
        }

        //根据相机信息计算下一个点！！！！！！！！！
        out_of_range_flag = 0;
        next_position = Get_next_position(new_vector, robot, xmatemodel);
        now_position = Get_now_position(robot, xmatemodel); //计算当前点坐标
        std::cout << "next position:" << std::endl << next_position << std::endl;
        std::cout << "new vector:" << std::endl << new_vector << std::endl;
        std::cout << "-------------------"<< std::endl;
        q_init = robot.receiveRobotState().q;
        q_t = Get_joint_position(q_init, next_position, robot, xmatemodel); //计算下一个点的轴角度
        q_t.at(6) = 0.000001;
        for(int i=0; i<7; i++)
        {
            joint_delta[i] = (q_t[i] - q_init[i]) / 100;
            if(q_t.at(i)>=2.0)
            {
                std::cout << "out of range!!!" << std::endl;    //关节角超限，直接退出回调
                return MotionFinished(output);
            }
        }
        
        //double delta_angle = M_PI / 20.0 * (1 - std::cos(M_PI/4 * time));

        JointPositions output = {{init_position[0] + joint_delta[0], init_position[1] + joint_delta[1],
                                  init_position[2] + joint_delta[2], init_position[3] + joint_delta[3],
                                  init_position[4] + joint_delta[4], init_position[5] + joint_delta[5],
                                  init_position[6] + joint_delta[6]}}; 
        time += 0.001; 
        if(time>=20){
            time = 0;
            std::cout<<"时间超限，运动结束"<<std::endl;
            return MotionFinished(output);
        }
        //std::cout << delta_angle << std::endl;
        return output;        
    };

    robot.Control(joint_position_callback);

    return 0;
}
