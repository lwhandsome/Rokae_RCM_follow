/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */

/**
 * read_load
 */

#include <cmath>
#include <iostream>
#include <fstream>
#include <functional>
#include <Eigen/Dense>

#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "unistd.h"
#include "move.h"
#include "model.h"

using namespace Eigen;
using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1338;

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
    int power_state=1;
    robot.setMotorPower(power_state);
    std::cout<<"机器人上电状态："<<robot.getMotorState()<<std::endl;

    std::ofstream ofs;
    ofs.open("data_tau.txt", std::ios::out);
    if (!ofs)
    {
        std::cout << "file open error!" << std::endl;
    }

    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,0,0,0,0,0,0}};
    std::array<double,7> tau, tau_full, tau_inertial, tau_coriolis, tau_friction, tau_gravity;
    std::array<double,7> dq = {{0,0,0,0,0,0,0}};
    std::array<double,7> ddq = {{0,0,0,0,0,0,0}};
    std::array<double,42> jaco;
    VectorXd tau_diff, force;
    Matrix<double, 6, 7> J;
    for (int i = 9; i < 10; i++)
    {
        q_drag[4] = 0;
        for (int j = 9; j < 10; j++)
        {
            q_init = robot.receiveRobotState().q;
            q_drag[5] = 0;
            MOVEJ(0.2,q_init,q_drag,robot);
            sleep(0.5);
            tau = robot.receiveRobotState().tau_m;
            xmatemodel.GetTauNoFriction(q_drag, dq, ddq, tau_full, tau_inertial, tau_coriolis, tau_gravity);

            jaco = xmatemodel.Jacobian(q_init, SegmentFrame::kJoint7);

            tau_diff = Map<Matrix<double, 7, 1>>(tau.data()) - Map<Matrix<double, 7, 1>>(tau_full.data());
            J = Map<Matrix<double, 6, 7, RowMajor>>(jaco.data());

            force = J * (J.transpose() * J).inverse() * tau_diff;
            std::cout << force << std::endl;
            // ofs << robot.receiveRobotState().q << tau << tau_gravity << std::endl;
        }
    }

    ofs.close();

    // q_init = robot.receiveRobotState().q;
    // q_drag = {{0,0,0,0,0,0,0}};
    // MOVEJ(0.2,q_init,q_drag,robot);
    // sleep(0.5);
    // tau = robot.receiveRobotState().tau_m;
    // xmatemodel.GetTauNoFriction(q_drag, dq, ddq, tau_full, tau_inertial, tau_coriolis, tau_gravity);

    // jaco = xmatemodel.Jacobian(q_init, SegmentFrame::kJoint7);

    // tau_diff = Map<Matrix<double, 7, 1>>(tau.data()) - Map<Matrix<double, 7, 1>>(tkJoint7au_full.data());
    // J = Map<Matrix<double, 6, 7, RowMajor>>(jaco.data());

    // force = J * (J.transpose() * J).inverse() * tau_diff;
    // std::cout << force << std::endl;

    // tau = robot.receiveRobotState().tau_m;
    std::cout << "tau_read: " << tau <<std::endl;

    // tau_full = xmatemodel.GetTau(q_drag, dq, ddq, TauType::TAU_FULL);
    std::cout << "tau_full: " << tau_full <<std::endl;

    // tau_inertial = xmatemodel.GetTau(q_drag, dq, ddq, TauType::TAU_INERTIAL);
    std::cout << "tau_inertial: " << tau_inertial <<std::endl;

    // tau_coriolis = xmatemodel.GetTau(q_drag, dq, ddq, TauType::TAU_CORIOLIS);
    std::cout << "tau_coriolis: " << tau_coriolis <<std::endl;

    // tau_friction = xmatemodel.GetTau(q_drag, dq, ddq, TauType::TAU_FRICTION);
    // std::cout << "tau_friction: " << tau_friction <<std::endl;

    // tau_gravity = xmatemodel.GetTau(q_drag, dq, ddq, TauType::TAU_GRAVITY);
    std::cout << "tau_gravity: " << tau_gravity <<std::endl;

    robot.setMotorPower(0);
    std::cout<<"机器人上电状态："<<robot.getMotorState()<<std::endl;

    return 0;
}
