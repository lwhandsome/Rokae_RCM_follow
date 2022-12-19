/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <unistd.h>
#include <Eigen/Dense>

#include <duration.h>
#include <xmate_exception.h>
#include <model.h>
#include <robot.h>

#include "ini.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "print_rci.h"
#include "move.h"

/**
 * @直接力矩控制，笛卡尔空间的阻抗控制，运行demo的过程中，请确保手持急停
 */
using namespace xmate;
using TorqueControl = std::function<Torques(RCI::robot::RobotState robot_state)>;
int main(int argc, char *argv[])
{
    // Check whether the required arguments were passed
    std::string ipaddr = "192.168.0.160";
    uint16_t port = 1337;

    std::string file = "../..xmate.ini";
    INIParser ini;
    if (ini.ReadINI(file))
    {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }

    xmate::Robot robot(ipaddr, port,XmateType::XMATE3_PRO);
    sleep(1);
    robot.setMotorPower(1);

    const double PI=3.14159;
    std::array<double,7> q_init;
    std::array<double,7> q_drag = {{0,PI/6,0,PI/3,0,PI/2,0}};
    q_init = robot.receiveRobotState().q;
    MOVEJ(0.2,q_init,q_drag,robot);

    xmate::XmateModel model(&robot,xmate::XmateType::XMATE3_PRO);

    //robot.reg();
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);

    // Compliance parameters
    const Eigen::DiagonalMatrix<double, 7, 7> Bd(0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2);
    const Eigen::DiagonalMatrix<double, 7, 7> Kd(100, 500, 100, 100, 100, 100, 100);

    Eigen::Matrix<double, 7, 1> q_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(q_drag.data());

    Eigen::VectorXd tau_d(7);
    std::array<double, 7> tau_d_array;

    std::array<double, 7> tau_full;
    std::array<double, 7> tau_inertial;
    std::array<double, 7> tau_coriolis;
    std::array<double, 7> tau_friction;
    std::array<double, 7> tau_gravity;

    // RCI::robot::RobotState robot_state = robot.receiveRobotState();

    // model.GetTauWithFriction(robot_state.q, robot_state.dq_m, robot_state.ddq_c, tau_full,
    //                              tau_inertial, tau_coriolis, tau_friction, tau_gravity);

    // // PD+ control
    // // tau_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_coriolis.data())
    // //         + Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_friction.data())
    // //         + Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_gravity.data())
    // //         - Bd * Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq_m.data())
    // //         - Kd * (q_d - Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data()));
    // tau_d = Eigen::MatrixXd::Zero(7, 1)
    //         - Bd * Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq_m.data())
    //         - Kd * (q_d - Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data()));

    
    // std::cout << Bd * Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq_m.data()) << std::endl << std::endl;
    // std::cout << Kd * (q_d - Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data())) << std::endl << std::endl;

    // std::cout << tau_d << std::endl;


    TorqueControl  torque_control_callback;
    torque_control_callback = [&](RCI::robot::RobotState robot_state) -> Torques {

        static double time=0;
        time += 0.001;

        model.GetTauWithFriction(robot_state.q, robot_state.dq_m, robot_state.ddq_c, tau_full,
                                 tau_inertial, tau_coriolis, tau_friction, tau_gravity);

        // PD+ control
        // tau_d = Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_coriolis.data())
        //         + Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_friction.data())
        //         + Eigen::Map<Eigen::Matrix<double, 7, 1>>(tau_gravity.data())
        //         - Bd * Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq_m.data())
        //         - Kd * (q_d - Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data()));
        tau_d = Eigen::MatrixXd::Zero(7, 1)
                - Bd * Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.dq_m.data())
                + Kd * (q_d - Eigen::Map<Eigen::Matrix<double, 7, 1>>(robot_state.q.data()));

        Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;

        Torques output{};
        output.tau_c = tau_d_array;

        if(time>30){
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }
        return output;
    };

    // start real-time control loop
    std::cout  << "Make sure you have the user stop at hand!" << std::endl
               << "After starting try to push the robot and see how it reacts." << std::endl
               << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kTorque,
    //                 RCI::robot::StartMoveRequest::MotionGeneratorMode::kIdle);

    robot.Control(torque_control_callback);

    //直接按ctrl_c停止，注意急停开关
    return 0;
}