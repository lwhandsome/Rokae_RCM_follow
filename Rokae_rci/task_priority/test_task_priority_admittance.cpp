#include <iostream>
#include <time.h>
#include <cmath>
#include <fstream>
#include <functional>
#include <Eigen/Dense>
#include <functional>
#include <thread>
#include <vector>

#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "unistd.h"
#include "move.h"
#include "model.h"

#include "task_priority.h"
#include "runner.h"

using namespace Eigen;
using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;

int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.0.160";
    std::string name = "callback";
    uint16_t port = 1337;
    std::string file = "../xmate.ini";
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
    //INIT 部分
    std::array<double,7> INIT_q_now, INIT_q_desired;
    // 移动到初始位置
    INIT_q_now = robot.receiveRobotState().q;
    INIT_q_desired = {{0,PI/6,0,PI/3,0,PI/3,0}};
    MOVEJ(0.2,INIT_q_now,INIT_q_desired,robot);

    MatrixXd INIT_T = Map<Matrix<double, 4, 4, RowMajor>>(xmatemodel.GetCartPose(INIT_q_desired, SegmentFrame::kFlange).data());
    std::cout << "INIT_T: " << std::endl << INIT_T << std::endl;

    // 求trocar点
    Vector3d p_trocar = INIT_T.block<3, 1>(0, 3) + 0.2 *  INIT_T.block<3, 1>(0, 2);
    std::cout << "Trocar_Point: " << std::endl << p_trocar << std::endl;
    // 求disired点
    Vector3d p_desired = INIT_T.block<3, 1>(0, 3) + Vector3d{{-0.01, 0, 0}};
    std::cout << "Desired_Point: " << std::endl << p_desired << std::endl;
    
    // 模型参数定义
    MatrixXd K = DiagonalMatrix<double, 4>(100, 50, 50, 50);
    MatrixXd D = DiagonalMatrix<double, 4>(10, 20, 20, 20);
    int calculate_interval = 12;
    TaskPriorityModel model(7, p_trocar, p_desired, 0.001 * calculate_interval, K, D);

    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    
    // RUNNER 部分
    double time = 0, runner_count = 0;
    VectorXd q_now(7), q_last(7), q_desired(7), q_command(7), dq_now(7);
    VectorXd error = MatrixXd::Zero(4, 1);
    dq_now = MatrixXd::Zero(7, 1);
    q_last = Map<Matrix<double, 7, 1>>(robot.receiveRobotState().q.data());
    q_desired = q_last;
    MatrixXd T(4, 4) ,J(6, 7);
    VectorXd tau(7);
    std::array<double,7> tau_full, tau_inertial, tau_coriolis, tau_friction, tau_gravity;
    std::array<double,7> dq = {{0,0,0,0,0,0,0}};
    std::array<double,7> ddq = {{0,0,0,0,0,0,0}};
    double mass = 0.193;
    std::array<double, 3> cog {{0,0,0.065}};
    std::array<double, 9> inertia {{0,0,0,0,0,0,0,0,0}};

    // Runner runner(model, q_last);
    // runner.start();

    VectorXd tau_del(7);       // 零漂
    std::array<double,7> tau_full_init, tau_inertial_init, tau_coriolis_init, tau_friction_init, tau_gravity_init;   // 理论计算值

    std::array<double,7> tau_init = robot.receiveRobotState().tau_m;     // 读取机械臂关节力矩

    std::array<double,7> q = robot.receiveRobotState().q;
    xmatemodel.GetTauWithFriction(mass, cog, inertia, q, dq, ddq, tau_full_init, tau_inertial_init, tau_coriolis_init, tau_friction_init,tau_gravity_init);
    tau_del = Map<Matrix<double, 7, 1>>(tau_full_init.data()) - Map<Matrix<double, 7, 1>>(tau_init.data()); // 机械臂方向统一,tau取负号
    std::cout<< "tau_del: "<<tau_del[0]<<", "<<tau_del[1]<<", "<<tau_del[2]<<", "<<tau_del[3]<<", "<<tau_del[4]<<", "<<tau_del[5]<<", "<<tau_del[6]<<std::endl;

    // record
    std::vector<VectorXd> tau_list, q_command_list, q_desired_list, q_now_list, error_list;

    JointPositions output;
    std::string joint_callback = "joint_callback";
    JointControl joint_position_callback;
    std::cout << "start control!" << std::endl;

    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }
        
        time += 0.001;

        runner_count++;

        q_now = Map<Matrix<double, 7, 1>>(robot_state.q.data());
        dq_now = Map<Matrix<double, 7, 1>>(robot_state.dq_m.data());

        xmatemodel.GetTauWithFriction(mass, cog, inertia, robot_state.q, dq, ddq, tau_full, tau_inertial, tau_coriolis, tau_friction, tau_gravity);
        tau = Map<Matrix<double, 7, 1>>(tau_full.data()) - Map<Matrix<double, 7, 1>>(robot_state.tau_m.data()); // 机械臂方向统一,tau取负号
        // 补偿
        for(int i = 0; i < 7; i++) tau[i] -= tau_del[i];
        // std::cout << "tau:" << std::endl << tau << std::endl; 

        if(runner_count == calculate_interval)
        {
            runner_count = 0;

            T = Map<Matrix<double, 4, 4, RowMajor>>(robot_state.toolTobase_pos_m.data());
            J = Map<Matrix<double, 6, 7, RowMajor>>(xmatemodel.Jacobian(robot_state.q, SegmentFrame::kFlange).data());

            q_desired = model.nextStep(T, J, tau, q_now, dq_now);
            error = model.error();
            // runner.setParameters(T, J, tau, q_now, dq_now);
	
            // q_command = runner.getResult();
            // runner.getResult(q_desired, error); // 获得误差
            // std::cout << "q_desired:" << std::endl << q_desired << std::endl;
            // std::cout << "error: "<< std::endl << error << std::endl << std::endl;

            q_last = q_now;
        }

        //q_command = (runner_count + 1) / calculate_interval * (q_desired - q_last) + q_last;
        q_command = q_desired;

        // record
        tau_list.push_back(tau);
        q_now_list.push_back(q_now);
        q_desired_list.push_back(q_desired);
        q_command_list.push_back(q_command);
        error_list.push_back(error);

        if (dq_now.array().abs().maxCoeff() > 2 * PI)
        {
          
          std::cout << "速度超限: " << dq_now.array().abs().maxCoeff() << std::endl;
          std::cout << dq_now << std::endl;
          return MotionFinished(output);
        }

        if(time > 9.9995 && time < 10.0005)
        {
            p_desired(0) = robot_state.toolTobase_pos_m[3];
            p_desired(1) = robot_state.toolTobase_pos_m[7];
            p_desired(2) = robot_state.toolTobase_pos_m[11];
            std::cout << "Desired_Point: " << std::endl << p_desired << std::endl;
            model.changePositionDesired(p_desired);
        }

        if (time < 30)
        {
            // output = {{q_now[0], q_now[1], q_now[2], q_now[3], q_now[4], q_now[5], q_now[6]}};
            output = {{q_command[0], q_command[1], q_command[2], q_command[3], q_command[4], q_command[5], q_command[6]}};
        }
        else
        {
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }

        return output;        
    };

    robot.Control(joint_position_callback);
    robot.setMotorPower(0);
    //runner.stop();

    // record
    std::ofstream ofs;
    ofs.open("tau_list.txt");
    if (!ofs)
    {
        std::cout << "file open error!" << std::endl;
    }
    for(auto it = tau_list.begin(); it != tau_list.end(); it++)
    {
      ofs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << std::endl;
    }
    ofs.close();  

    ofs.open("q_now_list.txt");
    if (!ofs)
    {
        std::cout << "file open error!" << std::endl;
    }
    for(auto it = q_now_list.begin(); it != q_now_list.end(); it++)
    {
      ofs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << std::endl;
    }
    ofs.close();

    ofs.open("q_desired_list.txt");
    if (!ofs)
    {
        std::cout << "file open error!" << std::endl;
    }
    for(auto it = q_desired_list.begin(); it != q_desired_list.end(); it++)
    {
      ofs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << std::endl;
    }
    ofs.close();

    ofs.open("q_command_list.txt");
    if (!ofs)
    {
        std::cout << "file open error!" << std::endl;
    }
    for(auto it = q_command_list.begin(); it != q_command_list.end(); it++)
    {
      ofs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << std::endl;
    }
    ofs.close();
    
    ofs.open("error_list.txt");
    if (!ofs)
    {
        std::cout << "file open error!" << std::endl;
    }
    for(auto it = error_list.begin(); it != error_list.end(); it++)
    {
      ofs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << std::endl;
    }
    ofs.close();  

    return 0;
}
