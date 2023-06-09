#include <iostream>
#include <time.h>
#include <cmath>
#include <fstream>
#include <functional>
#include <Eigen/Dense>
#include <functional>
#include <thread>
#include <vector>
#include <mutex>
#include <thread>

#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "unistd.h"
#include "move.h"
#include "model.h"

#include "admittance.h"
#include "tcp_recv.h"

using namespace Eigen;
using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;

class LoopRecv
{
public:
    LoopRecv()
    {
        zero_count = 0;
        stop_flag = false;
        data = std::vector<double>(2);
        if (!rec.setup())
        {
            cout << "FAIL to setup socket!" << endl;
            setup_flag = false;
        }
        else
        {
            setup_flag = true;
            t_get = std::thread(&LoopRecv::getVec, this);
        }
    }

    void getParam(std::vector<double>& p)
    {
        std::lock_guard<std::mutex> lock(mtx);
        p[0] = data[0];
        p[1] = data[1];
    }

    void stop()
    {
        stop_flag = true;
        t_get.join();
    }
    

private:
    CtrlParam param;
    Recver rec;
    std::vector<double> data;
    bool setup_flag, stop_flag;
    std::mutex mtx;
    std::thread t_get;
    int zero_count;

    void getVec()
    {
        while(setup_flag && !stop_flag)
        {
            if (!rec.getvec(param))
            {
                cout << "FAIL to receieve params!" << endl;
                setup_flag = false;
                break;
            }
            else
            {
                if(param.x == 0 && param.y == 0)
                {
                    zero_count++;
                    if(zero_count > 10)
                    {
                        zero_count = 0;
                        mtx.lock();
                        data[0] = 0;
                        data[1] = 0;
                        mtx.unlock();
                    }
                }
                else
                {
                    zero_count = 0;
                    mtx.lock();
                    data[0] = param.x;
                    data[1] = param.y;
                    mtx.unlock();
                }

                
            }
            sleep(0.005);
        }
    }
};

int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.0.160";
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

    LoopRecv recv;
    std::vector<double> p(2);

    //防止网络连接失败
    sleep(1);

    robot.setMotorPower(1);

    const double PI=3.14159;
    std::array<double,7> q_init, q_next;

    q_init = robot.receiveRobotState().q;
    q_next = {{0,PI/6,0,PI/3,0,PI/3,0}};
    MOVEJ(0.2,q_init,q_next,robot);

    // 设置工具坐标系
    std::array<double, 16> F_T_EE = {{0.86603, 0, 0.5, 0,
                                    0, 1, 0, 0,
                                    -0.5, 0, 0.86603, 0.28,
                                    0, 0, 0, 1}};
    std::array<double, 16> EE_T_K = {{1, 0, 0, 0,
                                    0, 1, 0, 0,
                                    0, 0, 1, 0,
                                    0, 0, 0, 1}};
    
    MatrixXd T_CAM = Map<Matrix<double, 4, 4, RowMajor>>(F_T_EE.data());

    xmatemodel.SetCoor(F_T_EE, EE_T_K);
    std::array<double, 16> O_T_Tool = xmatemodel.GetCartPose(q_next, SegmentFrame::kEndEffector);
    std::cout << "O_T_Tool: " << std::endl;
    for(int i = 0; i < 4; i++)
    {
        for(int j = 0; j < 4; j++)
        {
            std::cout << O_T_Tool[i * 4 + j] << " ";
        }
        std::cout << std::endl;
    }
    double z0 = O_T_Tool[2 * 4 + 3] - 0.05;

    double delta_z = 0, delta_z_int = 0;
    double z_kp = 20, z_ki = 1, delta_z_int_max = 10 / z_ki;

    // 求RCM点
    MatrixXd F_T_RCM {{1, 0, 0, 0},
                    {0, 1, 0, 0},
                    {0, 0, 1, 0.28},
                    {0, 0, 0, 1}};
    MatrixXd O_T_RCM = Map<Matrix<double, 4, 4, RowMajor>>(xmatemodel.GetCartPose(q_next, SegmentFrame::kFlange).data()) * F_T_RCM;
    Vector3d pc = O_T_RCM.block(0, 3, 3, 1);
    std::cout << "RCM_Point: " << std::endl << pc << std::endl;

    MatrixXd Lambda(DiagonalMatrix<double, 7>(0.5, 0.7, 0.5, 0.2, 0.1, 0.1, 1));
    MatrixXd D(DiagonalMatrix<double, 5>(15, 15, 15, 15, 15));
    double Alpha = 20, Beta = 20;

    AdmittanceModel model(7, pc, 0.01, Lambda, D, Alpha, Beta);

    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    
    double time = 0, error = 0;
    VectorXd q_now(7), q_last(7), q_d(7), q_command(7), dq_now(7); 
    q_d = MatrixXd::Zero(7, 1);
    q_last = Map<Matrix<double, 7, 1>>(robot.receiveRobotState().q.data());
    MatrixXd T(4, 4), J(6, 7), T_end(4, 4), J_end(6, 7);
    VectorXd tau(7);
    std::array<double,7> tau_full, tau_inertial, tau_coriolis, tau_friction, tau_gravity;
    std::array<double,7> dq = {{0,0,0,0,0,0,0}};
    std::array<double,7> ddq = {{0,0,0,0,0,0,0}};
    double mass = 0.193;
    std::array<double, 3> cog {{0,0,0.065}};
    std::array<double, 9> inertia {{0,0,0,0,0,0,0,0,0}};

    // VectorXd F {{0, 100, 0, 0, 0, 0}};

    std::vector<VectorXd> tau_list;
    std::vector<VectorXd> joint_list;
    
    Runner runner(model, q_last);
    runner.setTimer(2);

    VectorXd tau_extra(7);
    VectorXd extra_force(6);
    // x: 20, y: 20, z: 5, rx: 5, ry: 5, rz: 3
    extra_force << 0, 0, 0, 0, 0, 0;

    VectorXd tau_del(7);       // 零漂
    std::array<double,7> tau_full_init, tau_inertial_init, tau_coriolis_init, tau_friction_init, tau_gravity_init;   // 理论计算值

    std::array<double,7> tau_init = robot.receiveRobotState().tau_m;     // 读取机械臂关节力矩

    q_next = robot.receiveRobotState().q;
    xmatemodel.GetTauWithFriction(mass, cog, inertia, q_next, dq, ddq, tau_full_init, tau_inertial_init, tau_coriolis_init, tau_friction_init,tau_gravity_init);
    tau_del = Map<Matrix<double, 7, 1>>(tau_full_init.data()) - Map<Matrix<double, 7, 1>>(tau_init.data()); // 机械臂方向统一,tau取负号
    std::cout<< "tau_del: "<<tau_del[0]<<", "<<tau_del[1]<<", "<<tau_del[2]<<", "<<tau_del[3]<<", "<<tau_del[4]<<", "<<tau_del[5]<<", "<<tau_del[6]<<std::endl;

    JointPositions output;
    std::string joint_callback = "joint_callback";
    JointControl joint_position_callback;

    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }
        
        time += 0.001;

        q_now = Map<Matrix<double, 7, 1>>(robot_state.q.data());
        q_d = (q_now - q_last) / 0.001;
        T = Map<Matrix<double, 4, 4, RowMajor>>(robot_state.toolTobase_pos_m.data());
        J = Map<Matrix<double, 6, 7, RowMajor>>(xmatemodel.Jacobian(robot_state.q, SegmentFrame::kFlange).data());
        T_end = T * T_CAM;
        // J_end = Map<Matrix<double, 6, 7, RowMajor>>(xmatemodel.Jacobian(robot_state.q, SegmentFrame::kEndEffector).data());
        
        xmatemodel.GetTauWithFriction(mass, cog, inertia, robot_state.q, dq, ddq, tau_full, tau_inertial, tau_coriolis, tau_friction, tau_gravity);
        tau = Map<Matrix<double, 7, 1>>(tau_full.data()) - Map<Matrix<double, 7, 1>>(robot_state.tau_m.data()); // 机械臂方向统一,tau取负号
        // 补偿
        for(int i = 0; i < 7; i++) tau[i] -= tau_del[i];
        
        // 吸引力
        recv.getParam(p);

        // // X-axis
        // if(abs(p[0]) < 0.1)  extra_force[0] = 0;
        // else if(abs(p[0]) <= 1) extra_force[0] = p[0] > 0 ? -(p[0] * 20 + 10) : -(p[0] * 20 - 10);
        // else extra_force[0] = 0;
        // X-axis
        if(abs(p[0]) < 0.1)  extra_force[4] = 0;
        else if(abs(p[0]) <= 1) extra_force[4] = p[0] > 0 ? -(p[0] * 15 + 5) : -(p[0] * 15 - 5);
        else extra_force[4] = 0;

        // Y-axis
        if(abs(p[1]) < 0.1)  extra_force[1] = 0;
        else if(abs(p[1]) <= 1) extra_force[1] = p[1] > 0 ? -(p[1] * 20 + 10) : -(p[1] * 20 - 10);
        else extra_force[1] = 0;

        // Z-axis PID control
        delta_z = z0 - double(T_end(2, 3));
        if(abs(delta_z) < 0.005)
        {
            delta_z_int = 0;
        }
        else
        {
            delta_z_int += delta_z;
        }
        
        if(delta_z_int > delta_z_int_max)   delta_z_int = delta_z_int_max;
        else if(delta_z_int < -delta_z_int_max) delta_z_int = -delta_z_int_max;

        extra_force[2] = z_kp * delta_z + z_ki * delta_z_int;

        if(extra_force[0] > 30) extra_force[0] = 30;
        else if(extra_force[0] < -30) extra_force[0] = -30;

        if(extra_force[1] > 30) extra_force[1] = 30;
        else if(extra_force[1] < -30) extra_force[1] = -30;

        if(extra_force[2] > 30) extra_force[2] = 30;
        else if(extra_force[2] < -30) extra_force[2] = -30;

        if(extra_force[4] > 20) extra_force[4] = 20;
        else if(extra_force[4] < -20) extra_force[4] = -20;

        std::cout << extra_force <<std::endl << std::endl; 
        
        tau_extra = J.transpose() * extra_force;
        for(int i = 0; i < 7; i++) tau[i] += tau_extra[i];

        tau_list.push_back(tau);
        joint_list.push_back(q_now);
        // std::cout << tau <<std::endl << std::endl;     
        runner.setParameters(T, J, tau, q_now, q_d);
        
        q_command = runner.getResult();
        // runner.getResult(q_command, error); // 获得误差
        // std::cout << q_command << std::endl;

        dq_now = Map<Matrix<double, 7, 1>>(robot_state.dq_m.data());

        if (dq_now.array().abs().maxCoeff() > 2 * PI)
        {
          
          std::cout << "速度超限: " << dq_now.array().abs().maxCoeff() << std::endl;
          std::cout << dq_now << std::endl;
          return MotionFinished(output);
        }

        if (time < 120)
        {
            output = {{q_command[0], q_command[1], q_command[2], q_command[3], q_command[4], q_command[5], q_command[6]}};
        }
        else
        {
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }

        q_last = q_now;

        return output;        
    };

    robot.Control(joint_position_callback);
    recv.stop();
    robot.setMotorPower(0);

    // std::ofstream ofs;
    // ofs.open("tauq_list.txt");
    // if (!ofs)
    // {
    //     std::cout << "file open error!" << std::endl;
    // }

    // for(auto it = tau_list.begin(); it != tau_list.end(); it++)
    // {
    //   ofs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << std::endl;
    // }

    // ofs.close();

    // std::ofstream qfs;
    // qfs.open("q_list.txt");
    // if (!qfs)
    // {
    //     std::cout << "file open error!" << std::endl;
    // }

    // for(auto it = joint_list.begin(); it != joint_list.end(); it++)
    // {
    //   qfs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << std::endl;
    // }

    // qfs.close();

    return 0;
}
