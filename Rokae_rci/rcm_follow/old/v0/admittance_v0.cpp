#include <iostream>
#include <time.h>
#include <cmath>
#include <fstream>
#include <functional>
#include <Eigen/Dense>
#include <functional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
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

using namespace Eigen;
using namespace xmate;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;

class AdmittanceModel
{
public:
  AdmittanceModel(){}

  AdmittanceModel(int n, Vector3d &pc, double dt, MatrixXd &Lambda, MatrixXd &D, double Alpha, double Beta): 
      m_n(n), m_pc(pc), m_dt(dt), m_Lambda(Lambda), m_D(D), m_Alpha(Alpha), m_Beta(Beta)
  {
    A_last = MatrixXd::Zero(2, n);
    ZL_last = MatrixXd::Zero(n - 2, n);
  }
  
  VectorXd nextStep(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d)
  {
    MatrixXd B = T.block<3, 2>(0, 0);
    Vector3d pr = T.block<3, 1>(0, 3);
    MatrixXd A = calculateA(B, pr, J);
    MatrixXd Z = calculateZ(A);
    VectorXd q_d_expect = calculateQd(A, Z, B, pr, tau, q_d);
    
    VectorXd q = q_now + q_d_expect * m_dt;
    
    update(A, Z);

    return q;
  }

  double error(MatrixXd &T)
  {
    Vector3d pr = T.block<3, 1>(0, 3);
    Vector3d pt = T.block<3, 3>(0, 0) * Vector3d(0, 0, 1) + pr;
    
    return ((pr - pt).cross(pr - m_pc).norm() / (pr - pt).norm());
  }

private:
  MatrixXd m_Lambda, m_D, A_last, ZL_last;
  Vector3d m_pc;
  double m_Alpha, m_Beta, m_dt;
  int m_n;

  MatrixXd calculateA(MatrixXd &B, Vector3d &pr, MatrixXd &J)
  {
    Vector3d p = pr - m_pc;
    MatrixXd Jc = B.transpose() * MatrixXd{{1, 0, 0, 0, -p(2), p(1)}, 
                                           {0, 1, 0, p(2), 0, -p(0)}, 
                                           {0, 0, 1, -p(1), p(0), 0}};

    return Jc * J;
  }

  MatrixXd calculateZ(MatrixXd &A)
  {
    MatrixXd Z(m_n - 2, m_n);
    MatrixXd Am(2, 2), Ar(2, m_n - 2), Zm, Zr;
    int i, j;
    bool flag = false;
    for (i = 0; i < m_n - 1; i++)
    {
      for (j = i + 1; j < m_n; j++)
      { 
        if (A(0, i) * A(1, j) - A(0, j) * A(1, i) != 0) // full rank
        {
          flag = true;
          break;
        }
      }
      if (flag)
        {
          break;
        }
    }

    Am << A.block<2, 1>(0, i), A.block<2, 1>(0, j);
    Ar << A.block(0, 0, 2, i), A.block(0, i + 1, 2, j - i - 1), A.block(0, j + 1, 2, m_n - j - 1);
    Zm = -Ar.transpose() * Am.transpose().inverse();
    Zr = MatrixXd::Identity(m_n - 2, m_n - 2);

    Z << Zr.block(0, 0, m_n - 2, i), Zm.block(0, 0, m_n - 2, 1), Zr.block(0, i, m_n - 2, j - i - 1), Zm.block(0, 1, m_n - 2, 1), Zr.block(0, j - 1, m_n - 2, m_n - j - 1);
    return Z;
  }

  VectorXd calculateQd(MatrixXd &A, MatrixXd &Z, MatrixXd &B, Vector3d &pr, VectorXd &tau, VectorXd &q_d_last)
  {
    MatrixXd A_inv, Z_inv, Pf, A_d, ZL_d, Co1, Co2;
    VectorXd xc, q_d;
    A_inv = m_Lambda.inverse() * A.transpose() * (A * m_Lambda.inverse() * A.transpose()).inverse();
    Z_inv = m_Lambda * Z.transpose() * (Z * m_Lambda * Z.transpose()).inverse();
    Pf = Z_inv * Z;

    xc = B.transpose() * (pr - m_pc);
    A_d = (A - A_last) / m_dt;
    ZL_d = (Z * m_Lambda - ZL_last) / m_dt;

    Co1 = -m_Lambda.inverse() * (Z_inv * m_D * Z * m_Lambda + Z_inv * ZL_d) - A_inv * (A_d + 2 * m_Alpha * A);
    Co2 = m_Lambda.inverse() * (Pf * tau) - m_Beta * m_Beta * A_inv * xc;

    q_d = (MatrixXd::Identity(m_n, m_n) - Co1 * m_dt).inverse() * (q_d_last + Co2 * m_dt);
    return q_d;
  }

  void update(MatrixXd &A, MatrixXd &Z)
  {
    A_last = A;
    ZL_last = Z * m_Lambda;
  }
};

class Runner
{
public:
  Runner(){}
  
  Runner(AdmittanceModel &model, VectorXd &q_init)
  {
    m_model = model;
    m_q_command = q_init;
    m_error = 0;
    m_init = false;
  }

  void setTimer(int msec)
  {
    t_timer = std::thread(&Runner::timer, this, msec);
    
  }

  void setParameters(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d)
  {
    std::lock_guard<std::mutex> lock_p(mtx_p);
    m_T = T;
    m_J = J;
    m_tau =tau;
    m_q_now = q_now;
    m_q_d = q_d;
    m_init = true;
  }

  VectorXd getResult()
  {
    std::lock_guard<std::mutex> lock_r(mtx_r);
    return m_q_command;
  }

  void getResult(VectorXd &q_command, double &error)
  {
    std::lock_guard<std::mutex> lock_r(mtx_r);
    q_command = m_q_command;
    error = m_error;
  }

  void calcNext()
  {
    while(1)
    {
      std::unique_lock<std::mutex> lock_c(mtx_c);
      cv.wait(lock_c, [this] {
        return m_init;
      });
      lock_c.unlock();
      mtx_p.lock();
      MatrixXd T = m_T, J = m_J;
      VectorXd tau = m_tau, q_now = m_q_now, q_d = m_q_d;
      mtx_p.unlock();
      if (tau.array().abs().maxCoeff() > 2)
      {
        VectorXd q_command = m_model.nextStep(T, J, tau, q_now, q_d);
        double error = m_model.error(T);

        mtx_r.lock();
        m_q_command = q_command;
        m_error = error;
        mtx_r.unlock();
      }
    }
  }

private:
  void timer(int msec)
  {
    clock_t start, finish;
    t_calc = std::thread(&Runner::calcNext, this);
    
    while (1)
    {
      start = clock();
      std::this_thread::sleep_for(std::chrono::microseconds(20));
      mtx_c.lock();
      double totaltime = 0;
      while (1)
      {
        finish = clock();
        totaltime = (double)(finish - start) / CLOCKS_PER_SEC * 1000;
        if (totaltime > msec)
        {
          mtx_c.unlock();
          cv.notify_one();
          break;
        }
      }
    }
  }

private:
  AdmittanceModel m_model;
  MatrixXd m_T, m_J;
  VectorXd m_tau, m_q_now, m_q_d, m_q_command;
  double m_error;
  bool m_init;
  std::mutex mtx_p, mtx_r, mtx_c;
  std::condition_variable cv;
  std::thread t_timer, t_calc;
};


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
    std::array<double,7> q_init, q_next;

    q_init = robot.receiveRobotState().q;
    q_next = {{0,PI/6,0,PI/3,0,PI/3,0}};
    MOVEJ(0.2,q_init,q_next,robot);

    // 求RCM点
    static std::array<double, 16> coord_1 = {{1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 1, 0.3,
                                              0, 0, 0, 1}};
    static std::array<double, 16> coord_2 = {{1, 0, 0, 0,
                                              0, 1, 0, 0,
                                              0, 0, 1, 0,
                                              0, 0, 0, 1}};

    std::array<double, 16> pose = xmatemodel.GetCartPose(q_next, coord_1, coord_2, SegmentFrame::kEndEffector);
    std::cout << pose[3] << " " << pose[7] << " " << pose[11] << std::endl;

    MatrixXd Lambda(DiagonalMatrix<double, 7>(0.5, 0.7, 0.5, 0.2, 0.1, 0.1, 0.1));
    MatrixXd D(DiagonalMatrix<double, 5>(15, 15, 15, 15, 15));
    double Alpha = 20, Beta = 20;
    Vector3d pc(pose[3], pose[7], pose[11]);
    std::cout << pc <<std::endl;
    AdmittanceModel model(7, pc, 0.01, Lambda, D, Alpha, Beta);

    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);
    
    double time = 0, error = 0;
    VectorXd q_now(7), q_last(7), q_d(7), q_command(7), dq_now(7); 
    q_d = MatrixXd::Zero(7, 1);
    q_last = Map<Matrix<double, 7, 1>>(robot.receiveRobotState().q.data());
    MatrixXd T(4, 4), J(6, 7);
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
    // x: 20, y: 20, z: 5
    extra_force << 0, 0, 0, 0, 0, 0;

    VectorXd tau_del(7);       // 零漂
    std::array<double,7> tau_full_init, tau_inertial_init, tau_coriolis_init, tau_friction_init, tau_gravity_init;   // 理论计算值

    std::array<double,7> tau_init = robot.receiveRobotState().tau_m;     // 读取机械臂关节力矩

    q_next = robot.receiveRobotState().q;
    xmatemodel.GetTauWithFriction(mass, cog, inertia, q_next, dq, ddq, tau_full_init, tau_inertial_init, tau_coriolis_init, tau_friction_init,tau_gravity_init);
    tau_del = Map<Matrix<double, 7, 1>>(tau_full_init.data()) - Map<Matrix<double, 7, 1>>(tau_init.data()); // 机械臂方向统一,tau取负号
    std::cout<<tau_del[0]<<", "<<tau_del[1]<<", "<<tau_del[2]<<", "<<tau_del[3]<<", "<<tau_del[4]<<", "<<tau_del[5]<<", "<<tau_del[6]<<std::endl;

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
        J = Map<Matrix<double, 6, 7, RowMajor>>(xmatemodel.Jacobian(robot_state.q, SegmentFrame::kJoint7).data());
        // tau = J.transpose() * F;
        // xmatemodel.GetTauNoFriction(robot_state.q, dq, ddq, tau_full, tau_inertial, tau_coriolis, tau_gravity);
        xmatemodel.GetTauWithFriction(mass, cog, inertia, robot_state.q, dq, ddq, tau_full, tau_inertial, tau_coriolis, tau_friction, tau_gravity);
        tau = Map<Matrix<double, 7, 1>>(tau_full.data()) - Map<Matrix<double, 7, 1>>(robot_state.tau_m.data()); // 机械臂方向统一,tau取负号
        // 补偿
        for(int i = 0; i < 7; i++) tau[i] -= tau_del[i];
        
        // 恒力
        tau_extra = J.transpose() * extra_force;
        for(int i = 0; i < 7; i++) tau[i] += tau_extra[i];

        tau_list.push_back(tau);
        joint_list.push_back(q_now);
        std::cout << tau <<std::endl << std::endl;     
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

        if (time < 60)
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



// int main()
// {
//   MatrixXd Lambda(DiagonalMatrix<double, 7>(1, 1, 1, 1, 1, 1, 1));
//   MatrixXd D(DiagonalMatrix<double, 5>(0.1, 0.1, 0.1, 0.1, 0.1));
//   double Alpha = 10, Beta = 10;
//   Vector3d pc(0.8833, 0, -0.0345);
//   AdmittanceModel model(7, pc, 0.01, Lambda, D, Alpha, Beta);
  
//   // test
//   const double pi = 3.14159265;
//   VectorXd q_now{{0, pi/3, 0, pi/3, 0, pi/6, 0}};
//   VectorXd q_d = MatrixXd::Zero(7, 1);
//   MatrixXd T{{-0.866, 0, 0.5, 0.7833}, {0, 1, 0, 0}, {-0.5, 0, -0.866, 0.1387}, {0, 0, 0, 1}};
//   MatrixXd J{{0, -0.2028, 0, -0.3998, 0, -0.2168, 0}, 
//             {0.7833, 0, 0.5673, 0, 0.1251, 0, 0}, 
//             {0, -0.7833, 0, -0.4421, 0, -0.1252, 0}, 
//             {0, 0, 0.866, 0, 0.866, 0, 0.5}, 
//             {0, 1, 0, 1, 0, 1, 0}, 
//             {1, 0, 0.5, 0, -0.5, 0, -0.866}};
//   VectorXd tau{{7.8333, -7.8333, 5.6727, -4.4212, 1.2515, -1.2515, 0}};
  
//   clock_t start, end;
//   start = clock();
//   VectorXd q = model.nextStep(T, J, tau, q_now, q_d);
//   double error = model.error(T);
//   end = clock();
//   std::cout << "time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
//   std::cout << "q: \n" << q << std::endl;
//   std::cout << "error: " << error << std::endl;
// }