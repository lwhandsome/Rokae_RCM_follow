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

#include "ini.h"
#include "joint_motion.h"
#include "print_rci.h"
#include "rci_data/command_types.h"
#include "rci_data/robot_datas.h"
#include "robot.h"
#include "unistd.h"
#include "move.h"
#include "model.h"
#include "network1.h"
#include <iterator>

using namespace Eigen;
using namespace xmate;
using namespace std;
using JointControl = std::function<JointPositions(RCI::robot::RobotState robot_state)>;

std::mutex mtx;

double distance(Vector3d &p_1, Vector3d &p_2, Vector3d &p_RCM)
{
    Vector3d p12 = p_1 - p_2;
    return p12.cross(p_1 - p_RCM).norm() / p12.norm();
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

std::array<double, 7> Get_joint_position_quick(std::array<double, 7> &q_last, Vector3d &p_goal, XmateModel &xmatemodel)
{
  const double PI = 3.14159;
  std::array<double, 7> q_init,q_next;
  std::array<double, 16> pose1, pose2;
  std::array<double, 16> pose_next;
  Vector3d p_RCM, p_1, p_2, n, n_last, o_last, a_last,n_next, o_next, a_next, p_flange;
  p_RCM << 0.76, 0, 0.345;
  double theta;
  double effLenth = 0.28;
  Matrix3d R = Matrix3d::Identity(3,3);
  Matrix3d I = Matrix3d::Identity(3,3);
  Matrix3d N;

  static std::array<double, 16> coord_1 = {{1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0.28,
                                            0, 0, 0, 1}};
  static std::array<double, 16> coord_2 = {{1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1}};
  q_init = q_last;
  q_next = q_last;
  pose1 = xmatemodel.GetCartPose(q_init, coord_1, coord_2, SegmentFrame::kEndEffector); //末端位置 4X4 
  pose2 = xmatemodel.GetCartPose(q_init, SegmentFrame::kJoint7); //法兰位置矩阵 4X4 
  for (int i = 0; i < 3; i++)
  {
      p_1(i) = pose1.at(4 * i + 3);
      p_2(i) = pose2.at(4 * i + 3);
      n_last(i) = pose1.at(4 * i); 
      o_last(i) = pose1.at(4 * i + 1);
      a_last(i) = pose1.at(4 * i + 2);
  }
  a_next = (p_goal - p_RCM);
  a_next = a_next/a_next.norm();
  n = a_next.cross(a_last);
  n = n/n.norm();
  theta = acos(a_next.dot(a_last));
  N << 0, -n(2), n(1),
       n(2), 0, -n(0),
       -n(1), n(0), 0;
  R = cos(theta)*I + (1-cos(theta))*n*n.transpose() + sin(theta)*N;
  n_next = R * n_last;
  o_next = R * o_last;
  p_flange = p_goal - effLenth*a_next;

//   std::cout << p_flange <<std::endl;
//   std::cout << pose2 <<std::endl;
  

for (int i = 0; i < 4; i++)
  {
      if(i<3)
      {
        pose_next.at(4 * i) = n_next(i);
        pose_next.at(4 * i + 1) = o_next(i);
        pose_next.at(4 * i + 2) = a_next(i); 
        pose_next.at(4 * i + 3) = p_flange(i);
      }else{
        pose_next.at(4 * i) = 0;
        pose_next.at(4 * i + 1) = 0;
        pose_next.at(4 * i + 2) = 0; 
        pose_next.at(4 * i + 3) = 1;
      }
  }

  int flag = xmatemodel.GetJointPos(pose_next,0,q_last,q_next);
//   for (int i = 0; i < 7; i++){
//       if(q_next.at(i)-q_last.at(i)>0.01)
//       {
//           std::cout << "rrr" << std::endl;
//           q_next.at(i) = q_last.at(i);
//           break;
//       }
//       else if (q_next.at(i)-q_last.at(i)<-0.01)
//       {
//           std::cout << "rrr" << std::endl;
//           q_next.at(i) = q_last.at(i);
//           break;
//       }
//   }
  for (int i = 0; i < 7; i++){
      if(abs(q_next.at(i)-q_last.at(i))>0.5)
      {
          q_next =  q_last;
          std::cout << "rrr" <<std::endl;
          break;
      }
  }
  //std::cout << q_next <<std::endl;
  return q_next;
}


Vector3d test(int countv,vector<double>& vec,double Z)
{
    MatrixXd T_cam {{0.86603, 0, -0.5, 0},
                  {0, 1, 0, 0},
                  {0.5, 0, 0.86603, 0},
                  {0, 0, 0, 1}};
    MatrixXd T_tool {{1, 0, 0, 0},
                     {0, 1, 0, 0},
                     {0, 0, 1, 0.28},
                     {0, 0, 0, 1}};
    double fx = 475.0337;
    double fy = 475.0233;
    double cx = 345.3079;
    double cy = 198.5522;
    double y = 0;
    double x = 0;
    Vector3d p_next {{0, 0, 0}};
    if(countv*2+1 < vec.size()){
        y = vec[countv*2 + 1];
        x = vec[countv*2];
    }else{
        y = 240;
        x = 200;
    }
    CtrlParam param;
    MatrixXd T(4, 4);
    Vector4d p_next_ext {{0, 0, 0, 1}};
    p_next_ext[0] = Z/fy*(y - cy);
    p_next_ext[1] = Z/fx*(x - cx);
    //将像素点转换为空间位置

    //T = Map<Matrix<double, 4, 4, RowMajor>>(xmatemodel.GetCartPose(q_now, SegmentFrame::kJoint7).data());
    p_next = (T_cam.inverse() * p_next_ext).block<3, 1>(0, 0);
    return p_next;
  
}

int main(int argc, char *argv[]) {
    std::string ipaddr = "192.168.3.41";
    std::string name = "callback";
    uint16_t port = 1337;
    std::string file = "../../xmate.ini";
    std::vector<std::array<double, 7>> joint_list;
    int countv = 0;
    std::vector<double> vec;
    std::vector<string> words;
    INIParser ini;
    if (ini.ReadINI(file)) {
        ipaddr = ini.GetString("network", "ip");
        port = static_cast<uint16_t>(ini.GetInt("network", "port"));
    }
    ifstream f("center2.txt",ios::in);
    istream_iterator<string> in_iter(f),eof;
    while(in_iter != eof){
        words.push_back(*in_iter++);
    }
    for(auto word:words){
        std::cout<< word <<" ";
        std::cout<< std::endl;
        vec.push_back(atof(word.c_str()));
    }
    f.close();

    
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
    
    //开始运动前先设置控制模式和运动模式
    robot.startMove(RCI::robot::StartMoveRequest::ControllerMode::kJointPosition,
                    RCI::robot::StartMoveRequest::MotionGeneratorMode::kJointPosition);

    Vector4d p_next_ext;
    Vector3d p_next {{0, 0, 0}}, p_now;
    std::array<double, 16> pose,p_goal;
    std::array<double, 7> delta_q{};
    std::array<double, 7> q_command, q_goal, q_now = robot.receiveRobotState().q, q_last = robot.receiveRobotState().q;
    MatrixXd T(4, 4);
    double z_0 = 0.110; //病床到机器人基座的距离
    double Z = 0; //相机深度

    MatrixXd T_tool {{1, 0, 0, 0},
                     {0, 1, 0, 0},
                     {0, 0, 1, 0.28},
                     {0, 0, 0, 1}};
    static std::array<double, 16> coord_1 = {{1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0.28,
                                            0, 0, 0, 1}};
    static std::array<double, 16> coord_2 = {{1, 0, 0, 0,
                                            0, 1, 0, 0,
                                            0, 0, 1, 0,
                                            0, 0, 0, 1}};

    T = Map<Matrix<double, 4, 4, RowMajor>>(xmatemodel.GetCartPose(q_next, SegmentFrame::kJoint7).data());
    
    std::cout << T * T_tool << std::endl;
    MatrixXd T_cam {{0.86603, 0, -0.5, 0},
                    {0, 1, 0, 0},
                    {0.5, 0, 0.86603, 0.28},
                    {0, 0, 0, 1}};
    double d, threshold = 0.2 * 0.01;
    double time = 0;
    double a_now = 1;
    int count = 0;
    std::mutex mtx;
    CtrlParam param;
    //Recver rec;
    // if (!rec.setup())
    // {
    //     std::cerr <<"rec初始化失败"<<std::endl;
    //     return -1;
    // }

    //std::thread t_read(recData, std::ref(rec), std::ref(p_next), std::ref(q_now), std::ref(xmatemodel), std::ref(Z));

    JointPositions output;
    std::string joint_callback = "joint_callback";
    JointControl joint_position_callback;

    joint_position_callback = [&](RCI::robot::RobotState robot_state) -> JointPositions {
        if(robot_state.control_command_success_rate <0.9){
            std::cout<<"通信质量较差："<<robot_state.control_command_success_rate<<std::endl;
        }
        
        time += 0.001;
        if(count%200 == 0){
            countv++;
        }
        

        mtx.lock();
        q_now = robot_state.q;
        joint_list.push_back(q_now);
        //Vector3d p_next_copy = p_next;
        Vector3d p_next_copy = test(countv,vec,Z);
        mtx.unlock();
        d = p_next_copy.norm();
        p_next_copy /= d;


        T = Map<Matrix<double, 4, 4, RowMajor>>(robot_state.toolTobase_pos_m.data());
        p_now = (T * T_tool).block<3, 1>(0, 3);
        a_now = (T * T_cam)(2, 2);
        Z = (p_now[2] - z_0)/a_now;

        std::cout << count << std::endl;
        if (count % 200 == 0)
        {   
            q_last = q_now;
            count = 0;
            p_next_copy = p_now + threshold * p_next_copy;
            q_goal = Get_joint_position_quick(robot_state.q, p_next_copy, xmatemodel);
            p_goal = xmatemodel.GetCartPose(q_goal, coord_1, coord_2, SegmentFrame::kEndEffector);
            for(int i = 0; i < 7;i++){
                    delta_q.at(i) = (q_goal.at(i) - q_now.at(i))/200;
            }
            if(d < 0.01){
                for(int i = 0; i < 7;i++){
                    delta_q.at(i) = 0;
                }
            }
            std::cout << d << std::endl;
        }
        for(int i= 0;i < 7; i++){
            q_command.at(i) = q_last.at(i) + delta_q.at(i) * (count + 1);
        }
        std::cout << robot_state.q <<std::endl;
        std::cout << q_command <<std::endl;
        std::cout << "----------------"<<std::endl;
        

        if (time < 15)
        {
            output = {{q_command[0], q_command[1], q_command[2], q_command[3], q_command[4], q_command[5], q_command[6]}};
        }
        else
        {
            std::cout<<"运动结束"<<std::endl;
            return MotionFinished(output);
        }

        count += 1;
        return output;        
    };

    robot.Control(joint_position_callback);
    robot.setMotorPower(0);
    std::ofstream qfs;
    qfs.open("q_list711.txt");
    if (!qfs)
    {
        std::cout << "file open error!" << std::endl;
    }

    for(auto it = joint_list.begin(); it != joint_list.end(); it++)
    {
      qfs << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << std::endl;
    }

    qfs.close();

    return 0;
}