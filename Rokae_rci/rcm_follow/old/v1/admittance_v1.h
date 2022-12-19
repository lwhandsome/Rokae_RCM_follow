#pragma once

#include <iostream>
#include <time.h>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <condition_variable>

using namespace Eigen;

class AdmittanceModel
{
public:
  AdmittanceModel();

  AdmittanceModel(int n, Vector3d &pc, double dt, MatrixXd &Lambda, MatrixXd &D, double Alpha, double Beta);
  
  VectorXd nextStep(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d);

  double error(MatrixXd &T);

private:
  MatrixXd m_Lambda, m_D, A_last, ZL_last;
  Vector3d m_pc;
  double m_Alpha, m_Beta, m_dt;
  int m_n;

  MatrixXd calculateA(MatrixXd &B, Vector3d &pr, MatrixXd &J);

  MatrixXd calculateZ(MatrixXd &A);

  VectorXd calculateQd(MatrixXd &A, MatrixXd &Z, MatrixXd &B, Vector3d &pr, VectorXd &tau, VectorXd &q_d_last);

  void update(MatrixXd &A, MatrixXd &Z);
};

class Runner
{
public:
  Runner();
  
  Runner(AdmittanceModel &model, VectorXd &q_init);

  void setTimer(int msec);

  void setParameters(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d);

  VectorXd getResult();

  void getResult(VectorXd &q_command, double &error);

  void calcNext();

private:
  void timer(int msec);

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
