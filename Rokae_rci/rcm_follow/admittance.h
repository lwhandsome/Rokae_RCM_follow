#pragma once

#include <iostream>
#include <ctime>
#include <Eigen/Dense>

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

