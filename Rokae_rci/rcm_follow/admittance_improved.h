#pragma once

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;

class AdmittanceModel
{
public:
    AdmittanceModel() = default;

    AdmittanceModel(int n, Vector3d &pc, double dt, MatrixXd &Lambda, MatrixXd &D, double Alpha, double Beta);

    VectorXd nextStep(MatrixXd &Tt, MatrixXd &Jt, VectorXd &tau, VectorXd &q, VectorXd &dq);

    double error(MatrixXd &T);

private:
    MatrixXd m_Lambda, m_D;
    Vector3d m_pc;
    double m_Alpha, m_Beta, m_dt;
    int m_n;

    MatrixXd Ax, A, A_inv, A_last, dA;
    MatrixXd Zx, Z, Z_inv, Z_inv_last, dZ_inv;
    MatrixXd B, G, h, u, Ji;
    Vector2d xc, dxc;
    Vector3d nt, pt, p;
    VectorXd ddq;
};
