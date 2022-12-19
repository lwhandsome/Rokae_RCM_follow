#include "admittance_improved.h"

using namespace Eigen;

AdmittanceModel::AdmittanceModel(int n, Vector3d &pc, double dt, MatrixXd &Lambda, MatrixXd &D, double Alpha, double Beta):
    m_n(n), m_pc(pc), m_dt(dt), m_Lambda(Lambda), m_D(D), m_Alpha(Alpha), m_Beta(Beta)
{
    if(n != 7) throw;

    Ji = MatrixXd::Zero(6, 6);

    Z = MatrixXd::Zero(n - 2, n);
    G = MatrixXd::Zero(n - 6, n);

    A_last = MatrixXd::Zero(2, n);
    Z_inv_last = MatrixXd::Zero(n, n - 2);
}

VectorXd AdmittanceModel::nextStep(MatrixXd &Tt, MatrixXd &Jt, VectorXd &tau, VectorXd &q, VectorXd &dq)
{
    B = Tt.block<3, 2>(0, 0);
    nt = Tt.block<3, 1>(0, 2);
    pt = Tt.block<3, 1>(0, 3);

    p = pt - m_pc;

    Ax = B.transpose() * MatrixXd{{1, 0, 0, 0, -p(2), p(1)},
                                   {0, 1, 0, p(2), 0, -p(0)},
                                   {0, 0, 1, -p(1), p(0), 0}};

    A = Ax * Jt;

    Zx = MatrixXd{{nt(0), nt(1), nt(2), 0, 0, 0},
                   {0, -p(2), p(1), 1, 0, 0},
                   {p(2), 0, -p(0), 0, 1, 0},
                   {-p(1), p(0), 0, 0, 0, 1}};

    // 仅对于n = 7
    for(int i = 0; i < 7; i++)
    {
        Ji << Jt.block(0, 0, 6, i), Jt.block(0, i + 1, 6, 6 - i);
        G(0, i) = pow(-1, i) * Ji.determinant(); // -1^(n+i+1)
    }


    Z << Zx * (Jt * Jt.transpose()).inverse() * Jt, G;

    A_inv = m_Lambda.inverse() * A.transpose() * (A * m_Lambda.inverse() * A.transpose()).inverse();
    Z_inv = m_Lambda * Z.transpose() * (Z * m_Lambda * Z.transpose()).inverse();

    dxc = A * dq;
    xc = B.transpose() * p;
    dA = (A - A_last) / m_dt;
    dZ_inv = (Z_inv - Z_inv_last) / m_dt;

    h = dA * dq + 2 * m_Alpha * dxc + m_Beta * m_Beta * xc;
    u = (m_D * Z_inv.transpose() + dZ_inv.transpose()) * dq;

    ddq = -(A_inv * h + Z.transpose() * u) + Z.transpose() * Z * tau;

    A_last = A;
    Z_inv_last = Z_inv;

    return q + (dq + ddq * m_dt) * m_dt;
}

double AdmittanceModel::error(MatrixXd &T)
{
  Vector3d pt = T.block<3, 1>(0, 3);
  Vector3d nt = T.block<3, 1>(0, 2);

  return nt.cross(pt - m_pc).norm();
}



