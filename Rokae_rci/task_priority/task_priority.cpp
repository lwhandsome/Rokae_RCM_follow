#include "task_priority.h"

using namespace Eigen;

TaskPriorityModel::TaskPriorityModel(int n, Vector3d &p_trocar, Vector3d &p_desired, double dt, MatrixXd &K, MatrixXd &D, int mode = 0):
    m_n(n), m_p_trocar(p_trocar), m_p_desired(p_desired), m_dt(dt), m_K(K), m_D(D), m_mode(mode)
{
    if(n != 7) throw;
    // mode = 0: admittance control
    // mode = 1: impedance control
    if(mode != 0 && mode != 1) throw;
    m_error = MatrixXd::Zero(4, 1);
}

void TaskPriorityModel::changePositionDesired(Vector3d &p_desired)
{
    m_p_desired = p_desired;
}

VectorXd TaskPriorityModel::nextStep(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q, VectorXd &dq)
{
    // task1 : RCM constraint
    // 向量v：机械臂末端Z轴方向
    // 向量w：机械臂末端p_end指向p_trocar
    Vector3d v = T.block<3, 1>(0, 2);
    Vector3d w = m_p_trocar - T.block<3, 1>(0, 3);
    // x1 = -D
    // D：p_trocar到直线l的垂直距离，垂点为p_rcm，直线l由向量v和机械臂末端位置p_end定义
    // Lambda：p_rcm = p_end + Lambda * v
    double x1 = -v.cross(w).norm() / v.norm();
    double Lambda = w.dot(v) / (v.transpose() * v);

    // D_hat：p_rcm指向p_trocar的向量（归一化处理）
    Vector3d D_hat = w - Lambda * v;
    if(D_hat.norm() < 0.0001)
    {
        D_hat = MatrixXd::Ones(3, 1) / sqrt(3);
    }
    else
    {
        D_hat /= D_hat.norm();
    }

    // J1：J_rcm = D_hat' * (∂p_rcm/∂q)
    // ∂p_rcm/∂q由雅可比矩阵J直接推导
    MatrixXd p_ = MatrixXd{{0, -Lambda*v(2), Lambda*v(1)},
                            {Lambda*v(2), 0, -Lambda*v(0)},
                            {-Lambda*v(1), Lambda*v(0), 0}};
    MatrixXd J1 = D_hat.transpose() * (J.block<3, 7>(0, 0) - p_ * J.block<3, 7>(3, 0));
    
    // 计算dx1
    double dx1 = (J1 * dq)(0);

    // task2 : cartesian admittance control
    Vector3d x2 = T.block<3, 1>(0, 3) - m_p_desired;
    MatrixXd J2 = J.block<3, 7>(0, 0);
    MatrixXd J2_T = J2.transpose();
    // 计算dx2
    Vector3d dx2 = J2 * dq;

    // 记录误差
    m_error(0) = -x1;
    m_error.block<3, 1>(1, 0) = -x2;

    if(m_mode == 0)
    {
        // 更新dx1
        double ddx1 = -m_D(0, 0) * dx1 - m_K(0, 0) * x1;
        dx1 = dx1 + ddx1 * m_dt;
        
        // 更新dx2
        Vector3d ddx2 = -m_D.block<3, 3>(1, 1) * dx2 - m_K.block<3, 3>(1, 1) * x2;
        if (tau.array().abs().maxCoeff() > 5)
        {
            ddx2 += pinv_eigen_based(J2_T) * tau / 10;
        }
        dx2 = dx2 + ddx2 * m_dt;

        // task_priority 计算dq
        MatrixXd J1_inv = pinv_eigen_based(J1);
        MatrixXd J2_bar = J2 * (MatrixXd::Identity(7, 7) - J1_inv * J1);
        MatrixXd J2_bar_inv = pinv_eigen_based(J2_bar);
        VectorXd dq_next = J1_inv * dx1 + J2_bar_inv * (dx2 - J2 * (J1_inv * dx1));

        return q + dq_next * m_dt;
    }
    else if(m_mode == 1)
    {
        // task_priority 计算tau
        MatrixXd J1_inv = pinv_eigen_based(J1);
        MatrixXd J2_bar = J2 * (MatrixXd::Identity(7, 7) - J1_inv * J1);
        VectorXd tau = J1.transpose() * (m_D(0, 0) * dx1 + m_K(0, 0) * x1) + 
                    J2_bar.transpose() * (m_D.block<3, 3>(1, 1) * dx2 + m_K.block<3, 3>(1, 1) * x2);

        return tau;
    }
}

VectorXd TaskPriorityModel::error()
{
  return m_error;
}

MatrixXd TaskPriorityModel::pinv_eigen_based(MatrixXd & origin)
{
    // 进行SVD分解
    JacobiSVD<MatrixXd> svd_holder(origin, ComputeThinU | ComputeThinV);
    // 构建SVD分解结果
    MatrixXd U = svd_holder.matrixU();
    MatrixXd V = svd_holder.matrixV();
    MatrixXd D = svd_holder.singularValues();

    // 构建S矩阵
    MatrixXd S(V.cols(), U.cols());
    S.setZero();

    for (unsigned int i = 0; i < D.size(); ++i)
    {
        if (D(i, 0) > 0)
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

#ifdef DEBUG
void task_priority_debug()
{
    int n = 7;
    Vector3d p_trocar{{0.6582, 0, -0.0948}};
    Vector3d p_desired{{0.6082, 0, 0.1052}};
    double dt = 0.02;
    MatrixXd K = DiagonalMatrix<double, 4>(100, 10, 10, 10);
    MatrixXd D = DiagonalMatrix<double, 4>(20, 5, 5, 5);

    TaskPriorityModel t(n, p_trocar, p_desired, dt, K, D);

    MatrixXd T{{-1, 0, 0, 0.6582},
               {0, 1, 0, 0},
               {0, 0, -1, 0.1052},
               {0, 0, 0, 1}};
    MatrixXd J{{0, -0.2363, 0, -0.4333, 0, -0.2503, 0},
               {0.6582, 0, 0.5337, 0, 0.2168, 0, 0},
               {0, -0.6582, 0, -0.3170, 0, 0, 0},
               {0, 0, 0.8660, 0, 0.8660, 0, 0},
               {0, 1, 0, 1, 0, 1, 0},
               {1, 0, 0.5, 0, -0.5, 0, -1}};
    VectorXd tau{{0, 0, 0, 0, 0, 0, 0}};
    VectorXd q{{0, 1.0472, 0, 1.0472, 0, 1.0472, 0}};
    VectorXd dq{{0, 0, 0, 0, 0, 0, 0}};

    q = t.nextStep(T, J, tau, q, dq);

    std::cout << "q:" << std::endl << q << std::endl;
    std::cout << "error:" << std::endl << t.error() << std::endl;

    clock_t start_time = clock();
    for(int i = 0; i < 10000; i++)
    {
        VectorXd q_desired = t.nextStep(T, J, tau, q, dq);
        VectorXd error = t.error();
    }
    clock_t end_time = clock();

    std::cout << "spend time(10000 times): " << (double)(end_time - start_time) / CLOCKS_PER_SEC << std::endl;

}
#endif
