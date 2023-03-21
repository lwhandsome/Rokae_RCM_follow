#pragma once

#include <iostream>
#include "Eigen/Dense"
#include "model_base.h"
#include "DEBUG.h"

using namespace Eigen;

class TaskPriorityModel : public ModelBase
{
public:
    TaskPriorityModel() = default;

    TaskPriorityModel(int n, Vector3d &p_trocar, Vector3d &m_p_desired, double dt, MatrixXd &K, MatrixXd &D);

    VectorXd nextStep(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q, VectorXd &dq) override;

    VectorXd error() override;

protected:
    MatrixXd pinv_eigen_based(Eigen::MatrixXd & origin);

private:
    MatrixXd m_K, m_D;
    Vector3d m_p_trocar, m_p_desired;
    VectorXd m_error;
    double m_dt;
    int m_n;
};

#ifdef DEBUG
void task_priority_debug();
#endif