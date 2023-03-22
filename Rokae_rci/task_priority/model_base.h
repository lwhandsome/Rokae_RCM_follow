#pragma once

#include "Eigen/Dense"
using namespace Eigen;

class ModelBase
{
public:
    virtual VectorXd nextStep(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q, VectorXd &dq) = 0;
    virtual VectorXd error() = 0;
};
