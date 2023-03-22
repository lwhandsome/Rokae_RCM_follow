#pragma once

#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>
#include "model_base.h"

class Runner
{
public:
    Runner() = default;

    Runner(ModelBase &model, VectorXd &q_init);

    void start();

    void stop();

    void setParameters(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q, VectorXd &dq);

    VectorXd getResult();

    void getResult(VectorXd &q_desired, VectorXd &error);

private:
    ModelBase *m_model;
    MatrixXd m_T, m_J;
    VectorXd m_tau, m_q, m_dq, m_q_desired, m_error;
    bool m_ready, stop_flag;
    std::mutex mtx_p, mtx_r;
    std::condition_variable cv;
    std::thread t_runner;

    void calcNext();
};
