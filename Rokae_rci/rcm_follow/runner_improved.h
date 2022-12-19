#pragma once

#include <mutex>
#include <condition_variable>
#include <thread>
#include "admittance_improved.h"

class Runner
{
public:
    Runner() = default;

    Runner(AdmittanceModel &model, VectorXd &q_init);

    void start();

    void stop();

    void setParameters(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d);

    VectorXd getResult();

    void getResult(VectorXd &q_command, double &error);

private:
    AdmittanceModel m_model;
    MatrixXd m_T, m_J;
    VectorXd m_tau, m_q_now, m_q_d, m_q_command;
    double m_error;
    bool m_ready, stop_flag;
    std::mutex mtx_p, mtx_r;
    std::condition_variable cv;
    std::thread t_runner;

    void calcNext();
};
