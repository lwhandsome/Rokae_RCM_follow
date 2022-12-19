#include "runner.h"

Runner::Runner(AdmittanceModel &model, VectorXd &q_init)
{
    m_model = model;
    m_q_command = q_init;
    m_error = 0;
    m_ready = false;
}

void Runner::setParameters(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d)
{
    mtx_p.lock();
    m_T = T;
    m_J = J;
    m_tau =tau;
    m_q_now = q_now;
    m_q_d = q_d;
    m_ready = true;
    mtx_p.unlock();

    cv.notify_one();
}

void Runner::start()
{
    stop_flag = false;
    t_runner = std::thread(&Runner::calcNext, this);
}

void Runner::stop()
{
    stop_flag = true;
    cv.notify_one();
    t_runner.join();
}

VectorXd Runner::getResult()
{
    std::lock_guard<std::mutex> lock_r(mtx_r);
    return m_q_command;
}

void Runner::getResult(VectorXd &q_command, double &error)
{
    std::lock_guard<std::mutex> lock_r(mtx_r);
    q_command = m_q_command;
    error = m_error;
}

void Runner::calcNext()
{
    while(!stop_flag)
    {
        std::unique_lock<std::mutex> lock_p(mtx_p);
        cv.wait(lock_p, [this] {
            return m_ready || stop_flag;
        });
        MatrixXd T = m_T, J = m_J;
        VectorXd tau = m_tau, q_now = m_q_now, q_d = m_q_d;
        m_ready = false;
        lock_p.unlock();

        if (tau.array().abs().maxCoeff() > 2)
        {
            VectorXd q_command = m_model.nextStep(T, J, tau, q_now, q_d);
            double error = m_model.error(T);

            mtx_r.lock();
            m_q_command = q_command;
            m_error = error;
            mtx_r.unlock();
        }
    }
}
