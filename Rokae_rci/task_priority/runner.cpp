#include "runner.h"

Runner::Runner(ModelBase &model, VectorXd &q_init)
{
    m_model = &model;
    m_q_desired = q_init;
    m_error = model.error();
    m_ready = false;
}

void Runner::setParameters(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q, VectorXd &dq)
{
    mtx_p.lock();
    m_T = T;
    m_J = J;
    m_tau =tau;
    m_q = q;
    m_dq = dq;
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
    return m_q_desired;
}

void Runner::getResult(VectorXd &q_desired, VectorXd &error)
{
    std::lock_guard<std::mutex> lock_r(mtx_r);
    q_desired = m_q_desired;
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
        VectorXd tau = m_tau, q = m_q, dq = m_dq;
        m_ready = false;
        lock_p.unlock();

        if (tau.array().abs().maxCoeff() > 0)
        {
            VectorXd q_desired = m_model->nextStep(T, J, tau, q, dq);
            VectorXd error = m_model->error();

            mtx_r.lock();
            m_q_desired = q_desired;
            m_error = error;
            mtx_r.unlock();
        }
    }
}
