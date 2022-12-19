#include "admittance.h"

using namespace Eigen;

AdmittanceModel::AdmittanceModel(){}

AdmittanceModel::AdmittanceModel(int n, Vector3d &pc, double dt, MatrixXd &Lambda, MatrixXd &D, double Alpha, double Beta): 
    m_n(n), m_pc(pc), m_dt(dt), m_Lambda(Lambda), m_D(D), m_Alpha(Alpha), m_Beta(Beta)
{
  A_last = MatrixXd::Zero(2, n);
  ZL_last = MatrixXd::Zero(n - 2, n);
}

VectorXd AdmittanceModel::nextStep(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d)
{
  MatrixXd B = T.block<3, 2>(0, 0);
  Vector3d pr = T.block<3, 1>(0, 3);
  MatrixXd A = calculateA(B, pr, J);
  MatrixXd Z = calculateZ(A);
  VectorXd q_d_expect = calculateQd(A, Z, B, pr, tau, q_d);
  
  VectorXd q = q_now + q_d_expect * m_dt;
  
  update(A, Z);

  return q;
}

double AdmittanceModel::error(MatrixXd &T)
{
  Vector3d pr = T.block<3, 1>(0, 3);
  Vector3d pt = T.block<3, 3>(0, 0) * Vector3d(0, 0, 1) + pr;
  
  return ((pr - pt).cross(pr - m_pc).norm() / (pr - pt).norm());
}

MatrixXd AdmittanceModel::calculateA(MatrixXd &B, Vector3d &pr, MatrixXd &J)
{
  Vector3d p = pr - m_pc;
  MatrixXd Jc = B.transpose() * MatrixXd{{1, 0, 0, 0, -p(2), p(1)}, 
                                          {0, 1, 0, p(2), 0, -p(0)}, 
                                          {0, 0, 1, -p(1), p(0), 0}};

  return Jc * J;
}

MatrixXd AdmittanceModel::calculateZ(MatrixXd &A)
{
  MatrixXd Z(m_n - 2, m_n);
  MatrixXd Am(2, 2), Ar(2, m_n - 2), Zm, Zr;
  int i, j;
  bool flag = false;
  for (i = 0; i < m_n - 1; i++)
  {
    for (j = i + 1; j < m_n; j++)
    { 
      if (A(0, i) * A(1, j) - A(0, j) * A(1, i) != 0) // full rank
      {
        flag = true;
        break;
      }
    }
    if (flag)
      {
        break;
      }
  }

  Am << A.block<2, 1>(0, i), A.block<2, 1>(0, j);
  Ar << A.block(0, 0, 2, i), A.block(0, i + 1, 2, j - i - 1), A.block(0, j + 1, 2, m_n - j - 1);
  Zm = -Ar.transpose() * Am.transpose().inverse();
  Zr = MatrixXd::Identity(m_n - 2, m_n - 2);

  Z << Zr.block(0, 0, m_n - 2, i), Zm.block(0, 0, m_n - 2, 1), Zr.block(0, i, m_n - 2, j - i - 1), Zm.block(0, 1, m_n - 2, 1), Zr.block(0, j - 1, m_n - 2, m_n - j - 1);
  return Z;
}

VectorXd AdmittanceModel::calculateQd(MatrixXd &A, MatrixXd &Z, MatrixXd &B, Vector3d &pr, VectorXd &tau, VectorXd &q_d_last)
{
  MatrixXd A_inv, Z_inv, Pf, A_d, ZL_d, Co1, Co2;
  VectorXd xc, q_d;
  A_inv = m_Lambda.inverse() * A.transpose() * (A * m_Lambda.inverse() * A.transpose()).inverse();
  Z_inv = m_Lambda * Z.transpose() * (Z * m_Lambda * Z.transpose()).inverse();
  Pf = Z_inv * Z;

  xc = B.transpose() * (pr - m_pc);
  A_d = (A - A_last) / m_dt;
  ZL_d = (Z * m_Lambda - ZL_last) / m_dt;

  Co1 = -m_Lambda.inverse() * (Z_inv * m_D * Z * m_Lambda + Z_inv * ZL_d) - A_inv * (A_d + 2 * m_Alpha * A);
  Co2 = m_Lambda.inverse() * (Pf * tau) - m_Beta * m_Beta * A_inv * xc;

  q_d = (MatrixXd::Identity(m_n, m_n) - Co1 * m_dt).inverse() * (q_d_last + Co2 * m_dt);
  return q_d;
}

void AdmittanceModel::update(MatrixXd &A, MatrixXd &Z)
{
  A_last = A;
  ZL_last = Z * m_Lambda;
}

Runner::Runner(){}

Runner::Runner(AdmittanceModel &model, VectorXd &q_init)
{
  m_model = model;
  m_q_command = q_init;
  m_error = 0;
  m_init = false;
}

void Runner::setTimer(int msec)
{
  t_timer = std::thread(&Runner::timer, this, msec);
  
}

void Runner::setParameters(MatrixXd &T, MatrixXd &J, VectorXd &tau, VectorXd &q_now, VectorXd &q_d)
{
  std::lock_guard<std::mutex> lock_p(mtx_p);
  m_T = T;
  m_J = J;
  m_tau =tau;
  m_q_now = q_now;
  m_q_d = q_d;
  m_init = true;
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
  while(1)
  {
    std::unique_lock<std::mutex> lock_c(mtx_c);
    cv.wait(lock_c, [this] {
      return m_init;
    });
    lock_c.unlock();
    mtx_p.lock();
    MatrixXd T = m_T, J = m_J;
    VectorXd tau = m_tau, q_now = m_q_now, q_d = m_q_d;
    mtx_p.unlock();
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

void Runner::timer(int msec)
{
  clock_t start, finish;
  t_calc = std::thread(&Runner::calcNext, this);
  
  while (1)
  {
    start = clock();
    std::this_thread::sleep_for(std::chrono::microseconds(20));
    mtx_c.lock();
    double totaltime = 0;
    while (1)
    {
      finish = clock();
      totaltime = (double)(finish - start) / CLOCKS_PER_SEC * 1000;
      if (totaltime > msec)
      {
        mtx_c.unlock();
        cv.notify_one();
        break;
      }
    }
  }
}

// int main()
// {
//   MatrixXd Lambda(DiagonalMatrix<double, 7>(1, 1, 1, 1, 1, 1, 1));
//   MatrixXd D(DiagonalMatrix<double, 5>(0.1, 0.1, 0.1, 0.1, 0.1));
//   double Alpha = 10, Beta = 10;
//   Vector3d pc(0.8833, 0, -0.0345);
//   AdmittanceModel model(7, pc, 0.01, Lambda, D, Alpha, Beta);
  
//   // test
//   const double pi = 3.14159265;
//   VectorXd q_now{{0, pi/3, 0, pi/3, 0, pi/6, 0}};
//   VectorXd q_d = MatrixXd::Zero(7, 1);
//   MatrixXd T{{-0.866, 0, 0.5, 0.7833}, {0, 1, 0, 0}, {-0.5, 0, -0.866, 0.1387}, {0, 0, 0, 1}};
//   MatrixXd J{{0, -0.2028, 0, -0.3998, 0, -0.2168, 0}, 
//             {0.7833, 0, 0.5673, 0, 0.1251, 0, 0}, 
//             {0, -0.7833, 0, -0.4421, 0, -0.1252, 0}, 
//             {0, 0, 0.866, 0, 0.866, 0, 0.5}, 
//             {0, 1, 0, 1, 0, 1, 0}, 
//             {1, 0, 0.5, 0, -0.5, 0, -0.866}};
//   VectorXd tau{{7.8333, -7.8333, 5.6727, -4.4212, 1.2515, -1.2515, 0}};
  
//   clock_t start, end;
//   start = clock();
//   VectorXd q = model.nextStep(T, J, tau, q_now, q_d);
//   double error = model.error(T);
//   end = clock();
//   std::cout << "time: " << (double)(end - start) / CLOCKS_PER_SEC << std::endl;
//   std::cout << "q: \n" << q << std::endl;
//   std::cout << "error: " << error << std::endl;
// }