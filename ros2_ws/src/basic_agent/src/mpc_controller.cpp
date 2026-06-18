#include "basic_agent/mpc_controller.hpp"
#include <cstring>

MPCController::MPCController(int N, double dt,
                             double Q1, double Q2, double R_a, double R_j,
                             double a_min, double a_max,
                             double j_min, double j_max)
: N_(N), dt_(dt),
  Q1_(Q1), Q2_(Q2), R_a_(R_a), R_j_(R_j),
  a_min_(a_min), a_max_(a_max),
  j_min_(j_min), j_max_(j_max)
{
  A_ << 1, -dt_, 0,
        0,  1,   dt_,
        0,  0,   1;

  B_ << 0,
        0,
        dt_;

  buildPredictionMatrices();
}

void MPCController::buildPredictionMatrices()
{
  M_.resize(3 * N_, 3);
  C_.resize(3 * N_, N_);
  C_.setZero();

  Eigen::Matrix3d A_pow = A_;
  for (int i = 0; i < N_; i++) {
    M_.block(3 * i, 0, 3, 3) = A_pow;
    A_pow = A_pow * A_;
    for (int j = 0; j <= i; j++) {
      if (j == i) {
        C_.block(3 * i, j, 3, 1) = B_;
      } else {
        C_.block(3 * i, j, 3, 1) = A_ * C_.block(3 * (i - 1), j, 3, 1);
      }
    }
  }
}

void MPCController::buildQP(const Eigen::Vector3d& x0,
                              const Eigen::Vector3d& x_ref,
                              Eigen::MatrixXd& H,
                              Eigen::VectorXd& f,
                              bool position_penalty) {
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(3 * N_, 3 * N_);
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Identity(N_, N_) * R_j_;

  for (int i = 0; i < N_; i++) {
    Q_bar(3 * i,     3 * i)     = position_penalty ? Q1_ : 0.0;
    Q_bar(3 * i + 1, 3 * i + 1) = Q2_;
    Q_bar(3 * i + 2, 3 * i + 2) = R_a_;
  }

  Eigen::VectorXd X_ref(3 * N_);
  for (int i = 0; i < N_; i++) {
    X_ref.segment(3 * i, 3) = x_ref;
  }

  Eigen::VectorXd e0 = M_ * x0 - X_ref;

  H = 2.0 * (C_.transpose() * Q_bar * C_ + R_bar);
  f = 2.0 * C_.transpose() * Q_bar * e0;
}

double MPCController::compute(double s0, double v0, double a0, double v_ref, double s_ref, bool stopping) {
  Eigen::Vector3d x0(s0, v0, a0);
  Eigen::Vector3d x_ref;

  if (stopping) {
    x_ref << 0.0, 0.0, 0.0;
  } else {
    x_ref << s_ref, v_ref, 0.0;
  }

  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  buildQP(x0, x_ref, H, f, stopping);

  Eigen::SparseMatrix<double> H_sparse = H.sparseView();
  Eigen::MatrixXd A_accel(N_, N_);
  Eigen::MatrixXd M_accel(N_, 3);

  Eigen::SparseMatrix<double> A_con(2 * N_, N_);
  Eigen::VectorXd lb(2 * N_);
  Eigen::VectorXd ub(2 * N_);

  for (int i = 0; i < N_; i++) {
    A_accel.row(i) = C_.row(3 * i + 2);
    M_accel.row(i) = M_.row(3 * i + 2);
  }

  Eigen::VectorXd accel_offset = M_accel * x0;
  for (int i = 0; i < N_; i++) {
    A_con.insert(i, i) = 1.0;
    lb(i) = j_min_;
    ub(i) = j_max_;

    for (int j = 0; j < N_; j++) {
      if (A_accel(i, j) != 0.0) {
        A_con.insert(N_ + i, j) = A_accel(i, j);
      }
    }
    lb(N_ + i) = a_min_ - accel_offset(i);
    ub(N_ + i) = a_max_ - accel_offset(i);
  }

  // Clear previous data and solver state before re-initializing each cycle
  solver_.data()->clearHessianMatrix();
  solver_.data()->clearLinearConstraintsMatrix();
  solver_.clearSolver();

  solver_.data()->setNumberOfVariables(N_);
  solver_.data()->setNumberOfConstraints(2 * N_);
  solver_.data()->setHessianMatrix(H_sparse);
  solver_.data()->setGradient(f);
  solver_.data()->setLinearConstraintsMatrix(A_con);
  solver_.data()->setLowerBound(lb);
  solver_.data()->setUpperBound(ub);

  solver_.settings()->setVerbosity(false);
  solver_.settings()->setWarmStart(true);

  solver_.initSolver();
  solver_.solveProblem();

  Eigen::VectorXd U = solver_.getSolution();
  return a0 + U(0) * dt_;
}
