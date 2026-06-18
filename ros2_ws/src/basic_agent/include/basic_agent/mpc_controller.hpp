#pragma once

#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>

class MPCController {
  public: 
    MPCController(int N, double dt, double Q1, double Q2, double R_a, double R_j, double a_min, double a_max, double j_min, double j_max);

    double compute(double s0, double v0, double a0, double v_ref, double s_ref, bool stopping);

  private:
    int N_;
    double dt_;
    double Q1_, Q2_, R_a_, R_j_;
    double a_min_, a_max_, j_min_, j_max_;

    Eigen::Matrix3d A_;
    Eigen::Vector3d B_;
    Eigen::MatrixXd M_;
    Eigen::MatrixXd C_;

    OsqpEigen::Solver solver_;

    void buildPredictionMatrices();
    void buildQP(const Eigen::Vector3d& x0, const Eigen::Vector3d& x_ref, Eigen::MatrixXd& H, Eigen::VectorXd& f, bool position_penalty);

};