// Utilities for computing jacobian of a function

#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_deriv.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>
//** gsl_matrix * gsl_jacobian( gsl_vector * (*function)(const gsl_vector* x, void*params), const gsl_vector *x, void*params);

Eigen::MatrixXd gsl_jacobian(
    Eigen::MatrixXd (*function)(const Eigen::Matrix<double,Eigen::Dynamic,1> &x, void *params),
    const Eigen::Matrix<double, Eigen::Dynamic, 1> &x,
    void *params);