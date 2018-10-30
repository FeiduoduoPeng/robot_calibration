/*!	
	\file solver_utils.h
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Header of Solver_utils
*/
#ifndef SOLVER_UTILS_H
#define SOLVER_UTILS_H
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include <math.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/SVD>

#include <gsl/gsl_poly.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_complex.h>
#include <gsl/gsl_multimin.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_linalg.h>

#include <options/options.h>

#include "./calib_tuple.h"

#define sp(X)		( pow(X,2) )
#define cp(X)		( pow(X,3) )
#define fp(X)		( pow(X,4) )
#define vg(X,Y) 	( gsl_vector_get(X,Y) )
#define vs(X,Y,Z) 	( gsl_vector_set(X,Y,Z) )
#define mg(X,Y,Z) 	( gsl_matrix_get(X,Y,Z) )
#define ms(X,Y,Z,Q) 	( gsl_matrix_set(X,Y,Z,Q) )

using namespace std;

/*!
	\brief calculates condition number for a matrix
	@param M gsl_matrix pointer
	@return condition number of matrix M
*/
//** double cond_number(gsl_matrix * M);
double cond_number(const Eigen::MatrixXd &M);

/*!
	\brief Solves linear system \f$ Ax=g \f$, where A is a square matrix \f$ ( x = inv(A)g ) \f$
	@param A gsl_matrix pointer
	@param g gsl_vector pointer
	@param x gsl_vector pointer to store result
	@return 1 if succesfull, 0 otherwhise
*/
//** int solve_square_linear(gsl_matrix * A, gsl_vector * g, gsl_vector * x);
int solve_square_linear(const Eigen::MatrixXd &A, const Eigen::VectorXd &g, Eigen::VectorXd &x);
/*! 
	Compute eigenvalues and eigenvectors of a symmetric matrix M 
*/
/*!
	\brief Calculates eigenvalues and eigenvectors (sorted in ascending order) of a symmetric matrix
	@param M gsl_matrix pointer
	@param eigenvalues gsl_vector pointer
	@param eigenvectors gsl_vector pointer to store eigenvectors
	@return 1 if succesfull, 0 otherwise
*/
int eigenv(gsl_matrix * M, gsl_vector * eigenvalues, gsl_matrix * eigenvectors);

/*!
	\brief Calculates values of a quadratic function \f$ x'Mx \f$ for a given x
	@param x gsl_vector pointer
	@param M gsl_matrix pointer
	@return value of function
*/
//** double calculate_error(gsl_vector * x, gsl_matrix * M);
double calculate_error(const Eigen::VectorXd &x, const Eigen::VectorXd &M);

/*!
	\brief Calculate x solution of \f$ (M + \lambda W)x = 0 \f$; \lambda such that \f$ (M + \lambda W) \f$ is singular
	@param M gsl_matrix pointer
	@param lambda gsl_vector pointer
	@param W gsl_matrix pointer
	@return pointer to gsl_vector storing solution
	\note Caller must deallocate returned pointer
*/
//** gsl_vector * x_given_lambda(gsl_matrix * M, double lambda, gsl_matrix * W);
Eigen::VectorXd x_given_lambda(const Eigen::MatrixXd &M, double lambda, const Eigen::MatrixXd &W);

/*!
	\brief Solves \f$ (M + \lambda W)x = 0 \f$, M is a square matrix, \f$ W = \left[ \begin{array}{cc} 0_{3x3} & 0_{3x2} \\ 0_{2x3} & I_{2x2} \end{array} \right] \f$
	@param M gsl_matrix pointer
	@return pointer to gsl_vector storing solution
	\note Caller must deallocate returned pointer
*/
//** gsl_vector * full_calibration_min(gsl_matrix * M);
Eigen::VectorXd full_calibration_min(const Eigen::MatrixXd &M) ;

/*!
	\brief Numerically search minimum for a function \f$ x'Qx \f$, starting from a specified point, using symplex method
	@param init initial point for minimization
	@param min_found pointer to gsl_vector to store result
	@param Q pointer to gsl_matrix
	@param double(*f) is a pointer to function
	@return 1 if successfull, 0 otherwhise
*/
int fminimize_simplex(gsl_vector * init, gsl_vector * min_found, gsl_matrix * Q, double (*f) (const gsl_vector *z, void *params) );

/*!
	\brief Uses symplex method to estimate parameters
	@param H pointer to matrix of quadratic function (\f$ x'Hx \f$)
	@return pointer to gsl_vector storing minimum
*/
gsl_vector * numeric_calibration(gsl_matrix * H);

/*!
	\brief Implements a quadratic function (used in minimization functions)
	@param z pointer to point where the function has to be evaluated
	@param params pointer to parameters vector
*/
double f (const gsl_vector *z, void *params);


#endif