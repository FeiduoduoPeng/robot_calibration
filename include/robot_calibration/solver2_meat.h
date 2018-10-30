#ifndef SOLVER2_MEAT_H
#define SOLVER2_MEAT_H

#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <iostream>
#include <algorithm>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

#include <options/options.h>
#include <csm/csm_all.h>

#include <gsl/gsl_linalg.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_statistics_double.h>

#include "./solver_utils.h"

using namespace std;

//**  double f (const gsl_vector *z , void *params);
double f (const Eigen::VectorXd &z, void *params);


bool solve(const vector <calib_tuple>& tuple, CALIBR_MODE mode, double max_cond_number, struct calib_result& res);

int solve_jackknife(const vector <calib_tuple>& tuples, CALIBR_MODE mode, double max_cond_number, struct calib_result& res);

void estimate_noise(std::vector<calib_tuple>&tuples, const calib_result&res, double&std_x, double&std_y, double&std_th);

#endif // SOLVER2_MEAT_H
