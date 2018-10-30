/*!	
	\file solver_utils.cpp
	\author Carlo Masone, Margherita Petrocchi, Lorenzo Rosa
	\brief Implements methods for Solver_utils
*/
#include "../include/robot_calibration/solver_utils.h"
#include <complex>

using namespace std;

//计算矩阵的条件数．
//** double cond_number(gsl_matrix * M) 
double cond_number(const Eigen::MatrixXd &M) 
{
	using namespace Eigen;

	JacobiSVD<MatrixXd> svd(M);
	double cond = svd.singularValues()(0) / 
					svd.singularValues()( svd.singularValues().size()-1 );

	return cond;
}

//线性最小二乘的求解．
//**  int solve_square_linear(gsl_matrix * A, gsl_vector * g, gsl_vector * x) 
int solve_square_linear(const Eigen::MatrixXd &A, const Eigen::VectorXd &g, Eigen::VectorXd &x) 
{
	size_t sz = A.rows();
	if (A.cols() != sz) return 0;
	if (x.rows() != sz) return 0;
	if (g.rows() != sz) return 0;

	// x = A.reverse()*g; //this is not effecient;
	x = A.colPivHouseholderQr().solve(g);

	//gsl_matrix * LU 		= gsl_matrix_alloc(sz,sz);
	//gsl_permutation * p = gsl_permutation_alloc(sz);
	//gsl_matrix_memcpy(LU, A);
	//int signum;
	//gsl_linalg_LU_decomp (LU, p, &signum );
	//gsl_linalg_LU_solve  (LU, p, g, x );
	//gsl_matrix_free(LU);
	//gsl_permutation_free(p);

	return 1;
}

//计算矩阵的特征值．
int eigenv(gsl_matrix * M, gsl_vector * eigenvalues, gsl_matrix * eigenvectors) 
{
	//** size_t sz2 = M->size2;
	//** size_t sz  = M->size1;
	//** if (sz != sz2) return 0;
	//** gsl_eigen_symmv_workspace * wk = gsl_eigen_symmv_alloc(sz);
	//** gsl_matrix *M_tmp = gsl_matrix_alloc(sz, sz);
	
	//** gsl_matrix_memcpy(M_tmp, M);
	//** gsl_eigen_symmv(M_tmp, eigenvalues, eigenvectors, wk);	
	//** gsl_eigen_gensymmv_sort (eigenvalues, eigenvectors, GSL_EIGEN_SORT_ABS_ASC);
	
	//** gsl_matrix_free(M_tmp);
	//** gsl_eigen_symmv_free(wk);
	return 1;
}

//给定一个解x，计算误差．
//误差等于x^T M x
//**  double calculate_error(gsl_vector * x, gsl_matrix * M) 
double calculate_error(const Eigen::VectorXd &x, const Eigen::VectorXd &M) 
{
	return x.transpose() * M * x ;
}


//进行二次型的求解．
//** gsl_vector * full_calibration_min(gsl_matrix * M) 
Eigen::VectorXd full_calibration_min(const Eigen::MatrixXd &M) 
{
	using namespace Eigen;
	double 		m11 = M(0,0); //mg(M,0,0);
	double		m13 = M(0,2); //mg(M,0,2);
	double		m14 = M(0,3); //mg(M,0,3);
	double		m15 = M(0,4); //mg(M,0,4);
	double		m22 = M(1,1); //mg(M,1,1);
	double		m25 = M(1,4); //mg(M,1,4);
	double		m34 = M(2,3); //mg(M,2,3);
	double		m35 = M(2,4); //mg(M,2,4);
	double		m44 = M(3,3); //mg(M,3,3); 
	double		m55 = M(4,4); //mg(M,4,4);
	double a,b,c;

	/* 	Coefficienti del determinante M + lambda*W 	*/
    // M + lambda*W的行列式多项式的参数．－－公式见论文
	a = m11 * sp(m22) - m22 * sp(m13);

	b = 	  2 * m11 * sp(m22) * m44 - sp(m22) * sp(m14) 
		- 2 * m22 * sp(m13) * m44 - 2 * m11 * m22 * sp(m34) 
		- 2 * m11 * m22 * sp(m35) - sp(m22) * sp(m15) 
		+ 2 * m13 * m22 * m34 * m14 + sp(m13) * sp(m34) 
		+ 2 * m13 * m22 * m35 * m15 + sp(m13) * sp(m35);

	c = 	- 2 * m13 * cp(m35) * m15 - m22 * sp(m13) * sp(m44) + m11 * sp(m22) * sp(m44) 
		+ sp(m13) * sp(m35) * m44 + 2 * m13 * m22 * m34 * m14 * m44 + sp(m13) * sp(m34) * m44
		- 2 * m11 * m22 * sp(m34) * m44 - 2 * m13 * cp(m34) * m14 - 2 * m11 * m22 * sp(m35) * m44
		+ 2 * m11 * sp(m35) * sp(m34) + m22 * sp(m14) * sp(m35) - 2 * m13 * sp(m35) * m34 * m14
		- 2 * m13 * sp(m34) * m35 * m15 + m11 * fp(m34) + m22 * sp(m15) * sp(m34)
		+ m22 * sp(m35) * sp(m15) + m11 * fp(m35) - sp(m22) * sp(m14) * m44
		+ 2 * m13 * m22 * m35 * m15 * m44 + m22 * sp(m34) * sp(m14) - sp(m22) * sp(m15) * m44;

	/* 	Calcolo radice del polinomio 	*/
    // 对２次多项式进行求解，得到对应的两个解r1,r2--论文上的lamda1,lamda2

	double r0,r1;
	//gsl_poly_complex_solve_quadratic(a, b, c, r0, r1);	 //waiting for replace by eigen;

	if( pow(b,2) - 4*a*c < 0){
		sm_error("bad thing: imaginary solution\n");
		assert(false);
	}
    //如果有两个实根，才说明有解
	else{
		r0 = (-b + sqrt(pow(b,2) - 4*a*c) ) / (2*a);
		r1 = (-b - sqrt(pow(b,2) - 4*a*c) ) / (2*a);

		Matrix<double,5,5> W = Matrix<double,5,5>::Zero();
		W(3,3) = 1;
		W(4,4) = 1;

        //给定lamada1和lamada2之后，进行求解．
		VectorXd x0 = x_given_lambda(M, r0, W);
		VectorXd x1 = x_given_lambda(M, r1, W);

        //对于得到的两个解，选择一个误差最小的．其中,误差定义为 x' * M * x, 因为M是半正定的，所以误差一定是个非负数，不用加绝对值。
		//** double e0 = calculate_error(x0, M);
		//** double e1 = calculate_error(x1, M);
		double e0 = x0.transpose() * M * x0;
		double e1 = x0.transpose() * M * x0;
	
		return e0 < e1 ? x0 : x1;
	}
}

//给定lamda之后，进行求解．
//A = M + lamda * w
//A * x = 0
//对应的解为矩阵A的最小奇异值，对应的奇异向量
//A的奇异值即为A^TA的最小特征值，对应的特征向量

Eigen::VectorXd x_given_lambda(const Eigen::MatrixXd &M, double lambda, const Eigen::MatrixXd &W) 
{
	using namespace Eigen;

    // Z = M + lamda*W = A
	Matrix<double,5,5> Z = M + lambda*W;
	
    //矩阵A进行奇异值分解，该方程的解为最小奇异值对应的特征向量

	/* Calculate eigenvalues and eigenvectors */
	JacobiSVD<MatrixXd> svd(Z, ComputeThinU | ComputeThinV);

	//svd.singularValues();
	//svd.matrixU(); //left U matrix;
	//svd.matrixV(); //right V matrix;
    //对应的解v0
	VectorXd v0 = svd.matrixU().col(4);  //the singular values are sorted in decreasing order!
			
	/** Conditions: x1 > 0; x4^2 + x5^2 = 1 */
    //得到的解，必须满足约束条件．
	Vector2d temp_v( v0(3), v0(4) );

    //x4^2 + x5^2
	double norm = temp_v.norm(); //Frobenius Norm!
    //对初始解进行调整－－跟论文中一样
    //** double coeff = ( GSL_SIGN( v0(0) ) / norm );
	double coeff = sqrt( pow( v0(0), 2 ) ) / norm;

	v0 *= coeff;

    //返回最终的解．
	return v0;
}

/**
 * @brief fminimize_simplex
 * 进行迭代求解．
 * @param init          初始解
 * @param min_found
 * @param Q             对应的矩阵Qx = 0
 * @param f             误差函数或者说打分函数．
 * @return
 */
int fminimize_simplex(gsl_vector * init, gsl_vector * min_found, gsl_matrix * Q, double (*f) (const gsl_vector *z, void *params) ) 
{
	int sz = (int) init->size;
	if ( (Q->size1 != sz) || (Q->size2 != sz) ) return 0;
	const gsl_multimin_fminimizer_type * T = gsl_multimin_fminimizer_nmsimplex;
	gsl_multimin_fminimizer * s = gsl_multimin_fminimizer_alloc (T, sz);
	
	void *par = Q->data;
 	gsl_multimin_function func;
 	func.n = sz;
 	func.f = f;
	func.params = par;
	gsl_vector *step = gsl_vector_alloc(sz); 
	gsl_vector_set_all(step, 1e-12);
	
	int iter = 0;
	double size;
	int status;
	gsl_multimin_fminimizer_set (s, &func, init, step);

    do
    {
		iter++;
		status = gsl_multimin_fminimizer_iterate (s);
		if (status) break;

		size = gsl_multimin_fminimizer_size (s);

        status = gsl_multimin_test_size(size, 1e-8);
		
        if (status == GSL_SUCCESS)
        {
			sm_debug("Minimum found at:\n");
			for (int i = 0; i < sz; i++)
				sm_debug("%f \n", vg(s->x, i));
		}
	} while (status == GSL_CONTINUE && iter < 2000);
	
	if (status != GSL_SUCCESS) cout << "Symplex method: Maximum iteration reached before the minimum is found\n";
	gsl_vector_memcpy( min_found, gsl_multimin_fminimizer_x(s) );
	gsl_multimin_fminimizer_free (s);
	return 1;
}

//此方法为不转换成二次规划的方式来进行求解，而是使用迭代的方式来进行求解．
//即直接对二次型(phi^T M phi)进行求导,另其等于０
//M phi = 0;
//解为矩阵M最小奇异值对应的奇异向量
//得到的解要进行调整，使得其满足约束．
//在该解的基础上，进行进一步的迭代求解．
//gsl_vector * numeric_calibration(gsl_matrix * H)
//{
//    //svd分解
//	int sz = (int)(H->size1);
//	gsl_vector * eigenvalues  = gsl_vector_alloc(sz);
//	gsl_matrix * eigenvectors = gsl_matrix_alloc(sz,sz);
//	eigenv(H, eigenvalues, eigenvectors);
//	
//    //最小特征值对应的特征向量
//	gsl_vector * v0 = gsl_vector_alloc(sz);
//	gsl_matrix_get_col(v0, eigenvectors, sz-5);	/*Starting point for numerical search*/
//	
//    //进行调整，让其满足约束．
//	gsl_vector * tmp_v = gsl_vector_alloc(2);
//	vs(tmp_v , 0, vg(v0, sz-2));
//	vs(tmp_v , 1, vg(v0, sz-1));
//	double norm  = gsl_blas_dnrm2(tmp_v);
//	double coeff = ( GSL_SIGN( vg(v0,0) ) / norm );
//	gsl_vector_scale(v0 ,coeff);
//	gsl_vector_free(tmp_v);
//
//	gsl_vector * min = gsl_vector_alloc(sz);	/*vector where minimum will be stored*/
//	fminimize_simplex(v0, min, H, &f);
//		
//	return min;
//}

/**
	Function f(z) = [z(1) z(2) ... z(n)]' * M * [z(1) z(2) ... z(n)].
	Used for minimization in solver routine 
    这个函数是用来计算误差的函数．
    e = f(phi) = phi^T * M * phi;
*/

//double f (const gsl_vector *z, void *params)
double f (const Eigen::VectorXd &z, void *params)
{
	//using namespace Eigen;
	//int sz = (int) z.rows();
	//** gsl_vector * v = gsl_vector_alloc(sz);
	//** gsl_matrix * M = gsl_matrix_alloc(sz, sz);
	//** gsl_vector_memcpy(v, z);
	//Matrix<double,Dynamic,1> v = z;
	//MatrixXd M(sz,sz) = MatrixXd::Zero(sz,sz);
	
	//following code havn't been replaced!
	//double *p = (double *)params;
	//M->data = p;
	//double * res = new double;
	//gsl_vector * tmp = gsl_vector_calloc(sz);

	//gsl_blas_dgemv(CblasNoTrans, 1, M, v, 0, tmp);
	//gsl_blas_ddot(v ,tmp, res);
	//gsl_vector_free(tmp);	
 
  	//return( res[0] );
	return 0.0;
}


