#include "../include/robot_calibration/solver2_meat.h"

//给定传感器数据tuple，进行参数的估计．
//此函数即实现了对应论文中的算法．
bool solve(const vector <calib_tuple>& tuple, CALIBR_MODE mode, double max_cond_number, struct calib_result& res){
    using namespace Eigen;
    /*!<!--####################		FIRST STEP: estimate J21 and J22  	#################-->*/
    /*!<!--####################		FIRST STEP: 估计J21和J22  	#################-->*/
    double J21, J22;
    Matrix2d  A = Matrix2d::Zero();
    Vector2d  g = Vector2d::Zero();
    //L_i　即theta 和 J成的线性关系
    //最终的式子为:A*J = g
    //A = L_i^{T} * L_i
    //g = L_i^T *
    Vector2d  L_i = Vector2d::Zero();

    //tuple的个数
    int n = (int)tuple.size();

    for (int i = 0; i < n; i++)
    {
        const calib_tuple &t = tuple[i];

        //L_i即为里程计的积分得到的角度参数，－－左右两轮分别积分
        L_i << t.T*t.phi_l, t.T * t.phi_r;
        A += L_i * L_i.transpose(); //A = A + L_i' * L_i; Asymmetric
        g += L_i * t.sm[2]; //g = g + L_i' * y_i; sm : {x, y, theta}
    }

    // Verify that A isn't singular
    // 求解A的条件数，看矩阵A是否是奇异的，如果是奇异的则无法求解．
    double cond = cond_number(A);
    if (cond > max_cond_number)
    {
        std::cout << "ERROR ----- Matrix A is almost singular. Not very exciting data!" << std::endl;
        std::cout << "max_cond_number: " << cond << ", max allowed: " << max_cond_number << std::endl;
        return false;
    }

    // Ay = g --> y = inv(A)g; A square matrix;
    // 求解线性最小二乘
    Vector2d y = A.colPivHouseholderQr().solve(g);

    //** if (!solve_square_linear(A, g, y))
    //** {
    //**     std::cout << "ERROR ----- Cannot solve Ay=g. Invalid arguments" << std::endl;
    //**     return false;
    //** }

    //得到J21和J22.
    J21 = y(0);
    J22 = y(1);

    //** if (gsl_isnan(J21) || gsl_isnan(J22))
    //** {
    //**     std::cout << "ERROR ----- Could not find first two parameters J21, J22 by solving linear equation." << std::endl;
    //**     sm_error("This is A: \n\n"); gsl_matrix_fwrite(stderr, A); fprintf(stderr, "\n");
    //**     sm_error("This is g: \n\t"); gsl_vector_fwrite(stderr, g); fprintf(stderr, "\n");
    //**     sm_error("This is y: \n\t"); gsl_vector_fwrite(stderr, y); fprintf(stderr, "\n");
    //**     return false;
    //** }

    //** gsl_vector_free(y);

    /*!<!--############## 		SECOND STEP: estimate the remaining parameters  		########-->*/
    /*!<!--############## 		SECOND STEP: 估计剩下的参数  		########-->*/
    // Build M, M2

    Matrix<double,5,5> M = Matrix<double,5,5>::Zero();
    Matrix<double,2,5> L_k = Matrix<double,2,5>::Zero();
    /*M2 and L_2k is used for another solution*/
    Matrix<double,6,6> M2 = Matrix<double,6,6>::Zero();
    Matrix<double,2,6> L_2k = Matrix<double,2,6>::Zero();

    double c, cx, cy, cx1, cx2, cy1, cy2, t1, t2;
    double o_theta, w0;

    for (int k = 0; k < n; k++)
    {
        const calib_tuple &t = tuple[k];

        //本次tuple对应的角度－－里程计积分
        //这个时间段内，里程计的角度值
        o_theta  = t.T * (J21*t.phi_l + J22*t.phi_r);

        //本次tuple对应的角速度－－里程计
        w0 = o_theta / t.T;

        if ( fabs(o_theta) > 1e-12 )
        {
            t1 = (   sin(o_theta)   / (o_theta) );
            t2 = ( (1-cos(o_theta)) / (o_theta) );
        }
        else
        {
            t1 = 1;
            t2 = 0;
        }

        //求解对应的c_x,c_y
        cx1 = 0.5 * t.T * (-J21 * t.phi_l) * t1;
        cx2 = 0.5 * t.T * (+J22 * t.phi_r) * t1;
        cy1 = 0.5 * t.T * (-J21 * t.phi_l) * t2;
        cy2 = 0.5 * t.T * (+J22 * t.phi_r) * t2;

        if ((mode == MODE_0)||(mode == MODE_1))
        {
            //cx,cy
            cx = cx1 + cx2;
            cy = cy1 + cy2;

            //构造Q矩阵．
            //** double array[] =
            //** {
            //**     -cx, 1-cos(o_theta),   sin(o_theta), t.sm[0], -t.sm[1],
            //**     -cy,  -sin(o_theta), 1-cos(o_theta), t.sm[1],  t.sm[0]
            //** };

            double array[] =
            {
                -cx,            -cy,
                1-cos(o_theta), -sin(o_theta),
                sin(o_theta),   1-cos(o_theta),
                t.sm[0],        t.sm[1],
                -t.sm[1],       t.sm[0]      
            };  //here, eigen3 fill the matrix with array from row to row! thus, array is writen as this form.

            Matrix<double,2,5> temp_matrix(array);
            L_k = temp_matrix;

            // M = M + L_k' * L_k; M is symmetric
            // 构造M矩阵，M = Q_k^{T} *  Q_k
            //** gsl_blas_dgemm (CblasTrans,CblasNoTrans, 1, L_k, L_k, 1, M);
            M += L_k.transpose() * L_k;
        }
        //另外的求解方法？这个好像在论文上没有讲过．
        else
        {
            //** double array2[] =
            //** {
            //**     -cx1, -cx2, 1-cos(o_theta), 	sin(o_theta), t.sm[0], -t.sm[1],
            //**     -cy1, -cy2,  -sin(o_theta), 1-cos(o_theta), t.sm[1],  t.sm[0]
            //** };
            double array2[] = {
                -cx1,           -cy1,
                -cx2,           -cy2,
                1-cos(o_theta), -sin(o_theta),
                sin(o_theta),   1-cos(o_theta),
                t.sm[0],        t.sm[1],
                -t.sm[1],       t.sm[0]
            };

            //** gsl_matrix_view tmp = gsl_matrix_view_array(array2, 2, 6);
            Matrix<double,2,6> temp_matrix(array2);
            L_2k = temp_matrix;

            // M2 = M2 + L_2k' * L_2k; M2 is symmetric
            M2 += L_2k.transpose()* L_2k;
        }
    }

    double est_b, est_d_l, est_d_r, laser_x, laser_y, laser_th;
    VectorXd x;

    switch (mode)
    {
        case MODE_0:	/*!<!--######### mode 0: minimize in closed form, using M ########-->*/
        {
            x = full_calibration_min(M);

            est_b = x(0);
            est_d_l = 2*(-est_b * J21);
            est_d_r  = 2*(est_b * J22);
            laser_x = x(1);
            laser_y = x(2);
            laser_th = atan2(x(4), x(3));
    
            break;
        }
        case MODE_1:	/*!<!--####  mode 1: start at the minimum eigenvalue, find numerical solution using M  ####-->*/
        {
            //x = numeric_calibration(M);   //pf: numeric_calibration hasn't been rewrited!

            est_b = x(0);
            est_d_l = 2 * (-est_b * J21);
            est_d_r = 2 * (est_b * J22);
            laser_x = x(1);
            laser_y = x(2);
            laser_th = atan2(x(4), x(3));
            break;
        }
        case MODE_2:	/*!<!--####  mode 2: start at the minimum eigenvalue, find numerical solution using M2 ####-->*/
        {
            //x = numeric_calibration(M2); //pf: numeric_calibration hasn't been rewrited!

            est_b = (x(0) + x(1))/2;
            est_d_l = 2*(-est_b * J21);
            est_d_r  = 2*(est_b * J22);
            laser_x = x(2);
            laser_y = x(3);
            laser_th = atan2(x(5), x(4));
            break;
        }
        default:
            assert(false);
            break;
    }

    res.axle = est_b;
    res.radius_l = est_d_l/2;
    res.radius_r = est_d_r/2;
    res.l[0] = laser_x;
    res.l[1] = laser_y;
    res.l[2] = laser_th;

    return true;
}

int solve_jackknife(const vector <calib_tuple>& tuples, CALIBR_MODE mode, double max_cond_number, struct calib_result& res)
{

    union{
        double tmp_params[6];
        struct calib_result tmp_res;
    };

    // check that the union trick is working
    tmp_res.radius_l = 0;
    tmp_res.radius_r = 1;
    tmp_res.axle = 2;
    tmp_res.l[0] = 3;
    tmp_res.l[1] = 4;
    tmp_res.l[2] = 5;

    for(int i = 0; i < 6; i++)
    {
        if(tmp_params[i] != i)
        {
            sm_error("ERROR! the union trick is not working");
            exit(-1);
        }
    }

    int n = tuples.size();
    vector<calib_tuple> minusone;

    for(int i = 1; i < n; i++)
    {
        minusone.push_back(tuples[i]);
    }

    double avg_minusone[6] = {0,0,0,0,0,0};

    for(int i = 0; i < n; i++)
    {
        // let's put back the i-th and remove the (i-1)-nth
        minusone[i % (n-1)] = tuples[i];

        if(!solve(minusone, mode, max_cond_number, tmp_res))
            return 0;

        for(int j = 0; j < 6; j++)
        {
            avg_minusone[j] += tmp_params[j]/n;
        }
    }

    // solve once again with original tuples
    if(!solve(tuples, mode, max_cond_number, tmp_res))
        return 0;

    double jack[6];
    for(int i = 0; i < 6; i++)
    {
        jack[i] = n*tmp_params[i] - (n-1)*avg_minusone[i];
    }

    const char*format = "%.10f %.10f %.10f %.10f %.10f %.10f\n";
    fprintf(stderr, "avg\t");
    fprintf(stderr, format,
            avg_minusone[0],  avg_minusone[1],  avg_minusone[2],
            avg_minusone[3],  avg_minusone[4],  avg_minusone[5]);

    fprintf(stderr, "ori\t");
    fprintf(stderr, format,
            tmp_params[0],  tmp_params[1],  tmp_params[2],
            tmp_params[3],  tmp_params[4],  tmp_params[5]);

    fprintf(stderr, "jack");

    fprintf(stderr, format,
            jack[0],  jack[1],  jack[2],
            jack[3],  jack[4],  jack[5]);

    return 1;
}

/**
 * @brief estimate_noise
 *        计算激光scan-match的噪声特性．
 * @param tuples
 * @param res
 * @param std_x
 * @param std_y
 * @param std_th
 */
void estimate_noise(std::vector<calib_tuple> &tuples, const calib_result &res, double &std_x, double &std_y, double &std_th)
{
    int n = tuples.size();
    double err_sm[3][n];

    //枚举数据
    for(int i = 0; i < tuples.size(); i++)
    {
        //计算跟标定结果的差．
        tuples[i].compute_disagreement(res);
        //差的平方的累加
        std_x += pow(tuples[i].err_sm[0],2);
        std_y += pow(tuples[i].err_sm[1],2);
        std_th += pow(tuples[i].err_sm[2],2);

        //** err_sm[0][i] = tuples[i].err_sm[0];
        //** err_sm[1][i] = tuples[i].err_sm[1];
        //** err_sm[2][i] = tuples[i].err_sm[2];
    }
    //** std_x  = gsl_stats_sd(err_sm[0],1,n);
    //** std_y  = gsl_stats_sd(err_sm[1],1,n);
    //** std_th = gsl_stats_sd(err_sm[2],1,n);

    //样本方差的无偏估计
    std_x /= (tuples.size()-1) ;
    std_y /= (tuples.size()-1) ;
    std_th /= (tuples.size()-1) ;
    sm_info("Estimated sm errors (std dev):  %g mm, %g mm, %g deg\n", std_x * 1000, std_y * 1000, rad2deg(std_th));
}
