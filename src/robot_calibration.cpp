#include "../include/robot_calibration/robot_calibration.h"
#include "../include/robot_calibration/solver2_meat.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/LU>

namespace robot_calibration
{
//进行数据手机和参数初始化
RobotCalibrater::RobotCalibrater():data_collection_(false), last_scan_init_(false)
{
    ros::NodeHandle nh;

    //参数必须要这个．
    ros::NodeHandle private_param("~");

    //MODE_0即为论文中介绍的解法
    calibrat_mode_ = MODE_0;

    //solver1表示该方法没有outlier去除．
    //solver2表示该方法有outlier去除
    calibrat_solver_ = SOLVER_2;

    //修正时间戳不同步，激光数据的时间戳矫正．防止目前发送过来的激光数据的时间戳是错的．
    laser_stamp_offset_ = 0.0;

    //条件数，默认为50
    private_param.param("MaxConditionNumber",max_cond_number_,50.0);

    std::cout <<"Condition Number:"<<max_cond_number_<<std::endl;

    //outlier迭代次数，默认为4
    private_param.param("OutlierIterations",outliers_iterations_,4);
    std::cout <<"OutlierIterations:"<<outliers_iterations_<<std::endl;

    //每次去除的outlier数据的比例，默认为0.02
    private_param.param("OutlierPercentage",outliers_percentage_,0.02);
    std::cout <<"OutlierPercentage:"<<outliers_percentage_<<std::endl;

    private_param.param("DistThreshold",m_DistThreshold,0.05);
    std::cout <<"DistThreshold:"<<m_DistThreshold<<std::endl;

    private_param.param("AngleThreshold",m_AngleThreshold,3.0);
    std::cout <<"AngleThreshold:"<<m_AngleThreshold<<" degree"<<std::endl;

    //是否使用championScan的数据，默认为不使用．
    private_param.param("IsUseChampionScan",is_use_champion_scan_,false);

    //开始和停止数据收集
    start_data_collection_sub_ = nh.subscribe<std_msgs::Empty>("start_data_collection", 1, &RobotCalibrater::startCollectionCallback, this);
    end_data_collection_sub_ = nh.subscribe<std_msgs::Empty>("end_data_collection", 1, &RobotCalibrater::endCollectionCallback, this);

    //进行里程计数据(里程计数据为线信息，因为半径需要标定)和激光数据的订阅
    odom_data_sub_ = nh.subscribe<champion_nav_msgs::OdomRadData>("odom_rad_info", 1, &RobotCalibrater::getOdomDataCallback, this);

    //判断需要哪一种数据信息．
    if(is_use_champion_scan_ == false)
    {
        std::cout <<"Use ROS Scan!!!!"<<std::endl;
        laser_data_sub_ = nh.subscribe<sensor_msgs::LaserScan>("scan", 5, &RobotCalibrater::getLaserCallback, this);
    }
    else
    {
        std::cout <<"Use Champion Scan!!!"<<std::endl;
        champion_laser_data_sub_= nh.subscribe<champion_nav_msgs::ChampionNavLaserScan>("scan",5,&RobotCalibrater::getChampionLaserCallback, this);
    }
    //请求进行标定的信息．
    calibrat_request_sub_ = nh.subscribe<std_msgs::Empty>("request_calibra", 1, &RobotCalibrater::requestCalibraCallback, this);

}

RobotCalibrater::~RobotCalibrater()
{}

void RobotCalibrater::startCollectionCallback(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "Calibration ----- Start Data Collection..." << std::endl;
    data_collection_ = true;
    return;
}

void RobotCalibrater::endCollectionCallback(const std_msgs::Empty::ConstPtr& msg)
{
    std::cout << "Calibration ----- End Data Collection..." << std::endl;
    data_collection_ = false;

    std::cout <<"Odom Data Size:"<<odom_data_.size()<<std::endl;
    std::cout <<"LaserData Size:"<<laser_data_.size()<<std::endl;
    return;
}

/*
 * 得到时刻t时候 机器人在里程计坐标下的坐标
*/
bool RobotCalibrater::getOdomPose(Eigen::Vector3d& odomPose,const ros::Time& t)
{
    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, "base_link");
    tf::Stamped<tf::Transform> odom_pose;

    // get the global pose of the robot
    try
    {
        //if(m_transformListener.waitForTransform("/odom","/base_link",t,ros::Duration(0.2)))
        //{
        //    ROS_ERROR("Can not Wait Transform from /base_link to /odom");
        //    return false;
        //}
        //m_transformListener.waitForTransform("odom","base_link",t,ros::Duration(0.1));
        //m_transformListener.transformPose("odom", ident, odom_pose);
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR("calibration_node:No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ConnectivityException& ex)
    {
        ROS_ERROR("calibration_node:Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR("calibration_node:Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }

    //得到里程计位姿．
    double yaw = tf::getYaw(odom_pose.getRotation());
    odomPose << odom_pose.getOrigin().x(),odom_pose.getOrigin().y(),yaw;

    return true;
}

//里程计数据的回调函数
void RobotCalibrater::getOdomDataCallback(const champion_nav_msgs::OdomRadData::ConstPtr& msg)
{
    if(!data_collection_)
        return;

    odomData received_data;
    if (msg->reset_integra)
    {
        received_data.first_data = true;
    }
    else
    {
        received_data.first_data = false;
    }

    //记录下来时间戳和对应的左右轮的tick信息
    received_data.time_stamp = msg->header.stamp.toSec();
    received_data.link_rad_position = msg->link_rad;
    received_data.right_rad_position = msg->right_rad;

    //放置到队列中储存
    odom_data_.push_back(received_data);

    if(odom_data_.size() % 100 == 0)
        std::cout <<"Odom Data Size:"<<odom_data_.size()<<std::endl;
}

//把ros的激光雷达数据类型转换到generallaserscan
void RobotCalibrater::rosLaserScanToGeneralLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_scan,GeneralLaserScan & general_scan)
{
    general_scan.m_framdID = ros_scan->header.frame_id;
    general_scan.m_stamp = ros_scan->header.stamp;

    general_scan.m_rangeMax = ros_scan->range_max;
    general_scan.m_rangeMin = ros_scan->range_min;
    general_scan.m_rangeReadings.resize(ros_scan->ranges.size());
    for(int i = 0; i<ros_scan->ranges.size();i++)
    {
        general_scan.m_rangeReadings[i] = ros_scan->ranges[i];
    }

    general_scan.m_angleMax = ros_scan->angle_max;
    general_scan.m_angleMin = ros_scan->angle_min;
    general_scan.m_angleIncrement = ros_scan->angle_increment;
    general_scan.m_angleReadings.resize(ros_scan->ranges.size());
    for(int i = 0; i<ros_scan->ranges.size();i++)
    {
        general_scan.m_angleReadings[i] = ros_scan->angle_min + i * ros_scan->angle_increment;
    }
}

//把champion的激光雷达数据类型转换到generallaserscan
void RobotCalibrater::championLaserScanToGeneralLaserScan(const champion_nav_msgs::ChampionNavLaserScan&champion_scan,GeneralLaserScan & general_scan)
{
    general_scan.m_framdID = champion_scan.header.frame_id;
    general_scan.m_stamp = champion_scan.header.stamp;

    general_scan.m_rangeMax = champion_scan.range_max;
    general_scan.m_rangeMin = champion_scan.range_min;
    general_scan.m_rangeReadings.resize(champion_scan.ranges.size());
    for(int i = 0; i<champion_scan.ranges.size();i++)
    {
        general_scan.m_rangeReadings[i] = champion_scan.ranges[i];
    }

    general_scan.m_angleMax = champion_scan.angle_max;
    general_scan.m_angleMin = champion_scan.angle_min;
    general_scan.m_angleIncrement = (champion_scan.angle_max - champion_scan.angle_min) / champion_scan.angles.size();
    general_scan.m_angleReadings.resize(champion_scan.angles.size());
    for(int i = 0; i<champion_scan.angles.size();i++)
    {
        general_scan.m_angleReadings[i] = champion_scan.angles[i];
    }
}

//激光的回调函数
void RobotCalibrater::getLaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    if(!data_collection_)
        return;

    GeneralLaserScan general_scan;
    rosLaserScanToGeneralLaserScan(msg,general_scan);

    //得到激光匹配的里程计数据
    Eigen::Vector3d odomPose;
    bool isGood = getOdomPose(odomPose,general_scan.m_stamp);
    if(isGood == false )return ;

    laser_data_.push_back(general_scan);
    m_IntegOdomData.push_back(odomPose);

    if(laser_data_.size() % 50 == 0)
        std::cout <<"Laser Data Size:"<<laser_data_.size()<<std::endl;
}

void RobotCalibrater::getChampionLaserCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg)
{
    if(data_collection_ == false)
        return ;

    GeneralLaserScan general_scan;
    championLaserScanToGeneralLaserScan(*msg,general_scan);

    //得到激光匹配的里程计数据
    Eigen::Vector3d odomPose;
    bool isGood = getOdomPose(odomPose,general_scan.m_stamp);
    if(isGood == false )return ;

    //激光和匹配的积分里程计数据．
    laser_data_.push_back(general_scan);
    m_IntegOdomData.push_back(odomPose);

    if(laser_data_.size() % 50 == 0)
        std::cout <<"Laser Data Size:"<<laser_data_.size()<<std::endl;
}

//请求标定flag的回调函数
void RobotCalibrater::requestCalibraCallback(const std_msgs::Empty::ConstPtr& msg)
{
    //在数据收集阶段，不能标定．
    if (data_collection_)
    {
        std::cout << "ERROR ----- Please stop receive data first..." << std::endl;
        return;
    }

    //数据必须要＞＝２
    if (laser_data_.size() <= 1)
    {
        std::cout << "ERROR ----- Laser Data Size <= 1..." << std::endl;
        return;
    }

    bool find_first_scan = false;
    int avail_scan_cout_01 = 0, avail_scan_cout_02 = 0;
    int under_bound = -1, up_bound = -1;
    double laser_ref_ts, laser_sens_ts;

    GeneralLaserScan nowScan,prevScan;
    vectorScanPoseObser vectorObser;

    calibration_data_.clear();

    double test_odom_angle = 0.0;
    double test_odom_dist = 0.0, test_scan_dist = 0.0;

    std::cout << "----------" << std::endl;
    std::cout << "Syncronization..." << std::endl;
    std::cout << "laser_data_ Size: " << laser_data_.size() << std::endl;

    //进行角度的标定需要的参数
    std::vector<double> odomRelativeAngles;
    std::vector<double> laserRelativeAngles;

    int cnt0 = 0,icpFailCnt = 0;
    int refIndex;
    //枚举激光数据
    for(int idx = 0; idx < laser_data_.size(); idx++)
    {
        nowScan = laser_data_[idx];
        //如果是第一帧数据
        if(find_first_scan == false)
        {
            //得到激光的时间戳．
            laser_ref_ts =  nowScan.m_stamp.toSec() + laser_stamp_offset_;

            //找到离该激光数据最近的里程计数据，下界
            under_bound = findClosestRef(laser_ref_ts, odom_data_);

            //如果没有，则该数据不能要，因此不能插值
            if (under_bound != -1)
            {
                find_first_scan = true;
                prevScan = nowScan;
                refIndex = idx;
                continue;
            }
        }

        //找到该激光数据时间戳下界和上界的里程计信息．
        laser_sens_ts = nowScan.m_stamp.toSec() + laser_stamp_offset_;
        up_bound = findClosestSens(laser_sens_ts, odom_data_);

        under_bound = findClosestRef(laser_ref_ts, odom_data_);

        avail_scan_cout_01++;

        //没有找到对应的里程计数据，则直接忽略该数据．
        if(up_bound == -1 || under_bound == -1)continue;

        //如果车子中途复位，则需要找到第一个数据．
        //因此如果这中间如果出现了这个东西，说明是车子中途复位了．
        for(int i = under_bound+1; i <= up_bound-1; i++)
        {
            if (odom_data_[i].first_data)
            {
                find_first_scan = false;
                continue;
            }
        }

        //得到对应的积分里程计数据．
        Eigen::Vector3d prevOdomPose = m_IntegOdomData[refIndex];
        Eigen::Vector3d nowOdomPose  = m_IntegOdomData[idx];

        //判断走过的距离，必须要超过5cm和3度．
        double dlinear = std::pow(nowOdomPose(0) - prevOdomPose(0),2) + std::pow(nowOdomPose(1) - prevOdomPose(1),2);
        double dtheta = std::fabs(tfNormalizeAngle(nowOdomPose(2) - prevOdomPose(2)));
        if(dlinear < m_DistThreshold * m_DistThreshold && dtheta < tfRadians(m_AngleThreshold))
        {
            cnt0++;
            continue;
        }

        /////////////里程计积分
        Point2 radIncPoint = computeRadInc(laser_ref_ts, laser_sens_ts, under_bound, up_bound);
        double left_inc = radIncPoint.x;
        double right_inc = radIncPoint.y;
        if((left_inc == 0) && (right_inc == 0))
        {
            continue;
        }

        //两帧激光时间间隔－－间隔太大，则不要
        double t_k = laser_sens_ts - laser_ref_ts; //T

        //TODO 加入其他条件，比如合法激光数

        //计算轮子平均速度 (弧度)
        double left_speed = left_inc/t_k; //phi_l
        double right_speed = right_inc/t_k; //phi_r

        //这里估算一个车体运动在上一帧坐标系下运动的距离，给 ICP 初始解
        double d_t = (right_inc + left_inc)*DEFAULT_RAD_RADIUS/2.0;
        //double alpha_t = (right_inc - left_inc)*DEFAULT_RAD_RADIUS/DEFAULT_AXIS_DIST;

        //角度用陀螺仪来进行计算．
        double alpha_t = tfNormalizeAngle( nowOdomPose(2) - prevOdomPose(2) );

        //运动的距离－－车体运动的距离
        double x_t = d_t*cos(alpha_t);
        double y_t = d_t*sin(alpha_t);

        //雷达运动的距离
        double x_tt = (d_t+DEFAULT_LASER_MITTEL_DIST)*cos(alpha_t) - DEFAULT_LASER_MITTEL_DIST;
        double y_tt = (d_t+DEFAULT_LASER_MITTEL_DIST)*sin(alpha_t);

        //雷达的初始位姿
        Pose2 initPose(x_tt,y_tt,alpha_t);

        //把当前的激光数据转换为 pi-icp能识别的数据
        LDP currentLDP, prevLDP;
        GeneralScanToLDP(nowScan,currentLDP);
        GeneralScanToLDP(prevScan,prevLDP);

        //进行ICP匹配来计算雷达的相对位姿
        Pose2 relativPose;
        if (PIICPBetweenTwoFrames(currentLDP, prevLDP, nowScan, initPose, relativPose))
        {
            //构建用于矫正的数据结构－－包含dt,Vl,Vr,S_k
            calibrationData calib_data(t_k, left_speed, right_speed, relativPose);
            calibration_data_.push_back(calib_data);
            avail_scan_cout_02++;

            //存储用于诊断的输出
            scanPoseObser obs_data;
            obs_data.laser_ref_ts = laser_ref_ts;
            obs_data.laser_sens_ts = laser_sens_ts;
            obs_data.d_t = d_t;
            obs_data.alpha_t = alpha_t;
            obs_data.initPose = initPose;
            obs_data.relativPose = relativPose;
            vectorObser.push_back(obs_data);

            //For TEST
            test_odom_angle += alpha_t/3.1415926*180;
            test_odom_dist += d_t;
            double delta_x = relativPose.GetX();
            double delta_y = relativPose.GetY();
            test_scan_dist += std::sqrt(delta_x*delta_x + delta_y*delta_y);

            //存储激光雷达的相对角度和里程计的相对角度，用来进行陀螺仪的标定．
            odomRelativeAngles.push_back( tfNormalizeAngle(nowOdomPose(2) - prevOdomPose(2)));
            laserRelativeAngles.push_back(relativPose.GetHeading());
        }
        else
        {
            icpFailCnt++;
            test_scan_dist += std::sqrt(x_t*x_t + y_t*y_t);
        }

        prevScan = nowScan;
        refIndex = idx;
        laser_ref_ts = laser_sens_ts;

        //释放
        ld_free(currentLDP);
        ld_free(prevLDP);
    }

    std::cout <<"icpFailCnt:"<<icpFailCnt<<std::endl;

    //到这里准备好了，所有的标定数据．
    std::cout << "calibration_data Size: " << calibration_data_.size() << std::endl;

    //For TEST
    std::cout << "Odom Angle: " << test_odom_angle << std::endl;
    std::cout << "Odom Dist: " << test_odom_dist << std::endl;
    std::cout << "Scan Dist: " << test_scan_dist << std::endl;

    std::cout << "Total Scan Size: " << laser_data_.size() << std::endl;
    std::cout << " avail. Scan 01: " << avail_scan_cout_01 << std::endl;
    std::cout << " avail. Scan 02: " << avail_scan_cout_02 << std::endl;
    std::cout << "Sychron Data Size: " << calibration_data_.size() << std::endl;

    //写入文件
    writeToSychronDataFile(calibration_data_);
    //激光匹配中间数据写入文件
    writeToScanObserDataFile(vectorObser);

    //根据选择的标定算法来进行标定．
    std::cout << "----------" << std::endl;
    std::cout << "Calibrating..." << std::endl;
    if(calibrat_solver_ = SOLVER_1)
        computeCalibraParam_01(calibration_data_);
    else if(calibrat_solver_ = SOLVER_2)
        computeCalibraParam_02(calibration_data_);
    else
        std::cout << "ERROR ----- Undefined Solver!" << std::endl;

    odom_data_.clear();
    laser_data_.clear();

    //进行里程计标定
    double sumFactor = 0.0;
    int calibDataNumber = 0;
    for(int i = 0; i < odomRelativeAngles.size();i++)
    {
        double a = odomRelativeAngles[i];
        double b = laserRelativeAngles[i];

        if(std::fabs(a) < tfRadians(0.5) || std::fabs(b) < tfRadians(0.5))
        {
            continue;
        }

        sumFactor += b/a;
        calibDataNumber++;
    }
    std::cout <<"Data Use for Angle Calibration:"<<calibDataNumber<<std::endl;

    if(calibDataNumber == 0)
    {
        std::cout <<"Angle Calibration Failed!,Not Enought Data"<<std::endl;
        return ;
    }

    double xFactor = sumFactor / calibDataNumber;
    std::cout <<"Angle Factor:"<<xFactor<<std::endl;
}

bool RobotCalibrater::writeToSychronDataFile(vectorCalibration calibration_data)
{
    //打开要写入的文件
    std::string sychrondatafile = "/home/eventec/robot_calibration/data/sychron_data";

    std::ofstream out_sychron(sychrondatafile.c_str());

    if (!out_sychron)
    {
        out_sychron.close();

        std::cout << "ERROR ----- Cannot Write to Sychrondata" << std::endl <<
                     "Filename: " << sychrondatafile << std::endl;
        return false;
    }

    for(vectorCalibration::iterator it = calibration_data.begin();
        it != calibration_data.end(); it++)
    {
        out_sychron << "T: " << (*it).T << ", psi_l: " << (*it).left_speed <<
                       ", psi_r: " << (*it).right_speed << ", SM: [" <<
                       (*it).sm.GetX() << ", " << (*it).sm.GetY() << ", " <<
                       (*it).sm.GetHeading() << "]" << std::endl;
    }

    out_sychron.close();
}

bool RobotCalibrater::writeToScanObserDataFile(vectorScanPoseObser vectorObser)
{
    //打开要写入的文件
    std::string scanobserdatafile = "/home/eventec/robot_calibration/data/scan_obser_data";

    std::ofstream out_scanobser(scanobserdatafile.c_str());

    if (!out_scanobser)
    {
        out_scanobser.close();

        std::cout << "ERROR ----- Cannot Write to Sychrondata" << std::endl <<
                     "Filename: " << scanobserdatafile << std::endl;
        return false;
    }

    for(vectorScanPoseObser::iterator it = vectorObser.begin();
        it != vectorObser.end(); it++)
    {
        out_scanobser << "----------" << std::endl;
        out_scanobser << "first scan time: " << (*it).laser_ref_ts <<
                         ", second scan time: " << (*it).laser_sens_ts <<
                         ", odom_shift: " << (*it).d_t << ", odom_angle: " <<
                          (*it).alpha_t << std::endl;
        out_scanobser << "init Pose: " << (*it).initPose.GetX() << ", " <<
                         (*it).initPose.GetY() << ", " <<
                         (*it).initPose.GetHeading() << std::endl;
        out_scanobser << "relativ Pose: " << (*it).relativPose.GetX() << ", " <<
                         (*it).relativPose.GetY() << ", " <<
                         (*it).relativPose.GetHeading() << std::endl;
        out_scanobser << "----------" << std::endl << std::endl;
    }

    out_scanobser.close();
}

//用算法1来进行标定．
//感觉这个算法就是不用outlier去除模块．
//直接进行参数的计算，这个函数里面实现的功能就是solver_meat.cpp里面的solver()函数的功能．
//显然这样的速度会快很多，但是对噪声不robust

bool RobotCalibrater::computeCalibraParam_01(vectorCalibration calibration_data)
{
    //int n = calibration_data.size();
    //double J21, J22;
    //double T, phi_l, phi_r;
    //double sm[3];
    //double c, cx, cy, cx1, cx2, cy1, cy2, t1, t2;

    //gsl_vector * g = gsl_vector_calloc(2);
    //gsl_vector * L_i = gsl_vector_alloc(2);

    //gsl_matrix * A = gsl_matrix_calloc(2,2);
    //gsl_matrix * M = gsl_matrix_calloc(5,5);
    //gsl_matrix * M2 = gsl_matrix_calloc(6,6);
    //gsl_matrix * L_k = gsl_matrix_calloc(2,5);
    //gsl_matrix * L_2k = gsl_matrix_calloc(2,6);

    //for(int i = 0; i < n; i++)
    //{
    //    T = calibration_data[i].T;
    //    phi_l = calibration_data[i].left_speed;
    //    phi_r = calibration_data[i].right_speed;
    //    sm[0] = calibration_data[i].sm.GetX();
    //    sm[1] = calibration_data[i].sm.GetY();
    //    sm[2] = calibration_data[i].sm.GetHeading();

    //    vs(L_i, 0,  T*phi_l);
    //    vs(L_i, 1,  T*phi_r);
    //    gsl_blas_dger(1, L_i, L_i, A); // A = A + L_i'*L_i;  A symmetric
    //    gsl_blas_daxpy(sm[2], L_i, g);  // g = g + L_i'*theta_i;  sm :{x , y, theta}
    //}
    //gsl_vector_free(L_i);

    //// Verify that A isn't singular
    //double cond = cond_number(A);
    //if(cond > max_cond_number_)
    //{
    //    std::cout << "ERROR ----- Matrix A is almost singular. Not very exciting data!" << std::endl;
    //    std::cout << "max_cond_number: " << cond << ", max allowed: " << max_cond_number_ << std::endl;
    //    return false;
    //}

    //// Ay = g --> y = inv(A)g; A square matrix;
    //gsl_vector * y = gsl_vector_calloc(2);
    //if(!solve_square_linear(A, g, y))
    //{
    //    std::cout << "ERROR ----- Cannot solve Ay=g. Invalid arguments" << std::endl;
    //    return false;
    //}

    ////得到 J21, J22
    //J21 = vg(y, 0); J22 = vg(y, 1);

    //if(gsl_isnan(J21) || gsl_isnan(J22))
    //{
    //    sm_error("Could not find first two parameters J21, J22");
    //    return false;
    //}
    //gsl_vector_free(y);

    //for(int k = 0; k < n; k++)
    //{
    //    double o_theta, w0;

    //    T = calibration_data[k].T;
    //    phi_l = calibration_data[k].left_speed;
    //    phi_r = calibration_data[k].right_speed;
    //    sm[0] = calibration_data[k].sm.GetX();
    //    sm[1] = calibration_data[k].sm.GetY();
    //    sm[2] = calibration_data[k].sm.GetHeading();

    //    o_theta = T*(J21*phi_l + J22*phi_r);  //r_theta
    //    w0 = o_theta / T;

    //    if(fabs(o_theta) > 1e-12)
    //    {
    //        t1 = sin(o_theta) / o_theta;
    //        t2 = (1-cos(o_theta)) / o_theta;
    //    }
    //    else
    //    {
    //        t1 = 1.0;
    //        t2 = 0.0;
    //    }

    //    cx1 = 0.5 * T * (-J21 * phi_l) * t1;
    //    cx2 = 0.5 * T * (+J22 * phi_r) * t1;
    //    cy1 = 0.5 * T * (-J21 * phi_l) * t2;
    //    cy2 = 0.5 * T * (+J22 * phi_r) * t2;

    //    if ( (calibrat_mode_ == MODE_0)||(calibrat_mode_ == MODE_1) )
    //    {
    //        cx = cx1 + cx2;
    //        cy = cy1 + cy2;

    //        double array[] = {
    //            -cx, 1-cos(o_theta),   sin(o_theta), sm[0], -sm[1],
    //            -cy,  -sin(o_theta), 1-cos(o_theta), sm[1],  sm[0]
    //        };

    //        gsl_matrix_view tmp = gsl_matrix_view_array(array, 2, 5);
    //        L_k = &tmp.matrix;
    //        // M = M + L_k' * L_k; M is symmetric
    //        gsl_blas_dgemm (CblasTrans,CblasNoTrans, 1, L_k, L_k, 1, M);
    //    }
    //    else
    //    {
    //        double array2[] = {
    //            -cx1, -cx2, 1-cos(o_theta),   sin(o_theta), sm[0], -sm[1],
    //            -cy1, -cy2,  -sin(o_theta), 1-cos(o_theta), sm[1],  sm[0]
    //        };

    //        gsl_matrix_view tmp = gsl_matrix_view_array(array2, 2, 6);
    //        L_2k = &tmp.matrix;
    //        // M2 = M2 + L_2k' * L_2k; M2 is symmetric
    //        gsl_blas_dgemm (CblasTrans,CblasNoTrans, 1, L_2k, L_2k, 1, M2);
    //    }
    //}

    //double est_b, est_d_l, est_d_r, laser_x, laser_y, laser_th;
    //gsl_vector * result_x;
    //switch (calibrat_mode_)
    //{
    //case MODE_0:	/*!<!--######### mode 0: minimize in closed form, using M ########-->*/
    //{
    //    result_x = full_calibration_min(M);

    //    est_b = vg(result_x, 0);
    //    est_d_l = 2*(-est_b * J21);
    //    est_d_r  = 2*(est_b * J22);
    //    laser_x = vg(result_x, 1);
    //    laser_y = vg(result_x, 2);
    //    laser_th = atan2(vg(result_x, 4), vg(result_x, 3));
    //    break;
    //}
    //case MODE_1:	/*!<!--####  mode 1: start at the minimum eigenvalue, find numerical solution using M  ####-->*/
    //{
    //    result_x = numeric_calibration(M);

    //    est_b = vg(result_x, 0);
    //    est_d_l = 2*(-est_b * J21);
    //    est_d_r  = 2*(est_b * J22);
    //    laser_x = vg(result_x, 1);
    //    laser_y = vg(result_x, 2);
    //    laser_th = atan2(vg(result_x, 4), vg(result_x, 3));
    //    break;
    //}
    //case MODE_2:	/*!<!--####  mode 2: start at the minimum eigenvalue, find numerical solution using M2 ####-->*/
    //{
    //    result_x = numeric_calibration(M2);

    //    est_b = (vg(result_x, 0) + vg(result_x, 1))/2;
    //    est_d_l = 2*(-est_b * J21);
    //    est_d_r  = 2*(est_b * J22);
    //    laser_x = vg(result_x, 2);
    //    laser_y = vg(result_x, 3);
    //    laser_th = atan2(vg(result_x, 5), vg(result_x, 4));
    //    break;
    //}
    //default:
    //    break;
    //}

    //std::cout << "axle: " << est_b << std::endl;
    //std::cout << "l_diam: " << est_d_l << std::endl;
    //std::cout << "r_diam: " << est_d_r << std::endl;
    //std::cout << "l_x: " << laser_x << std::endl;
    //std::cout << "l_y: " << laser_y << std::endl;
    //std::cout << "l_theta: " << laser_th << std::endl;

    return true;
}

//用算法2来进行标定．－－目前我们使用的就是这个方法．
bool RobotCalibrater::computeCalibraParam_02(vectorCalibration calibration_data)
{
    //构造标定的tuple．tuple包含的数据为时间dt,左右两轮的速度left_speed,right_speed
    //激光雷达计算的相对位姿(dx,dy,dtheta)
    using namespace Eigen;
    vector<calib_tuple> tuples;
    for(int i = 0; i < calibration_data.size(); i++)
    {
        calib_tuple tuple;

        tuple.T = calibration_data[i].T;
        tuple.phi_l = calibration_data[i].left_speed;
        tuple.phi_r = calibration_data[i].right_speed;
        tuple.sm[0] = calibration_data[i].sm.GetX();
        tuple.sm[1] = calibration_data[i].sm.GetY();
        tuple.sm[2] = calibration_data[i].sm.GetHeading();

        tuples.push_back(tuple);
    }

    //标定的结果．
    calib_result res;

    vector<calib_tuple> original_tuples = tuples;
    vector<calib_tuple> tuple_history[outliers_iterations_+1];

    //进行迭代，默认迭代次数为5次．进行迭代的主要原因是去除outlier数据．
    for(int iteration = 0; iteration <= outliers_iterations_; iteration++)
    {
        tuple_history[iteration] = tuples;

        // Calibration--进行标定，这里是主要的标定计算的函数．
        if(!solve(tuples, calibrat_mode_, max_cond_number_, res))
        {
            std::cout << "ERROR ----- Failed calibration." << std::endl;
            return false;
        }

        // Compute residuals--计算残差
        for(int i = 0; i < tuples.size(); i++)
        {
            tuples[i].compute_disagreement(res);
        }

        // Sort residuals and compute thresholds
        // 对残差进行排序

        //角度残差
        vector<double> err_theta;
        for(int i = 0; i < tuples.size(); i++)
        {
            err_theta.push_back( fabs(tuples[i].err[2]) );
        }

        //x,y残差
        vector<double> err_xy;
        for(int i = 0; i < tuples.size(); i++)
        {
            double x = tuples[i].err[0];
            double y = tuples[i].err[1];
            err_xy.push_back( sqrt(x*x + y*y) );
        }

        vector<double> err_theta_sorted(err_theta);
        sort(err_theta_sorted.begin(), err_theta_sorted.end());

        vector<double> err_xy_sorted(err_xy);
        sort(err_xy_sorted.begin(), err_xy_sorted.end());

        //计算需要求出的数量，从小到大排序，也可以等价与下标．
        int threshold_index = (int) round((1-outliers_percentage_)*tuples.size());

        //进行限幅
        if(threshold_index<0)
            threshold_index = 0;

        if(threshold_index>=tuples.size())
            threshold_index = tuples.size()-1;

        //得到对应的残差阈值
        double threshold_theta = err_theta_sorted[threshold_index];
        double threshold_xy = err_xy_sorted[threshold_index];

        int noutliers = 0;
        int noutliers_theta = 0;
        int noutliers_xy = 0;
        int noutliers_both = 0;

        //对于残差超过阈值的，直接设置为outlier;
        for(int i = 0; i < tuples.size(); i++)
        {
            int xy = err_xy[i] > threshold_xy;
            int theta = err_theta[i] > threshold_theta;

            tuples[i].mark_as_outlier = xy | theta;

            if(xy)
                noutliers_xy++;
            if(theta)
                noutliers_theta++;
            if(xy&&theta)
                noutliers_both++;
            if(xy||theta)
                noutliers++;
        }

        // Output statistics
        // 输出统计信息．
        sm_info("Outlier detection: #tuples = %d, removing percentage = %.0f%% (index=%d)\n", tuples.size(),
                100*outliers_percentage_, threshold_index);
        sm_info("    err_theta  min = %f  max = %f   threshold = %f \n",
                err_theta[0], err_theta[tuples.size()-1], threshold_theta);
        sm_info("    err_xy  min = %f  max = %f   threshold = %f \n",
                err_xy[0], err_xy[tuples.size()-1], threshold_xy);

        sm_info("    num_outliers = %d; for theta = %d; for xy = %d; for both = %d\n", noutliers, noutliers_theta,
                noutliers_xy, noutliers_both);

        // Write new tuples
        // 更新数据．
        vector<calib_tuple> n;
        for(int i = 0; i < tuples.size(); i++)
        {
            if(!tuples[i].mark_as_outlier)
                n.push_back(tuples[i]);
        }

        tuples = n;
    }

    /* Let's estimate the scan matching noise */
    //估计激光扫描匹配的协方差．
    double laser_std_x, laser_std_y, laser_std_th;
    int estimate_with = 1;
    estimate_noise(tuple_history[estimate_with], res, laser_std_x, laser_std_y, laser_std_th);
    std::cout << "Using sm errors (std dev): " << laser_std_x * 1000 << " mm, " <<
                 laser_std_y * 1000 << " mm, " << rad2deg(laser_std_th) << " deg" << std::endl;


    //计算费雪信息矩阵．
    //** gsl_matrix *laser_fim = gsl_matrix_alloc(3,3);
    //** gsl_matrix_set_zero(laser_fim);
    //** gsl_matrix_set(laser_fim,0,0, 1 / (laser_std_x*laser_std_x));
    //** gsl_matrix_set(laser_fim,1,1, 1 / (laser_std_y*laser_std_y));
    //** gsl_matrix_set(laser_fim,2,2, 1 / (laser_std_th*laser_std_th));

    Matrix3d eigen_laser_fim = Matrix3d::Zero();
    eigen_laser_fim(0,0) =  1 / ( pow(laser_std_x, 2) );
    eigen_laser_fim(1,1) =  1 / ( pow(laser_std_y, 2) );
    eigen_laser_fim(2,2) =  1 / ( pow(laser_std_th, 2) );

    Matrix<double, 6, 6> fim = Matrix<double, 6, 6>::Zero();
    for(int i = 0; i < tuples.size(); i++)
    {
        MatrixXd fimi = tuples[i].compute_fim(res, eigen_laser_fim);
        fim += fimi;
    }
    /* Inverting the FIM to get covariance */
    Matrix<double, 6, 6> state_cov = fim.inverse();
 
    std::cout << "Uncertainty r_L  = " << 1000 * sqrt( state_cov(0, 0) ) << " mm" << std::endl;
    std::cout << "Uncertainty r_R  = " << 1000 * sqrt( state_cov(1, 1) ) << " mm" << std::endl;
    std::cout << "Uncertainty axle = " << 1000 * sqrt( state_cov(2, 2) ) << " mm" << std::endl;
    std::cout << "Uncertainty l_x  = " << 1000 * sqrt( state_cov(3, 3) ) << " mm" << std::endl;
    std::cout << "Uncertainty l_y  = " << 1000 * sqrt( state_cov(4, 4) ) << " mm" << std::endl;
    std::cout << "Uncertainty l_th = " << rad2deg( sqrt( state_cov(5, 5) )) << " deg" << std::endl << std::endl;
 
    std::cout << "axle: " << res.axle << std::endl;
    std::cout << "l_diam: " << res.radius_l << std::endl;
    std::cout << "r_diam: " << res.radius_r << std::endl;
    std::cout << "l_x: " << res.l[0] << std::endl;
    std::cout << "l_y: " << res.l[1] << std::endl;
    std::cout << "l_theta: " << res.l[2]<<"|"<<res.l[2]*57.295 << std::endl;

    return true;
}

//计算轮子的增加值
Point2 RobotCalibrater::computeRadInc(double laser_ref_ts, double laser_sens_ts,
                                      int under_bound, int up_bound)
{
    Point2 radInc;
    double times[4];
    double tr1, tr2, ts1, ts2;
    double Lr1, Lr2, Ls1, Ls2;
    double Rr1, Rr2, Rs1, Rs2;
    double ref_alpha, sens_alpha, avg_left_start, avg_right_start,
            avg_left_end, avg_right_end, left_inc, right_inc;

    tr1 = odom_data_[under_bound].time_stamp;
    tr2 = odom_data_[under_bound+1].time_stamp;

    ts1 = odom_data_[up_bound-1].time_stamp;
    ts2 = odom_data_[up_bound].time_stamp;

    times[0] = fabs(tr1 - laser_ref_ts);
    times[1] = fabs(tr2 - laser_ref_ts);

    times[2] = fabs(ts1 - laser_sens_ts);
    times[3] = fabs(ts2 - laser_sens_ts);

    ref_alpha  = (laser_ref_ts - tr1) / (tr2 - tr1);
    sens_alpha = (laser_sens_ts - ts1) /(ts2 - ts1);

    //左轮里程计位置
    Lr1 = odom_data_[under_bound].link_rad_position;
    Lr2 = odom_data_[under_bound+1].link_rad_position;
    Ls1 = odom_data_[up_bound-1].link_rad_position;
    Ls2 = odom_data_[up_bound].link_rad_position;

    //右轮里程计位置
    Rr1 = odom_data_[under_bound].right_rad_position;
    Rr2 = odom_data_[under_bound+1].right_rad_position;
    Rs1 = odom_data_[up_bound-1].right_rad_position;
    Rs2 = odom_data_[up_bound].right_rad_position;

    //插值计算编码器位置
    avg_left_start  = (1-ref_alpha) * Lr1 + ref_alpha * Lr2;
    avg_right_start = (1-ref_alpha) * Rr1 + ref_alpha * Rr2;
    avg_left_end 	= (1-sens_alpha)* Ls1 + sens_alpha* Ls2;
    avg_right_end 	= (1-sens_alpha)* Rs1 + sens_alpha* Rs2;

    //编码器的增量
    left_inc  = (avg_left_end - avg_left_start);
    right_inc = (avg_right_end - avg_right_start);

    radInc.x = left_inc;
    radInc.y = right_inc;

    return radInc;
}

//查找里程计的索引，使时间 ts 界于第 i 和 i+1 个里程计数据之间，返回 i, 下界
// 找到离时间ts最近的里程计数据i,i<ts
int RobotCalibrater::findClosestRef(double ts, vectorOdom odo)
{
    double ts_i, ts_ip;

    for (int i=0; i<(int)odo.size()-1; i++)
    {
        ts_i = odo[i].time_stamp;
        ts_ip = odo[i+1].time_stamp;
        if ((ts_i <= ts) && (ts_ip >= ts))
            return i;
    }

    return -1;
}

//查找里程计的索引，使时间 ts 介于第 i-1 和 i 个里程计数据之间，返回 i, 上界
int RobotCalibrater::findClosestSens(double ts, vectorOdom odo)
{
    double ts_i, ts_ip;

    for (int i=1; i<(int)odo.size(); i++)
    {
        ts_i = odo[i].time_stamp;
        ts_ip = odo[i-1].time_stamp;
        if ((ts_i >= ts) && (ts_ip <= ts))
            return i;
    }

    return -1;
}

//把激光雷达数据 转换为 CSM 所需要的数据
void RobotCalibrater::LocalizedRangeScanToLDP(sensor_msgs::LaserScan scan,
                                              LDP& ldp)
{
    int nPts = scan.ranges.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        if(scan.ranges[i] > scan.range_min && scan.ranges[i] < scan.range_max)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = scan.ranges[i];
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }
        double scan_angle = scan.angle_min + i*scan.angle_increment;

        if(scan_angle > scan.angle_max)
            scan_angle = scan.angle_max;

        ldp->theta[i] = scan_angle;
    }
//    ldp->min_theta = scan.angle_min;
//    ldp->max_theta = scan.angle_max;
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];


    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}


//把通用激光雷达数据　转换为　CSM所需要的数据．
void RobotCalibrater::GeneralScanToLDP(GeneralLaserScan &scan,
                                              LDP& ldp)
{
    int nPts = scan.m_rangeReadings.size();
    ldp = ld_alloc_new(nPts);

    for(int i = 0;i < nPts;i++)
    {
        if(scan.m_rangeReadings[i] > scan.m_rangeMin && scan.m_rangeReadings[i] < scan.m_rangeMax)
        {
            ldp->valid[i] = 1;
            ldp->readings[i] = scan.m_rangeReadings[i];
        }
        else
        {
            ldp->valid[i] = 0;
            ldp->readings[i] = -1;
        }

        double scan_angle = scan.m_angleReadings[i];

        ldp->theta[i] = scan_angle;
    }
//    ldp->min_theta = scan.angle_min;
//    ldp->max_theta = scan.angle_max;
    ldp->min_theta = ldp->theta[0];
    ldp->max_theta = ldp->theta[nPts-1];


    ldp->odometry[0] = 0.0;
    ldp->odometry[1] = 0.0;
    ldp->odometry[2] = 0.0;

    ldp->true_pose[0] = 0.0;
    ldp->true_pose[1] = 0.0;
    ldp->true_pose[2] = 0.0;
}



//设置PI-ICP的参数
void RobotCalibrater::SetPIICPParams(GeneralLaserScan& scan)
{
    //设置激光的范围
    m_PIICPParams.min_reading = scan.m_rangeMin;
    m_PIICPParams.max_reading = scan.m_rangeMax;

    //设置位姿最大的变化范围
    //m_PIICPParams.max_angular_correction_deg = 5.0;
    m_PIICPParams.max_angular_correction_deg = 5.0;
    //m_PIICPParams.max_linear_correction = 0.1;
    m_PIICPParams.max_linear_correction = 0.05;

    //设置迭代停止的条件
    //m_PIICPParams.max_iterations = 10;
    m_PIICPParams.max_iterations = 500;
    //m_PIICPParams.epsilon_xy = 0.00001;
    //m_PIICPParams.epsilon_theta = 0.00001;
    m_PIICPParams.epsilon_xy = 0.000001;
    m_PIICPParams.epsilon_theta = 0.000001;

    //设置correspondence相关参数
    //m_PIICPParams.max_correspondence_dist = 0.05;
    m_PIICPParams.max_correspondence_dist = 0.2;
    m_PIICPParams.sigma = 0.01;
    m_PIICPParams.use_corr_tricks = 1;

    //设置restart过程，因为不需要restart所以可以不管
    m_PIICPParams.restart = 0;
    m_PIICPParams.restart_threshold_mean_error = 0.01;
    m_PIICPParams.restart_dt = 1.0;
    m_PIICPParams.restart_dtheta = 0.1;

    //设置聚类参数
    m_PIICPParams.clustering_threshold = 0.2;

    //用最近的10个点来估计方向
    m_PIICPParams.orientation_neighbourhood = 10;

    //设置使用PI-ICP
    m_PIICPParams.use_point_to_line_distance = 1;

    //不进行alpha_test
    m_PIICPParams.do_alpha_test = 0;
    m_PIICPParams.do_alpha_test_thresholdDeg = 5;

    //设置trimmed参数 用来进行outlier remove
    m_PIICPParams.outliers_maxPerc = 0.9;
    m_PIICPParams.outliers_adaptive_order = 0.7;
    m_PIICPParams.outliers_adaptive_mult = 2.0;

    //进行visibility_test 和 remove double
    m_PIICPParams.do_visibility_test = 1;
    m_PIICPParams.outliers_remove_doubles = 1;
    m_PIICPParams.do_compute_covariance = 0;
    m_PIICPParams.debug_verify_tricks = 0;
    m_PIICPParams.use_ml_weights = 0;
    m_PIICPParams.use_sigma_weights = 0;
}

//求两帧之间的icp位姿匹配
bool RobotCalibrater::PIICPBetweenTwoFrames(LDP& currentLDPScan, LDP& prevLDP,
                                            GeneralLaserScan& scan, Pose2 initPose,
                                            Pose2& matchPose)
{
    prevLDP->estimate[0] = 0.0;
    prevLDP->estimate[1] = 0.0;
    prevLDP->estimate[2] = 0.0;

    SetPIICPParams(scan);

    //设置匹配的参数值
    m_PIICPParams.laser_ref = prevLDP;
    m_PIICPParams.laser_sens = currentLDPScan;

    m_PIICPParams.first_guess[0] = initPose.GetX();
    m_PIICPParams.first_guess[1] = initPose.GetY();
    m_PIICPParams.first_guess[2] = initPose.GetHeading();

    m_OutputResult.cov_x_m = 0;
    m_OutputResult.dx_dy1_m = 0;
    m_OutputResult.dx_dy2_m = 0;

    sm_icp(&m_PIICPParams,&m_OutputResult);

    //nowPose在lastPose中的坐标
    if(m_OutputResult.valid)
    {
        //得到两帧激光之间的相对位姿
        matchPose.SetX(m_OutputResult.x[0]);
        matchPose.SetY(m_OutputResult.x[1]);
        matchPose.SetHeading(m_OutputResult.x[2]);

        return true;
    }
    else
    {
        std::cout <<"ERROR ----- PI ICP Failed"<<std::endl;

        return false;
    }
}

}
