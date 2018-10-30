#ifndef ROBOT_CALIBRATION_H
#define ROBOT_CALIBRATION_H

#include <ros/ros.h>

#include "std_msgs/Empty.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"

#include "champion_nav_msgs/OdomRadData.h"
#include "champion_nav_msgs/ChampionNavLaserScan.h"

//std
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include <vector>
#include <algorithm>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//gsl
#include <gsl/gsl_blas.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_cblas.h>
#include <gsl/gsl_statistics_double.h>

#include <options/options.h>

//boost
#include "boost/foreach.hpp"

#include "./geometry_datatype.h"
#include "./solver_utils.h"
#include "./solver2_meat.h"

#include "eigen3/Eigen/Core"

#define TK_THRESHOLD 2.5
#define DEFAULT_AXIS_DIST 0.62
#define DEFAULT_RAD_RADIUS 0.1
#define DEFAULT_LASER_MITTEL_DIST 0.303

namespace robot_calibration
{
struct odomData
{
    bool first_data;
    double time_stamp;
    double link_rad_position;
    double right_rad_position;
};

struct calibrationData{
    double T;
    double left_speed;
    double right_speed;
    Pose2 sm;

    calibrationData()
    {}

    calibrationData(double input_T, double input_left_speed,
                    double input_right_speed, Pose2 input_sm)
    {
        T = input_T;
        left_speed = input_left_speed;
        right_speed = input_right_speed;
        sm = input_sm;
    }
};

struct scanPoseObser
{
    //上一帧激光的时间戳
    double laser_ref_ts;
    //当前激光的时间戳
    double laser_sens_ts;
    //里程计估计移动距离
    double d_t;
    //里程计估计旋转角度
    double alpha_t;
    //匹配前两帧激光的相对位姿
    Pose2 initPose;
    //匹配后两帧激光的相对位姿
    Pose2 relativPose;
};

typedef struct general_laser_scan
{
    std::string m_framdID;
    ros::Time m_stamp;
    std::vector<double> m_rangeReadings;
    std::vector<double> m_angleReadings;
    std::vector<bool> m_isDynamics;       //指示对应的激光束是否是动态物体
    double m_rangeMin,m_rangeMax;
    double m_angleMin,m_angleMax;
    double m_angleIncrement;

    int m_validMeasurementCount;        //距离有效激光数据
    int m_dynamicCount;                 //被认为是动态的激光数据

}GeneralLaserScan;

typedef std::vector<odomData> vectorOdom;
typedef std::vector<GeneralLaserScan> vectorScan;
typedef std::vector<calibrationData> vectorCalibration;
typedef std::vector<scanPoseObser> vectorScanPoseObser;

class RobotCalibrater
{
public:
    RobotCalibrater();
    ~RobotCalibrater();

private:
    void getOdomDataCallback(const champion_nav_msgs::OdomRadData::ConstPtr& msg);
    void getLaserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void getChampionLaserCallback(const champion_nav_msgs::ChampionNavLaserScanConstPtr& msg);
    void startCollectionCallback(const std_msgs::Empty::ConstPtr& msg);
    void endCollectionCallback(const std_msgs::Empty::ConstPtr& msg);
    void requestCalibraCallback(const std_msgs::Empty::ConstPtr& msg);

    void GeneralScanToLDP(sensor_msgs::LaserScan scan,
                                           LDP& ldp);

    //得到里程计位姿．
    bool getOdomPose(Eigen::Vector3d& odomPose,const ros::Time& t);

    //通用函数．
    void rosLaserScanToGeneralLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_scan,GeneralLaserScan & general_scan);
    void championLaserScanToGeneralLaserScan(const champion_nav_msgs::ChampionNavLaserScan&champion_scan,GeneralLaserScan & general_scan);

    bool writeToSychronDataFile(vectorCalibration calibration_data);
    bool writeToScanObserDataFile(vectorScanPoseObser vectorObser);
    bool computeCalibraParam_01(vectorCalibration calibration_data);
    bool computeCalibraParam_02(vectorCalibration calibration_data);
    Point2 computeRadInc(double laser_ref_ts, double laser_sens_ts, int under_bound, int up_bound);
    int findClosestRef(double ts, vectorOdom odo);
    int findClosestSens(double ts, vectorOdom odo);

    void LocalizedRangeScanToLDP(sensor_msgs::LaserScan scan, LDP& ldp);

    void GeneralScanToLDP(GeneralLaserScan &scan,
                          LDP& ldp);

    void SetPIICPParams(GeneralLaserScan& scan);
    bool PIICPBetweenTwoFrames(LDP& currentLDPScan, LDP& prevLDP, GeneralLaserScan& scan, Pose2 initPose,
                               Pose2& matchPose);

protected:
    ros::Subscriber start_data_collection_sub_, end_data_collection_sub_, calibrat_request_sub_;
    ros::Subscriber odom_data_sub_, laser_data_sub_,champion_laser_data_sub_;

    //进行数据求解．
    tf::TransformListener m_transformListener;

    bool data_collection_, last_scan_init_;
    bool is_use_champion_scan_;

    int outliers_iterations_;
    double outliers_percentage_;
    double max_cond_number_;
    double laser_stamp_offset_;

    //进行数据选择的距离和角度阈值
    double m_DistThreshold;
    double m_AngleThreshold;


    //进行PI-ICP需要的变量
    sm_params m_PIICPParams;
    sm_result m_OutputResult;

    CALIBR_MODE calibrat_mode_;
    CALIBR_SOLVER calibrat_solver_;

    std::vector<Eigen::Vector3d> m_IntegOdomData;
    vectorOdom odom_data_;
    vectorScan laser_data_;
    vectorCalibration calibration_data_;
};
}

#endif // ROBOT_CALIBRATION_H
