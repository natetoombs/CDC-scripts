#include <ros/ros.h>
#include "../include/ekf.h"
#include <ros/console.h>
#include "tf/transform_datatypes.h"
#include <thread>
#include <math.h>

using namespace Eigen;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "extended_kalman_filter");
    ros::NodeHandle nh_;

    nh_.param("divergence_deg",divergence,12.0);

    vel_sub_ = nh_.subscribe("camera/odom/sample", 100, velCallback);
    laser_sub_ = nh_.subscribe("laser_strength", 100, laserCallback);
    estimate_pub_ = nh_.advertise<nav_msgs::Odometry>("ekf_estimate", 1);

    // std::thread propagation(propagate);

    // Initialize x
    x << 0.2, 0;

     // Initialize Q
    double pos_process_noise = 0.005;
    double vel_process_noise = 0.02;
    Q << pos_process_noise, 0, 0, vel_process_noise;
    
    // Initialize P
    P << 0.5, 0, 0, 0.2;
    
    // Initialize R
    double laser_cov = 0.025;
    double vel_cov = 0.005;
    VectorXd meas_cov(2);
    meas_cov << laser_cov, vel_cov;
    R = meas_cov.asDiagonal();
    
    time_prev_ = ros::Time::now();
    ros::spin();

    // propagation.join();

    return 0;
}


void velCallback(const nav_msgs::OdometryConstPtr& msg)
// Receive the IMU data, sort, send to calculateEstimate()
{
    z(1) = msg->twist.twist.linear.y;
    altitude = -msg->pose.pose.position.z;

    // propagate();
    // update();
    // publishEstimate();
}

void laserCallback(const std_msgs::Float64ConstPtr& msg)
// Receive the IMU data, sort, send to calculateEstimate()
{
    z(0) = msg->data;

    propagate();
    update();
    publishEstimate();
}

void propagate()
{
    ros::Time now = ros::Time::now();
    double dt = now.toSec() - time_prev_.toSec();
    time_prev_ = now;
    F = I;
    F(0,1) = dt;
    x = F*x;
    P = F*P*F.transpose() + Q;
}

// void propagate()
// {   
//     ros::Rate rate(100);
//     while (ros::ok())
//     {
//         ros::Time now = ros::Time::now();
//         double dt = now.toSec() - time_prev_.toSec();
//         time_prev_ = now;
//         Vector3d dt_vec(dt, dt, dt);
//         // Build F
//         F = I;
//         F.topRightCorner(3, 3) = dt_vec.asDiagonal();
//         x = F*x;
//         P = F*P*F.transpose() + Q;
//         rate.sleep();
//     }
// }

void update()
{
    seth();
    setH();
    y = z - h;
    S = H*P*H.transpose() + R;
    K = P*H.transpose()*S.inverse();
    x = x + K*y;
    P = (I - K*H)*P;
}

void publishEstimate()
{
    estimate_msg_.header.stamp = ros::Time::now();
    estimate_msg_.pose.pose.position.x = x(0);
    estimate_msg_.twist.twist.linear.x = x(1);
    estimate_msg_.pose.covariance[0] = P(0,0);

    estimate_pub_.publish(estimate_msg_);
}

void setH()
{
    double z = altitude;
    double theta = divergence*3.1415926535/180;
    double n = 8;
    double dIdx = 18.17/pow(z+8.13,2)*exp(-2*pow((x(0)/(z*sin(theta/2))),n))*-2*n/pow((z*sin(theta/2)),n)*pow(x(0),(n-1));
    double dvdv = 1;
    H << dIdx, 0,
         0, dvdv;
}

void seth()
{
    double z = altitude;
    double theta = divergence*3.1415926535/180;
    double n = 8;
    double I = 18.17/pow(z+8.13,2)*exp(-2*pow((x(0)/(z*sin(theta/2))),n));
    double vel = x(1);
    
    VectorXd h_temp(2);
    h_temp << I, vel;
    h = h_temp;
}