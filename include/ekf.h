#include <ros/ros.h>
#include <Eigen/Dense>
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float64.h"

void velCallback(const nav_msgs::OdometryConstPtr& msg);
void laserCallback(const std_msgs::Float64ConstPtr& msg);
void publishEstimate();

void propagate();
void update();
void setH();
void seth();
Eigen::VectorXd x(2); // state vector {x x_dot}
Eigen::VectorXd z(2); // measurements vector {I vel}
Eigen::VectorXd y(2); // measurement pre-fit resitual {I vel}
Eigen::MatrixXd F(2,2); // state transition model matrix
Eigen::MatrixXd P(2,2); // covariance matrix
Eigen::MatrixXd Q(2,2); // process noise covariance matrix
Eigen::MatrixXd K(2,2); // kalman gain matrix
Eigen::MatrixXd H(2,2); // observation model matrix
Eigen::VectorXd h(2); // Map state to measurements
Eigen::MatrixXd R(2,2); // observation noise covariance matrix
Eigen::MatrixXd S(2,2); // pre-fit residual covariance matrix
Eigen::MatrixXd I = Eigen::MatrixXd::Identity(2,2); // 6x6 Identity Matrix

ros::Subscriber vel_sub_;
ros::Subscriber laser_sub_;
ros::Publisher estimate_pub_;

ros::Time time_prev_;

nav_msgs::Odometry estimate_msg_;