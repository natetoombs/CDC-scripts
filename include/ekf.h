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
Eigen::VectorXd x(4); // state vector {r theta r_dot theta_dot}
Eigen::VectorXd z(3); // measurements vector {I velx vely}
Eigen::VectorXd y(3); // measurement pre-fit resitual {I velx vely}
Eigen::MatrixXd F(4,4); // state dynamic model matrix
Eigen::MatrixXd P(4,4); // covariance matrix
Eigen::MatrixXd Q(4,4); // process noise covariance matrix
Eigen::MatrixXd K(4,3); // kalman gain matrix
Eigen::MatrixXd H(3,4); // observation model matrix
Eigen::VectorXd h(3); // Map state to measurements
Eigen::MatrixXd R(3,3); // observation noise covariance matrix
Eigen::MatrixXd S(3,3); // pre-fit residual covariance matrix
Eigen::MatrixXd I = Eigen::MatrixXd::Identity(4,4); // 6x6 Identity Matrix

ros::Subscriber vel_sub_;
ros::Subscriber laser_sub_;
ros::Publisher estimate_pub_;

ros::Time time_prev_;

nav_msgs::Odometry estimate_msg_;

double altitude;
double divergence;

double x_uav;
double y_uav;