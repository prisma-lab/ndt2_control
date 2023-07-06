#include "ros/ros.h"
#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"
#include "utils.h"
#include "math.h"
#include "planner_spline.h"

using namespace std;

class VOLIRO_CTRL{
    public:
        VOLIRO_CTRL();
        void run();
        void pose_cb(geometry_msgs::PoseStamped);
        void vel_cb(geometry_msgs::TwistStamped);
        void quaternion_error();
        void position_control();
        void attitude_control();
        void allocation();
        void load_parameters();
        void desired_trajectory();
    
    private:
        ros::NodeHandle _nh;
        ros::Subscriber _pose_sub;
        ros::Subscriber _vel_sub;
        ros::Publisher  _motor_vel_pub;
        ros::Publisher  _motor1_tilt_pub;
        ros::Publisher  _motor2_tilt_pub;
        ros::Publisher  _motor3_tilt_pub;
        ros::Publisher  _motor4_tilt_pub;
        ros::Publisher  _motor5_tilt_pub;
        ros::Publisher  _motor6_tilt_pub;

        Eigen::Vector3d _local_pos;
        Eigen::Vector3d _local_vel;
        Eigen::Vector3d _local_ang_vel;
        Eigen::Vector4d _local_quat;
        Eigen::Vector3d _F_des;
        Eigen::Vector3d _M_des;
        Eigen::Vector3d _omega_des;
        CARTESIAN_PLANNER *_trajectory;

        Eigen::Vector3d _des_pos;
        Eigen::Vector3d _des_vel;
        Eigen::Vector3d _des_acc;
        Eigen::Vector4d _des_att;
        Eigen::Vector3d _des_ang_vel;

        std_msgs::Float32MultiArray _rotors_vel;
        std_msgs::Float64 _tilt_m1;
        std_msgs::Float64 _tilt_m2;
        std_msgs::Float64 _tilt_m3;
        std_msgs::Float64 _tilt_m4;
        std_msgs::Float64 _tilt_m5;
        std_msgs::Float64 _tilt_m6;

        //Control weights
        std::vector<double> _Kp_pos_yaml;
        std::vector<double> _Ki_pos_yaml;
        std::vector<double> _Kd_pos_yaml;
        std::vector<double> _Kp_att_yaml;
        std::vector<double> _Kd_att_yaml;
        Eigen::DiagonalMatrix<double, 3> _Kp_pos;
        Eigen::DiagonalMatrix<double, 3> _Ki_pos;
        Eigen::DiagonalMatrix<double, 3> _Kd_pos;
        Eigen::DiagonalMatrix<double, 3> _Kp_att;
        Eigen::DiagonalMatrix<double, 3> _Kd_att;

        //Drone parameters
        double _mass;
        double _gravity;
        double _r_com_off_x;
        double _r_com_off_y;
        double _r_com_off_z;
        double _Ixx;
        double _Iyy;
        double _Izz;
        double _Kf;
        double _Kq;
        double _arm;
        Eigen::VectorXd _arm_length;
        std::vector<double> _arm_h_yaml;
        std::vector<double> _c_yaml;
        std::vector<double> _ang_mot_yaml;
        Eigen::Matrix<double, 8, 1> _c;
        Eigen::Matrix<double, 8, 1> _ang_mot;
        Eigen::Matrix<double, 8, 1> _arm_h;
        int _rate;

        bool _first_measure;
};