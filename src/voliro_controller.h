#include "ros/ros.h"
#include "Eigen/Dense"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"
#include "boost/thread.hpp"
#include "utils.h"
#include "math.h"
#include "planner_spline.h"
#include <fstream>

using namespace std;

// Storing data classes definition
class saved_data {
public:
    vector<double> x;
    vector<double> y;
    vector<double> z;

    vector<double> vx;
    vector<double> vy;
    vector<double> vz;

	vector<double> r;
    vector<double> p;
    vector<double> yaw;

    vector<double> wx;
    vector<double> wy;
    vector<double> wz;

    vector<double> fx;
    vector<double> fy;
    vector<double> fz;

	vector<double> f_des;

    vector<double> x1;
    vector<double> x2;
    vector<double> x3;
    vector<double> x4;
    vector<double> y1;
    vector<double> y2;
    vector<double> y3;
    vector<double> y4;

    vector<double> vx1;
    vector<double> vx2;
    vector<double> vx3;
    vector<double> vx4;
    vector<double> vy1;
    vector<double> vy2;
    vector<double> vy3;
    vector<double> vy4;

	vector<double> x1_des;
    vector<double> x2_des;
    vector<double> x3_des;
    vector<double> x4_des;
    vector<double> y1_des;
    vector<double> y2_des;
    vector<double> y3_des;
    vector<double> y4_des;

    vector<double> es1;
    vector<double> es2;
    vector<double> es3;
    vector<double> es4;
    vector<double> es5;
    vector<double> es6;
    vector<double> es7;
    vector<double> es8;

    vector<double> eds1;
    vector<double> eds2;
    vector<double> eds3;
    vector<double> eds4;
    vector<double> eds5;
    vector<double> eds6;
    vector<double> eds7;
    vector<double> eds8;

    vector<double> eis1;
    vector<double> eis2;
    vector<double> eis3;
    vector<double> eis4;
    vector<double> eis5;
    vector<double> eis6;
    vector<double> eis7;
    vector<double> eis8;
};

class VOLIRO_CTRL{
    public:
        VOLIRO_CTRL();
        void run();
        void pose_cb(geometry_msgs::PoseStamped);
        void arm_wrench_cb(geometry_msgs::Wrench);
        void vel_cb(geometry_msgs::TwistStamped);
        void camera_cb(geometry_msgs::Pose);
        void camera_flag_cb(std_msgs::Bool);
        void vs_cb(std_msgs::Float64MultiArray);
        void term_cb(std_msgs::Bool);
        void feedback();
        void quaternion_error();
        void position_control();
        void attitude_control();
        void allocation();
        void load_parameters();
        void desired_trajectory();
        void ibvs();
    	void writeclass(const vector<double>& data, std::ofstream & pfs);
		void fileclose();

    private:
        ros::NodeHandle _nh;
        ros::Subscriber _pose_sub;
        ros::Subscriber _vel_sub;
        ros::Subscriber _ext_f_sub;
        ros::Subscriber _image_fb_sub;
        ros::Subscriber _tag_flag_sub;
        ros::Subscriber _vs_sub;
        ros::Subscriber _termination_sub;
        ros::Publisher  _motor_vel_pub;
        ros::Publisher  _ext_force_pub;
        ros::Publisher  _vs_completed;
        ros::Publisher  _motor1_tilt_pub;
        ros::Publisher  _motor2_tilt_pub;
        ros::Publisher  _motor3_tilt_pub;
        ros::Publisher  _motor4_tilt_pub;
        ros::Publisher  _ft_pub;

        Eigen::Vector3d _local_pos;
        Eigen::Vector3d _local_vel;
        Eigen::Vector3d _local_ang_vel;
        Eigen::Vector4d _local_quat;
        Eigen::Vector3d _F_des;
        Eigen::Vector3d _M_des;
        Eigen::Vector3d _omega_des;
        CARTESIAN_PLANNER *_trajectory; 
        geometry_msgs::Wrench _ext_f;
        Eigen::VectorXd _arm_ext_f;

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

        geometry_msgs::Wrench _cmd_ft;

        //Control weights
        std::vector<double> _Kp_pos_yaml;
        std::vector<double> _Ki_pos_yaml;
        std::vector<double> _Kd_pos_yaml;
        std::vector<double> _Kp_att_yaml;
        std::vector<double> _Kd_att_yaml;
        std::vector<double> _Kp_cam_yaml;
        std::vector<double> _Kd_cam_yaml;
        std::vector<double> _Ki_cam_yaml;
        Eigen::DiagonalMatrix<double, 3> _Kp_pos;
        Eigen::DiagonalMatrix<double, 3> _Ki_pos;
        Eigen::DiagonalMatrix<double, 3> _Kd_pos;
        Eigen::DiagonalMatrix<double, 3> _Kp_att;
        Eigen::DiagonalMatrix<double, 3> _Kd_att;
        Eigen::MatrixXd _Kp_cam, _Kd_cam, _Ki_cam;

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

        // Camera variables
		geometry_msgs::Pose _tag_pose_fb;
        Eigen::Vector4d _pose_tag_aug;
		Eigen::MatrixXd _K_camera, _P_camera, _T_cb, _T_temp_cb;
		Eigen::Vector3d _image_fb_aug, _image_fb_aug_norm;
        Eigen::VectorXd _local_vel_6d, _local_vel_6d_cf, _local_vel_6d_cf_temp;
        bool _camera_on, _found_tag;
        Eigen::MatrixXd _L_matrix,_L_matrix_p1,_L_matrix_p2,_L_matrix_p3,_L_matrix_p4, _J_image, _gamma_matrix;
		Eigen::MatrixXd _s, _s_dot,_arm_vel_mes_cam_fram, _arm_vel_mes_cam_fram_temp, ROT_Frame;
        Eigen::MatrixXd _pose_tag_corner_aug, _image_fb_aug_corner, _image_fb_aug_norm_corner, _T_tag;
        Eigen::MatrixXd J_img, J_img_temp, J_img_pinv;
        Eigen::Matrix4d _T_drone;
        bool _sec_stage;
        Eigen::VectorXd _tag_corners, _e_i;
        std_msgs::Bool _vs_ok;

        std::ofstream _p_uavFile, _v_uavFile, _rpy_uavFile, _w_uavFile, _vs_uavFile, _ps_uavFile, _psdes_uavFile, _es_uavFile, _eds_uavFile, _eis_uavFile;
        saved_data _uav;
        bool _term_flag;

};