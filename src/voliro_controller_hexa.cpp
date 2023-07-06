#include "voliro_controller_hexa.h"

void VOLIRO_CTRL::load_parameters(){
    if( !_nh.getParam("rate", _rate)) {
        _rate = 100;
    }
    if( !_nh.getParam("mass", _mass)) {
        _mass = 12.0;
    }
    if( !_nh.getParam("gravity", _gravity)) {
        _gravity = 9.81;
    }
    if( !_nh.getParam("r_com_off_x", _r_com_off_x)) {
        _r_com_off_x = 0.0;
    }
    if( !_nh.getParam("r_com_off_y", _r_com_off_y)) {
        _r_com_off_y = 0.0;
    }
    if( !_nh.getParam("r_com_off_z", _r_com_off_z)) {
        _r_com_off_z = 0.0;
    }
    if( !_nh.getParam("Ixx", _Ixx)) {
        _Ixx = 0.01;
    }
    if( !_nh.getParam("Iyy", _Iyy)) {
        _Iyy = 0.01;
    }
    if( !_nh.getParam("Izz", _Izz)) {
        _Izz = 0.01;
    }
    if( !_nh.getParam("arm", _arm)) {
        _arm = 0.340;
    }
    
    if( !_nh.getParam("Kf", _Kf)) {
        _Kf = 0.000001;
    }
    if( !_nh.getParam("Kq", _Kq)) {
        _Kq = 0.000001;
    }
    _Kq *= _Kf;

    _nh.getParam("rotor_sign", _c_yaml);
    _nh.getParam("rotor_angle", _ang_mot_yaml);
    _nh.getParam("arm_height", _arm_h_yaml);
    _nh.getParam("Kp_pos", _Kp_pos_yaml);
    _nh.getParam("Ki_pos", _Ki_pos_yaml);
    _nh.getParam("Kd_pos", _Kd_pos_yaml);
    _nh.getParam("Kp_att", _Kp_att_yaml);
    _nh.getParam("Kd_att", _Kd_att_yaml);

    _Kp_pos.diagonal() << _Kp_pos_yaml[0], _Kp_pos_yaml[1], _Kp_pos_yaml[2];
    _Kd_pos.diagonal() << _Kd_pos_yaml[0], _Kd_pos_yaml[1], _Kd_pos_yaml[2];
    _Ki_pos.diagonal() << _Ki_pos_yaml[0], _Ki_pos_yaml[1], _Ki_pos_yaml[2];
    _Kp_att.diagonal() << _Kp_att_yaml[0], _Kp_att_yaml[1], _Kp_att_yaml[2];
    _Kd_att.diagonal() << _Kd_att_yaml[0], _Kd_att_yaml[1], _Kd_att_yaml[2];

    _c << _c_yaml[0], _c_yaml[1], _c_yaml[2], _c_yaml[3],_c_yaml[4], _c_yaml[5], _c_yaml[6], _c_yaml[7];
    _ang_mot << _ang_mot_yaml[0], _ang_mot_yaml[1], _ang_mot_yaml[2], _ang_mot_yaml[3],
                _ang_mot_yaml[4], _ang_mot_yaml[5], _ang_mot_yaml[6], _ang_mot_yaml[7];
    _arm_h << _arm_h_yaml[0], _arm_h_yaml[1], _arm_h_yaml[2], _arm_h_yaml[3],
              _arm_h_yaml[4], _arm_h_yaml[5], _arm_h_yaml[6], _arm_h_yaml[7];
}

VOLIRO_CTRL::VOLIRO_CTRL(){
    
    load_parameters();
    
    _pose_sub = _nh.subscribe("/firefly_tilt/local_pose", 1, &VOLIRO_CTRL::pose_cb, this);
    _vel_sub = _nh.subscribe("/firefly_tilt/local_vel", 1, &VOLIRO_CTRL::vel_cb, this);
    _motor_vel_pub = _nh.advertise<std_msgs::Float32MultiArray> ("/firefly_tilt/cmd/motor_vel", 0);
    _motor1_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_rotor_0_joint_controller/command", 0);
    _motor2_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_rotor_1_joint_controller/command", 0);
    _motor3_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_rotor_2_joint_controller/command", 0);
    _motor4_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_rotor_3_joint_controller/command", 0);
    _motor5_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_rotor_4_joint_controller/command", 0);
    _motor6_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_rotor_5_joint_controller/command", 0);
    _first_measure = false;

    _trajectory = new CARTESIAN_PLANNER(_rate/2);

}

void VOLIRO_CTRL::run(){
    boost::thread position_control_t( &VOLIRO_CTRL::position_control, this );
    boost::thread attitude_control_t( &VOLIRO_CTRL::attitude_control, this );
    boost::thread allocation_t( &VOLIRO_CTRL::allocation, this);
    boost::thread trajectory_t( &VOLIRO_CTRL::desired_trajectory, this);

    ros::spin();
}

void VOLIRO_CTRL::pose_cb(geometry_msgs::PoseStamped pose){
    _local_pos << pose.pose.position.x, pose.pose.position.y, pose.pose.position.z;
    _local_quat << pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z;

    if(!_first_measure){
        _des_pos = _local_pos;
        _des_vel << 0.0, 0.0, 0.0;
        _des_acc << 0.0, 0.0, 0.0;
        _des_att = _local_quat; 
        _des_ang_vel << 0.0, 0.0, 0.0;
        _first_measure = true;  
    }
}

void VOLIRO_CTRL::vel_cb(geometry_msgs::TwistStamped twist){
    _local_vel << twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z;
    _local_ang_vel << twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z;

}

void VOLIRO_CTRL::desired_trajectory(){
    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<double> times;
    geometry_msgs::PoseStamped p;
    double t;
    bool finish_traj = false;

    ros::Rate r(10);

    while(!_first_measure){
        r.sleep();
    }

    while(ros::ok()){
        
        p.pose.position.x = _local_pos(0);
        p.pose.position.y = _local_pos(1);
        p.pose.position.z = _local_pos(2);

        p.pose.orientation.w = _local_quat(0);
        p.pose.orientation.x = _local_quat(1);
        p.pose.orientation.y = _local_quat(2);
        p.pose.orientation.z = _local_quat(3);
        t = 0.0;

        poses.push_back(p);
        times.push_back(t);
        
        cout << "Desired position waypoint(x,y,z) in NED (NB: z negative to takeoff): " << endl;
        float x,y,z;
        scanf( "%f%f%f", &x, &y, &z);

        cout << "Desired orientation waypoint(XYZ) in NED: " << endl;
        float roll,pitch,yaw;
        scanf( "%f%f%f", &roll, &pitch, &yaw);

        cout << "Trajectory duration: " << endl;
        double t;
        scanf("%lf", &t);

        Eigen::Vector3d XYZ;
        Eigen::Vector4d des_quat;
        XYZ << roll, pitch, yaw;

        des_quat = utilities::rot2quat( utilities::XYZ2R(XYZ) );

        p.pose.position.x = x;
        p.pose.position.y = y;
        p.pose.position.z = z;

        p.pose.orientation.w = des_quat(0);
        p.pose.orientation.x = des_quat(1);
        p.pose.orientation.y = des_quat(2);
        p.pose.orientation.z = des_quat(3);

        poses.push_back(p);
        times.push_back(t);

        _trajectory->set_waypoints(poses, times);
        _trajectory->compute();

        geometry_msgs::PoseStamped x_traj;
        geometry_msgs::TwistStamped xd_traj;
        geometry_msgs::AccelStamped xdd_traj;

        ros::Rate r_traj(_rate);
        while( _trajectory->getNext(x_traj, xd_traj, xdd_traj) ) { 
            _des_pos << x_traj.pose.position.x, x_traj.pose.position.y, x_traj.pose.position.z;
            _des_vel << xd_traj.twist.linear.x, xd_traj.twist.linear.y, xd_traj.twist.linear.z;
            _des_acc << xdd_traj.accel.linear.x, xdd_traj.accel.linear.y, xdd_traj.accel.linear.z;

            _des_att << x_traj.pose.orientation.w, x_traj.pose.orientation.x, x_traj.pose.orientation.y, x_traj.pose.orientation.z;
            _des_ang_vel << xd_traj.twist.angular.x, xd_traj.twist.angular.y, xd_traj.twist.angular.z;

            r_traj.sleep();
        }

        poses.clear();
        times.clear();
        r.sleep();
    }

}

void VOLIRO_CTRL::position_control(){
    Eigen::Vector3d prop_err;
    Eigen::Vector3d old_prop_err;
    Eigen::Vector3d deriv_err;
    Eigen::Vector3d integ_err;
    Eigen::Vector3d g;

    ros::Rate r(_rate);

    g << 0.0, 0.0, _gravity;
    old_prop_err << 0.0, 0.0, 0.0;

    while(!_first_measure){
        r.sleep();
    }

    while(ros::ok()){
        
        //Tutto in terna NED
        prop_err = _des_pos - _local_pos;
        deriv_err = _des_vel - _local_vel;
        integ_err = (old_prop_err + _Ki_pos * prop_err * (1.0/_rate));
        
        old_prop_err = prop_err;
        //cout << "pos_err: " << prop_err.transpose() << endl;
        //cout << "deriv_err: " << deriv_err.transpose() << endl;
        //cout << "int_err: " << integ_err.transpose() << endl;
        //cout << "\n ----- \n";
        _F_des = utilities::QuatToMat(_local_quat).transpose() * (_Kp_pos * prop_err + _Kd_pos * deriv_err + integ_err - _mass * g + _des_acc);
        //cout << "F_des: " << _F_des.transpose() << endl;
        r.sleep();
    }
}

void VOLIRO_CTRL::quaternion_error(){
    //Eigen::Vector3d local_att;
    Eigen::Vector3d vec_err;
    Eigen::Vector3d vec_des;
    Eigen::Vector3d vec_local;
    double scalar_des;
    double scalar_local;
    double scalar_err;
    double sign;

    //local_att = utilities::R2XYZ(utilities::QuatToMat(_local_quat));
    //des_quat = utilities::rot2quat( utilities::XYZ2R(_des_att) );
    scalar_des = _des_att(0);
    //cout << "scalar_des: " << scalar_des << endl;
    scalar_local = _local_quat(0);
    //cout << "scalar_local: " << scalar_local << endl;
    vec_des << _des_att(1), _des_att(2), _des_att(3);
    //cout << "vec_des: " << vec_des << endl;
    vec_local << _local_quat(1), _local_quat(2), _local_quat(3);
    //cout << "vec_local: " << vec_local << endl;

    scalar_err = scalar_des * scalar_local + vec_des.transpose() * vec_local;
    vec_err = scalar_local * vec_des - scalar_des * vec_local + vec_des.cross(-1*vec_local);

    if(scalar_err >= 0)
        sign = 1.0;
    else if(scalar_err < 0)
        sign = -1.0;

    //cout << "scalar_err: " << scalar_err << endl;
    //cout << "att_err: " << vec_err.transpose() << endl;

    _omega_des = _Kp_att * sign* vec_err;
}

void VOLIRO_CTRL::attitude_control(){
    Eigen::Matrix3d J;
    Eigen::Vector3d r_com_off;

    ros::Rate r(_rate);

    J << _Ixx, 0.0, 0.0, 0.0, _Iyy, 0.0, 0.0, 0.0, _Izz;
    r_com_off << _r_com_off_x, _r_com_off_y, _r_com_off_z;

    while(!_first_measure){
        r.sleep();
    }

    while(ros::ok()){
        
        quaternion_error();
        //cout << "omega_des: " << _omega_des.transpose() << endl;
        _M_des = utilities::QuatToMat(_local_quat).transpose() *(_Kd_att * (_omega_des - _local_ang_vel) - r_com_off.cross(_F_des) + _local_ang_vel.cross(J*_local_ang_vel));
        //cout << "M_des: " << _M_des.transpose() << endl;
        //cout << "\n ----- \n";
        r.sleep();
    }

}

void VOLIRO_CTRL::allocation(){
    Eigen::MatrixXd A_static_pinv;
    Eigen::Matrix<double, 6, 12> A_static;
    Eigen::Matrix<double, 12, 1> V_dec;
    Eigen::Matrix<double, 6, 1> FM_des;
    double Kf_inv;

    ros::Rate r(_rate);

    A_static.row(0) << 0, -sin(_ang_mot[0]), 0, -sin(_ang_mot[1]),
                       0, -sin(_ang_mot[2]), 0, -sin(_ang_mot[3]),
                       0, -sin(_ang_mot[4]), 0, -sin(_ang_mot[5]);

    A_static.row(1) << 0, cos(_ang_mot[0]), 0, cos(_ang_mot[1]),
                       0, cos(_ang_mot[2]), 0, cos(_ang_mot[3]),
                       0, cos(_ang_mot[4]), 0, cos(_ang_mot[5]);

    A_static.row(2) << -1, 0, -1, 0, -1, 0, -1, 0, -1, 0, -1, 0;
    
    A_static.row(3) << -_arm*sin(_ang_mot[0]), - _arm_h[0]*cos(_ang_mot[0]) - (_c[0]*_Kq*sin(_ang_mot[0]))/_Kf,
                       -_arm*sin(_ang_mot[1]), - _arm_h[1]*cos(_ang_mot[1]) - (_c[1]*_Kq*sin(_ang_mot[1]))/_Kf,
                       -_arm*sin(_ang_mot[2]), - _arm_h[2]*cos(_ang_mot[2]) - (_c[2]*_Kq*sin(_ang_mot[2]))/_Kf,
                       -_arm*sin(_ang_mot[3]), - _arm_h[3]*cos(_ang_mot[3]) - (_c[3]*_Kq*sin(_ang_mot[3]))/_Kf,
                       -_arm*sin(_ang_mot[4]), - _arm_h[4]*cos(_ang_mot[4]) - (_c[4]*_Kq*sin(_ang_mot[4]))/_Kf,
                       -_arm*sin(_ang_mot[5]), - _arm_h[5]*cos(_ang_mot[5]) - (_c[5]*_Kq*sin(_ang_mot[5]))/_Kf;

    A_static.row(4) << _arm*cos(_ang_mot[0]), (_c[0]*_Kq*cos(_ang_mot[0]))/_Kf - _arm_h[0]*sin(_ang_mot[0]),
                       _arm*cos(_ang_mot[1]), (_c[1]*_Kq*cos(_ang_mot[1]))/_Kf - _arm_h[1]*sin(_ang_mot[1]),
                       _arm*cos(_ang_mot[2]), (_c[2]*_Kq*cos(_ang_mot[2]))/_Kf - _arm_h[2]*sin(_ang_mot[2]),
                       _arm*cos(_ang_mot[3]), (_c[3]*_Kq*cos(_ang_mot[3]))/_Kf - _arm_h[3]*sin(_ang_mot[3]),
                       _arm*cos(_ang_mot[4]), (_c[4]*_Kq*cos(_ang_mot[4]))/_Kf - _arm_h[4]*sin(_ang_mot[4]),
                       _arm*cos(_ang_mot[5]), (_c[5]*_Kq*cos(_ang_mot[5]))/_Kf - _arm_h[5]*sin(_ang_mot[5]);

    A_static.row(5) << -(_c[0]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[0]),2) + _arm*pow(sin(_ang_mot[0]),2),
                       -(_c[1]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[1]),2) + _arm*pow(sin(_ang_mot[1]),2),
                       -(_c[2]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[2]),2) + _arm*pow(sin(_ang_mot[2]),2),
                       -(_c[3]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[3]),2) + _arm*pow(sin(_ang_mot[3]),2),
                       -(_c[4]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[4]),2) + _arm*pow(sin(_ang_mot[4]),2),
                       -(_c[5]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[5]),2) + _arm*pow(sin(_ang_mot[5]),2);


    _rotors_vel.data.resize(6);
    Kf_inv = 1.0/_Kf;
    
    //Pseudoinverse evaluation
    double epsilon = 0.001;
    Eigen::JacobiSVD<MatrixXd> svd(A_static, Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(A_static.cols(), A_static.rows()) *svd.singularValues().array().abs()(0);
    A_static_pinv = svd.matrixV() * ((svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0)).matrix().asDiagonal() * svd.matrixU().adjoint();
 
    while(!_first_measure){
        r.sleep();
    }

    while(ros::ok()){
        
        FM_des.block<3,1>(0,0) = _F_des;
        FM_des.block<3,1>(3,0) = _M_des;

        V_dec = A_static_pinv * FM_des;

        _rotors_vel.data[0] =  sqrt(Kf_inv * sqrt(pow(V_dec[0],2)  + pow(V_dec[1], 2)));
        _rotors_vel.data[1] =  sqrt(Kf_inv * sqrt(pow(V_dec[2],2)  + pow(V_dec[3], 2)));
        _rotors_vel.data[2] =  sqrt(Kf_inv * sqrt(pow(V_dec[4],2)  + pow(V_dec[5], 2)));
        _rotors_vel.data[3] =  sqrt(Kf_inv * sqrt(pow(V_dec[6],2)  + pow(V_dec[7], 2)));
        _rotors_vel.data[4] =  sqrt(Kf_inv * sqrt(pow(V_dec[8],2)  + pow(V_dec[9], 2)));
        _rotors_vel.data[5] =  sqrt(Kf_inv * sqrt(pow(V_dec[10],2) + pow(V_dec[11], 2)));

        _tilt_m1.data = atan2(V_dec[1],  V_dec[0]);
        _tilt_m2.data = atan2(V_dec[3],  V_dec[2]);
        _tilt_m3.data = atan2(V_dec[5],  V_dec[4]);
        _tilt_m4.data = atan2(V_dec[7],  V_dec[6]);
        _tilt_m5.data = atan2(V_dec[9],  V_dec[8]);
        _tilt_m6.data = atan2(V_dec[11],  V_dec[10]);
         
        for(int i=0; i<6; i++){
            //_rotors_vel.data[i] = (_rotors_vel.data[i] * 9.54 * 1100.0)/5000.0;
            
            if(_rotors_vel.data[i] > 800)
                _rotors_vel.data[i] = 800;
        }

        
        if(_tilt_m1.data > 0.785) 
            _tilt_m1.data = 0.785;
        else if(_tilt_m1.data < -0.785)
            _tilt_m1.data = -0.785;

        if(_tilt_m2.data > 0.785) 
            _tilt_m2.data = 0.785;
        else if(_tilt_m2.data < -0.785)
            _tilt_m2.data = -0.785;

        if(_tilt_m3.data > 0.785) 
            _tilt_m3.data = 0.785;
        else if(_tilt_m3.data < -0.785)
            _tilt_m3.data = -0.785;

        if(_tilt_m4.data > 0.785) 
            _tilt_m4.data = 0.785;
        else if(_tilt_m4.data < -0.785)
            _tilt_m4.data = -0.785;

        if(_tilt_m5.data > 0.785) 
            _tilt_m5.data = 0.785;
        else if(_tilt_m5.data < -0.785)
            _tilt_m5.data = -0.785;

        if(_tilt_m6.data > 0.785) 
            _tilt_m6.data = 0.785;
        else if(_tilt_m6.data < -0.785)
            _tilt_m6.data = -0.785;        
        
        _motor_vel_pub.publish(_rotors_vel);
        _motor1_tilt_pub.publish(_tilt_m1);
        _motor2_tilt_pub.publish(_tilt_m2);
        _motor3_tilt_pub.publish(_tilt_m3);
        _motor4_tilt_pub.publish(_tilt_m4);
        _motor5_tilt_pub.publish(_tilt_m5);
        _motor6_tilt_pub.publish(_tilt_m6);

        r.sleep();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voliro_controller");
    VOLIRO_CTRL vol;
    vol.run();

    return 0;
}