#include "voliro_controller.h"

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
    _nh.getParam("Kp_cam", _Kp_cam_yaml);
    _nh.getParam("Kd_cam", _Kd_cam_yaml);
    _nh.getParam("Ki_cam", _Ki_cam_yaml);

    _Kp_pos.diagonal() << _Kp_pos_yaml[0], _Kp_pos_yaml[1], _Kp_pos_yaml[2];
    _Kd_pos.diagonal() << _Kd_pos_yaml[0], _Kd_pos_yaml[1], _Kd_pos_yaml[2];
    _Ki_pos.diagonal() << _Ki_pos_yaml[0], _Ki_pos_yaml[1], _Ki_pos_yaml[2];
    _Kp_att.diagonal() << _Kp_att_yaml[0], _Kp_att_yaml[1], _Kp_att_yaml[2];
    _Kd_att.diagonal() << _Kd_att_yaml[0], _Kd_att_yaml[1], _Kd_att_yaml[2];

    _Kp_cam.resize(8,8);
	_Kd_cam.resize(8,8);
	_Ki_cam.resize(8,8);
	_Kp_cam <<  _Kp_cam_yaml[0],              0,               0,               0,               0,               0,               0,               0,
	                          0,_Kp_cam_yaml[1],               0,               0,               0,               0,               0,               0,
						      0,              0, _Kp_cam_yaml[2],               0,               0,               0,               0,               0,
						      0,              0,               0, _Kp_cam_yaml[3],               0,               0,               0,               0,
						      0,              0,               0,               0, _Kp_cam_yaml[4],               0,               0,               0,
						      0,              0,               0,               0,               0, _Kp_cam_yaml[5],               0,               0,
						      0,              0,               0,               0,               0,               0, _Kp_cam_yaml[6],               0,
						      0,              0,               0,               0,               0,               0,               0, _Kp_cam_yaml[7];
	_Kd_cam <<  _Kd_cam_yaml[0],              0,               0,               0,               0,               0,               0,               0,
	                          0,_Kd_cam_yaml[1],               0,               0,               0,               0,               0,               0,
						      0,              0, _Kd_cam_yaml[2],               0,               0,               0,               0,               0,
						      0,              0,               0, _Kd_cam_yaml[3],               0,               0,               0,               0,
						      0,              0,               0,               0, _Kd_cam_yaml[4],               0,               0,               0,
						      0,              0,               0,               0,               0, _Kd_cam_yaml[5],               0,               0,
						      0,              0,               0,               0,               0,               0, _Kd_cam_yaml[6],               0,
						      0,              0,               0,               0,               0,               0,               0, _Kd_cam_yaml[7];

    _Ki_cam <<  _Ki_cam_yaml[0],              0,               0,               0,               0,               0,               0,               0,
	                          0,_Ki_cam_yaml[1],               0,               0,               0,               0,               0,               0,
						      0,              0, _Ki_cam_yaml[2],               0,               0,               0,               0,               0,
						      0,              0,               0, _Ki_cam_yaml[3],               0,               0,               0,               0,
						      0,              0,               0,               0, _Ki_cam_yaml[4],               0,               0,               0,
						      0,              0,               0,               0,               0, _Ki_cam_yaml[5],               0,               0,
						      0,              0,               0,               0,               0,               0, _Ki_cam_yaml[6],               0,
						      0,              0,               0,               0,               0,               0,               0, _Ki_cam_yaml[7];


    _c << _c_yaml[0], _c_yaml[1], _c_yaml[2], _c_yaml[3],_c_yaml[4], _c_yaml[5], _c_yaml[6], _c_yaml[7];
    _ang_mot << _ang_mot_yaml[0], _ang_mot_yaml[1], _ang_mot_yaml[2], _ang_mot_yaml[3],
                _ang_mot_yaml[4], _ang_mot_yaml[5], _ang_mot_yaml[6], _ang_mot_yaml[7];
    _arm_h << _arm_h_yaml[0], _arm_h_yaml[1], _arm_h_yaml[2], _arm_h_yaml[3],
              _arm_h_yaml[4], _arm_h_yaml[5], _arm_h_yaml[6], _arm_h_yaml[7];
}

VOLIRO_CTRL::VOLIRO_CTRL(){

    load_parameters();

    _pose_sub = _nh.subscribe("/ndt2/local_pose", 1, &VOLIRO_CTRL::pose_cb, this);
    _vel_sub = _nh.subscribe("/ndt2/local_vel", 1, &VOLIRO_CTRL::vel_cb, this);
    _ext_f_sub = _nh.subscribe("/arm/wrench", 1, &VOLIRO_CTRL::arm_wrench_cb, this);
    _image_fb_sub = _nh.subscribe("/ndt2/camera/tag_pose", 1, &VOLIRO_CTRL::camera_cb, this);
    _tag_flag_sub = _nh.subscribe("/ndt2/camera/tag_flag", 1, &VOLIRO_CTRL::camera_flag_cb, this);
    _vs_sub = _nh.subscribe("/ndt2/tag_pos_px_corner", 1, &VOLIRO_CTRL::vs_cb, this);
    _motor_vel_pub = _nh.advertise<std_msgs::Float32MultiArray> ("/ndt2/cmd/motor_vel", 0);
    _ext_force_pub = _nh.advertise<geometry_msgs::Wrench> ("/ndt2/wrench", 0);
    _motor1_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_motor_1/command", 0);
    _motor2_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_motor_2/command", 0);
    _motor3_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_motor_3/command", 0);
    _motor4_tilt_pub = _nh.advertise<std_msgs::Float64> ("/tilt_motor_4/command", 0);
    _vs_completed = _nh.advertise<std_msgs::Bool> ("/ndt2/ibvs/flag", 0);
    _first_measure = false;

    _trajectory = new CARTESIAN_PLANNER(_rate/2);
    _arm_ext_f.resize(6);
	_image_fb_aug << 0,0,0;
	_image_fb_aug_norm << 0,0,0;

	_K_camera.resize(3,3);
	_P_camera.resize(3,4);
	_T_cb.resize(4,4);
    _K_camera <<    476.7030836014194,               0.0,     400.5,
								  0.0, 476.7030836014194,     300.5,
								  0.0,               0.0,       1.0;

	_P_camera << 1,0,0,0,
		  		 0,1,0,0,
		  		 0,0,1,0; // PI matrix

    _T_cb <<    0, 0, 1, 0.085, // From NED uav frame to camera frame
                1, 0, 0,     0,
                0, 1, 0, 0.060,
                0, 0, 0,     1;

    _local_vel_6d.resize(6);
    _camera_on = false;

	_L_matrix.resize(8,6);
	_L_matrix.setZero();
	_L_matrix_p1.resize(2,6);
	_L_matrix_p1.setZero();
	_L_matrix_p2.resize(2,6);
	_L_matrix_p2.setZero();
	_L_matrix_p3.resize(2,6);
	_L_matrix_p3.setZero();
	_L_matrix_p4.resize(2,6);
	_L_matrix_p4.setZero();
	_J_image.resize(8,6);
	_J_image.setZero();
	_arm_vel_mes_cam_fram_temp.resize(6,1);
	_arm_vel_mes_cam_fram_temp.setZero();
	_arm_vel_mes_cam_fram.resize(6,1);
	_arm_vel_mes_cam_fram.setZero();
	_s_dot.resize(8,1);
	_s_dot.setZero();
	ROT_Frame.resize(6,6);

	_gamma_matrix.resize(6,6);
	_gamma_matrix.setIdentity();
	_gamma_matrix = -1*_gamma_matrix;

	_pose_tag_corner_aug.resize(4,4);
	_image_fb_aug_corner.resize(3,4);
	_image_fb_aug_norm_corner.resize(3,4);

	_local_vel_6d_cf.resize(6);
	_local_vel_6d_cf_temp.resize(6);
    _s.resize(8,1);
    _sec_stage=false;
    _T_tag.resize(4,4);
    _T_tag.setIdentity();
    _tag_corners.resize(8);
    _tag_corners.setZero();
    _e_i.resize(8);
    _e_i.setZero();
    _T_drone.setIdentity();
    _vs_ok.data = false;
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
    // cout<<"ndt rpy: "<<utilities::MatToRpy(utilities::QuatToMat(_local_quat)).transpose()<<endl;
    _T_drone.block<3,3>(0,0) = (utilities::QuatToMat(_local_quat));
    _T_drone.block<3,1>(0,3) << _local_pos(0), _local_pos(1), _local_pos(2);
    _T_drone.block<1,4>(3,0) << 0,0,0,1;
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
    // _local_vel_6d << _local_vel[0],_local_vel[1],_local_vel[2], _local_ang_vel[0], _local_ang_vel[1], _local_ang_vel[2];
    _local_vel_6d << _local_ang_vel[0], _local_ang_vel[1], _local_ang_vel[2], _local_vel[0],_local_vel[1],_local_vel[2];
    //CHECK
}

void VOLIRO_CTRL::camera_flag_cb(std_msgs::Bool flag_msg){
	_found_tag = flag_msg.data;
}

// Callback image servoing
void VOLIRO_CTRL::camera_cb(geometry_msgs::Pose pose){
    _tag_pose_fb.position.x = pose.position.x;
    _tag_pose_fb.position.y = pose.position.y;
    _tag_pose_fb.position.z = pose.position.z;
    _tag_pose_fb.orientation.w = pose.orientation.w;
    _tag_pose_fb.orientation.x = pose.orientation.x;
    _tag_pose_fb.orientation.y = pose.orientation.y;
    _tag_pose_fb.orientation.z = pose.orientation.z;
    _T_tag.block<3,3>(0,0) = utilities::QuatToMat(Eigen::Vector4d(_tag_pose_fb.orientation.w,_tag_pose_fb.orientation.x,_tag_pose_fb.orientation.y,_tag_pose_fb.orientation.z));
    _T_tag.block<3,1>(0,3) << _tag_pose_fb.position.x, _tag_pose_fb.position.y, _tag_pose_fb.position.z;
    //cout<<"tag rpy: "<<(_T_cb.block<3,3>(0,0)*utilities::MatToRpy(utilities::QuatToMat(Eigen::Vector4d(_tag_pose_fb.orientation.w,_tag_pose_fb.orientation.x,_tag_pose_fb.orientation.y,_tag_pose_fb.orientation.z)))).transpose()<<endl;
    _T_tag = _T_drone*_T_cb*_T_tag;
    // cout<<"T_tag: "<<_T_tag<<endl;
    _camera_on = true;
    feedback();

}

void VOLIRO_CTRL::vs_cb(std_msgs::Float64MultiArray msg){
    _tag_corners.resize(msg.data[0]);
    for(int i = 0;i<_tag_corners.size();i++){
        _tag_corners(i) = msg.data[i+1];
    }
}

void VOLIRO_CTRL::feedback(){
    Eigen::VectorXd vel,vel_temp;
    vel.resize(6);
    vel_temp.resize(6);

    _pose_tag_aug << _tag_pose_fb.position.x, _tag_pose_fb.position.y, _tag_pose_fb.position.z, 1; //object pose in camera frame (x,y,z,1)
    _image_fb_aug = _K_camera*_P_camera*_pose_tag_aug;
    _image_fb_aug = _image_fb_aug/_image_fb_aug(2);											       //object pose in pixels coordinates (X_i,Y_i,1)
    _image_fb_aug_norm = _K_camera.inverse()*_image_fb_aug;										   //object pose in normalized pixels coordinates (X,Y,1)

    _image_fb_aug_corner.block<1,4>(2,0) << 1,1,1,1;
    _image_fb_aug_corner.block<2,1>(0,0) << _tag_corners(1), _tag_corners(0);
    _image_fb_aug_corner.block<2,1>(0,1) << _tag_corners(3), _tag_corners(2);
    _image_fb_aug_corner.block<2,1>(0,2) << _tag_corners(5), _tag_corners(4);
    _image_fb_aug_corner.block<2,1>(0,3) << _tag_corners(7), _tag_corners(6);
    
	// Augmented normalized pixels corners coordinates
	_image_fb_aug_norm_corner.block<3,1>(0,0) = _K_camera.inverse()*_image_fb_aug_corner.block<3,1>(0,0);
	_image_fb_aug_norm_corner.block<3,1>(0,1) = _K_camera.inverse()*_image_fb_aug_corner.block<3,1>(0,1);
	_image_fb_aug_norm_corner.block<3,1>(0,2) = _K_camera.inverse()*_image_fb_aug_corner.block<3,1>(0,2);
	_image_fb_aug_norm_corner.block<3,1>(0,3) = _K_camera.inverse()*_image_fb_aug_corner.block<3,1>(0,3);
    _s << _image_fb_aug_norm_corner(0,0), _image_fb_aug_norm_corner(1,0), _image_fb_aug_norm_corner(0,1), _image_fb_aug_norm_corner(1,1), _image_fb_aug_norm_corner(0,2), _image_fb_aug_norm_corner(1,2), _image_fb_aug_norm_corner(0,3), _image_fb_aug_norm_corner(1,3); 

	// Interaction Matrix
	_L_matrix_p1 <<  -1/_pose_tag_aug(2), 					 0,  _image_fb_aug_norm_corner(0,0)/_pose_tag_aug(2),  _image_fb_aug_norm_corner(0,0)*_image_fb_aug_norm_corner(1,0), 		               -(1+pow(_image_fb_aug_norm_corner(0,0),2)),		 _image_fb_aug_norm_corner(1,0),
									   0,  -1/_pose_tag_aug(2),  _image_fb_aug_norm_corner(1,0)/_pose_tag_aug(2),                      (1+pow(_image_fb_aug_norm_corner(1,0),2)),  -_image_fb_aug_norm_corner(0,0)*_image_fb_aug_norm_corner(1,0),		-_image_fb_aug_norm_corner(0,0);

	_L_matrix_p2 <<  -1/_pose_tag_aug(2), 					 0,  _image_fb_aug_norm_corner(0,1)/_pose_tag_aug(2),  _image_fb_aug_norm_corner(0,1)*_image_fb_aug_norm_corner(1,1), 					   -(1+pow(_image_fb_aug_norm_corner(0,1),2)),		 _image_fb_aug_norm_corner(1,1),
									   0,  -1/_pose_tag_aug(2),  _image_fb_aug_norm_corner(1,1)/_pose_tag_aug(2),					   (1+pow(_image_fb_aug_norm_corner(1,1),2)),  -_image_fb_aug_norm_corner(0,1)*_image_fb_aug_norm_corner(1,1),		-_image_fb_aug_norm_corner(0,1);

	_L_matrix_p3 <<  -1/_pose_tag_aug(2), 				     0,  _image_fb_aug_norm_corner(0,2)/_pose_tag_aug(2),  _image_fb_aug_norm_corner(0,2)*_image_fb_aug_norm_corner(1,2), 					   -(1+pow(_image_fb_aug_norm_corner(0,2),2)),		 _image_fb_aug_norm_corner(1,2),
									   0,  -1/_pose_tag_aug(2),	 _image_fb_aug_norm_corner(1,2)/_pose_tag_aug(2),					   (1+pow(_image_fb_aug_norm_corner(1,2),2)),  -_image_fb_aug_norm_corner(0,2)*_image_fb_aug_norm_corner(1,2),		-_image_fb_aug_norm_corner(0,2);

	_L_matrix_p4 <<  -1/_pose_tag_aug(2), 		             0,  _image_fb_aug_norm_corner(0,3)/_pose_tag_aug(2),  _image_fb_aug_norm_corner(0,3)*_image_fb_aug_norm_corner(1,3), 					   -(1+pow(_image_fb_aug_norm_corner(0,3),2)),		 _image_fb_aug_norm_corner(1,3),
									   0,  -1/_pose_tag_aug(2),	 _image_fb_aug_norm_corner(1,3)/_pose_tag_aug(2),	   				   (1+pow(_image_fb_aug_norm_corner(1,3),2)),  -_image_fb_aug_norm_corner(0,3)*_image_fb_aug_norm_corner(1,3),		-_image_fb_aug_norm_corner(0,3);

	_L_matrix.block<2,6>(0,0) = _L_matrix_p1;
	_L_matrix.block<2,6>(2,0) = _L_matrix_p2;
	_L_matrix.block<2,6>(4,0) = _L_matrix_p3;
	_L_matrix.block<2,6>(6,0) = _L_matrix_p4;

	_gamma_matrix.block<3,3>(0,3) = utilities::skew(Eigen::Vector3d(_pose_tag_aug(0),_pose_tag_aug(1),_pose_tag_aug(2)));

	_J_image = _L_matrix*_gamma_matrix.inverse();			                                            // Used to map the velocity of the object wrt the camera and the velocity of the feature point s in image plane
    vel_temp = utilities::Ad_f(_T_cb.inverse())*_local_vel_6d;
    vel << vel_temp(3), vel_temp(4), vel_temp(5), vel_temp(0), vel_temp(1), vel_temp(2);
    _s_dot = _J_image*vel; 
}

// Callback arm wrench feedback
void VOLIRO_CTRL::arm_wrench_cb( const geometry_msgs::Wrench wrench_msg ) {
    Eigen::VectorXd _arm_ext_f_temp; 
    _arm_ext_f_temp.resize(6);
    _arm_ext_f_temp << wrench_msg.torque.x, wrench_msg.torque.y, wrench_msg.torque.z, wrench_msg.force.x, wrench_msg.force.y, wrench_msg.force.z;
	// _arm_ext_f_temp = (utilities::Ad_f(utilities::rotx_T(-M_PI))*_arm_ext_f_temp);
    // _arm_ext_f << _arm_ext_f_temp(3),_arm_ext_f_temp(4), _arm_ext_f_temp(5),_arm_ext_f_temp(0),_arm_ext_f_temp(1), _arm_ext_f_temp(2);
    _arm_ext_f << _arm_ext_f_temp(3),_arm_ext_f_temp(4), _arm_ext_f_temp(5),0,0,0;

	// cout<<"Contact forces: "<<(_arm_ext_f).transpose()<<endl;
}

void VOLIRO_CTRL::desired_trajectory(){
    std::vector<geometry_msgs::PoseStamped> poses;
    std::vector<double> times;
    geometry_msgs::PoseStamped p;
    double t;
    bool finish_traj = false;
    bool takeoff = false;
    int cont = 0;

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
        
        float x,y,z;
        float roll,pitch,yaw;
        double t;
        string input;
        //
        if (!takeoff && _first_measure){
            cout<<"Want to takeoff?"<<endl;
		    getline( cin, input);
            if(input == "y"){
                x = 0.5;
                y = 0;
                z = -1.0;
                roll = 0;
                pitch = 0;
                yaw = 0;
                t = 10;
            }
            takeoff = true;
            cont++;
        }
        else if ( _camera_on && !_sec_stage && _first_measure && takeoff) {
            x = _T_tag(0,3) - (0.08*4-0.055) -0.085;
            y = _T_tag(1,3);
            z = _T_tag(2,3) + 0.06;
            roll = 0.0;
            pitch = 0.0;
            yaw = 0.0;
            cout<<"des pos: "<<x<<", "<<y<<", "<<z<<endl;
            cout<<"des or: "<<roll<<", "<<pitch<<", "<<yaw<<endl;
            cout << "Trajectory duration: " << endl;
            scanf("%lf", &t);
            cont++;
        }
        //
        else{
            // xyz: 2.42 -0.45 -1.44 to approach apriltag
            cout << "Desired position waypoint(x,y,z) in NED (NB: z negative to takeoff): " << endl;
            scanf( "%f%f%f", &x, &y, &z);

            cout << "Desired orientation waypoint(XYZ) in NED: " << endl;
            scanf( "%f%f%f", &roll, &pitch, &yaw);
            
            cout << "Trajectory duration: " << endl;
            scanf("%lf", &t);
        }

               
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
        if(cont==2 && _camera_on) {
            _sec_stage = true;
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
        _vs_completed.publish(_vs_ok);
        if (_sec_stage && _camera_on) {
            ibvs();
            // cout<<"ibvs"<<endl;
        }
        else{
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
            // cout << "F_des: " << _F_des.transpose() << endl;
        }
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
        if (!_sec_stage ){ 
            _M_des = utilities::QuatToMat(_local_quat).transpose() *(_Kd_att * (_omega_des - _local_ang_vel) - r_com_off.cross(_F_des) + _local_ang_vel.cross(J*_local_ang_vel));
        }
        //cout << "M_des: " << _M_des.transpose() << endl;
        //cout << "\n ----- \n";
        r.sleep();
    }

}

void::VOLIRO_CTRL::ibvs(){

    Eigen::VectorXd e_s, e_sdot;
    Eigen::MatrixXd kpc, kdc,kic,sd,vs,vs_temp, g;
    Eigen::Matrix3d J;
    Eigen::Vector3d NL_comp, F_des, M_des;
    vs.resize(6,1);
    vs_temp.resize(6,1);
    sd.resize(8,1);
    // tag 0.16
    // sd << -0.297868, 0.0736275,  0.308222, 0.0736275,  0.308222, -0.532462, -0.297868, -0.532462;       //tag vicino in alto

    // sd << -0.299125,  0.300783,  0.307344,  0.300783,  0.307344, -0.305687, -0.299125, -0.305687;    //tag centrato
    // sd <<  -0.121786,  0.121486,  0.122886,  0.121486,  0.122886, -0.123187, -0.121786, -0.123187;   //tag centrato lontano

    // tag 0.08 --pose des: 2.39 -0.45 -1.34
    sd << -0.146116, -0.232102,  0.156216, -0.231255,  0.158847, -0.534022,  -0.14591, -0.534183; // tag alto

    e_s = -(sd -_s);
    e_sdot = (-_s_dot);
    _e_i = _e_i+e_s*0.01;
    kpc.resize(8,8);
    kdc.resize(8,8);
    kic.resize(8,8);
    kpc = _Kp_cam;
    kdc = _Kd_cam;
    kic = _Ki_cam;

    g.resize(6,1);
    g << 0.0, 0.0, 0.0, 0.0, 0.0, _gravity;
    J << _Ixx, 0.0, 0.0, 0.0, _Iyy, 0.0, 0.0, 0.0, _Izz;
    NL_comp = _local_ang_vel.cross(J*_local_ang_vel);

    // vs_temp = ((_J_image.completeOrthogonalDecomposition()).pseudoInverse())*(kpc*e_s+kdc*e_sdot+kic*_e_i); //<- not working anymore
    // vs_temp = (_J_image.transpose()*_J_image).inverse()*_J_image.transpose()*(kpc*e_s+kdc*e_sdot+kic*_e_i); // <- 1st sol
    vs_temp = (_Kp_cam*e_s+_Kd_cam*e_sdot+_Ki_cam*_e_i);                                                        // <- 2nd sol
    J_img = _L_matrix*_gamma_matrix.inverse();                                                                  // <- 2nd sol
    J_img_temp = J_img.completeOrthogonalDecomposition().pseudoInverse();                                       // <- 2nd sol
    J_img_pinv = J_img_temp;                                                                                    // <- 2nd sol
    vs_temp = J_img_pinv*vs_temp;                                                                               // <- 2nd sol
    vs << vs_temp(3), vs_temp(4), vs_temp(5), vs_temp(0), vs_temp(1), vs_temp(2);
    F_des = utilities::QuatToMat(_local_quat).transpose()*(-_mass*g+_mass*utilities::Ad_f(_T_cb)*vs).block<3,1>(3,0);
    _F_des = F_des;
    M_des =  utilities::QuatToMat(_local_quat).transpose()*((NL_comp)+(J*((utilities::Ad_f(_T_cb)*vs).block<3,1>(0,0))));
    _M_des = M_des;
    if(_vs_ok.data == false){
        if(abs(e_s(0))<0.001 || abs(e_s(1))<0.001 || abs(e_s(2))<0.001 || abs(e_s(3))<0.001 || abs(e_s(4))<0.001 || abs(e_s(5))<0.001 || abs(e_s(6))<0.001 || abs(e_s(7))<0.001){
            _vs_ok.data = true;
        }
    }
    // _vs_completed.publish(_vs_ok);
    // cout<<"\nT_tag:\n"<<_T_tag<<endl;
}

void VOLIRO_CTRL::allocation(){
    Eigen::MatrixXd A_static_pinv;
    Eigen::Matrix<double, 6, 16> A_static;
    Eigen::Matrix<double, 16, 1> V_dec;
    Eigen::Matrix<double, 6, 1> FM_des;
    double Kf_inv;

    ros::Rate r(_rate);

    A_static.row(0) << 0, -sin(_ang_mot[0]), 0, -sin(_ang_mot[1]),
                       0, -sin(_ang_mot[2]), 0, -sin(_ang_mot[3]),
                       0, -sin(_ang_mot[4]), 0, -sin(_ang_mot[5]),
                       0, -sin(_ang_mot[6]), 0, -sin(_ang_mot[7]);

    A_static.row(1) << 0, cos(_ang_mot[0]), 0, cos(_ang_mot[1]),
                       0, cos(_ang_mot[2]), 0, cos(_ang_mot[3]),
                       0, cos(_ang_mot[4]), 0, cos(_ang_mot[5]),
                       0, cos(_ang_mot[6]), 0, cos(_ang_mot[7]);

    A_static.row(2) << -1, 0, -1, 0, -1, 0, -1, 0,
                       -1, 0, -1, 0, -1, 0, -1, 0;

    A_static.row(3) << -_arm*sin(_ang_mot[0]), - _arm_h[0]*cos(_ang_mot[0]) - (_c[0]*_Kq*sin(_ang_mot[0]))/_Kf,
                       -_arm*sin(_ang_mot[1]), - _arm_h[1]*cos(_ang_mot[1]) - (_c[1]*_Kq*sin(_ang_mot[1]))/_Kf,
                       -_arm*sin(_ang_mot[2]), - _arm_h[2]*cos(_ang_mot[2]) - (_c[2]*_Kq*sin(_ang_mot[2]))/_Kf,
                       -_arm*sin(_ang_mot[3]), - _arm_h[3]*cos(_ang_mot[3]) - (_c[3]*_Kq*sin(_ang_mot[3]))/_Kf,
                       -_arm*sin(_ang_mot[4]), - _arm_h[4]*cos(_ang_mot[4]) - (_c[4]*_Kq*sin(_ang_mot[4]))/_Kf,
                       -_arm*sin(_ang_mot[5]), - _arm_h[5]*cos(_ang_mot[5]) - (_c[5]*_Kq*sin(_ang_mot[5]))/_Kf,
                       -_arm*sin(_ang_mot[6]), - _arm_h[6]*cos(_ang_mot[6]) - (_c[6]*_Kq*sin(_ang_mot[6]))/_Kf,
                       -_arm*sin(_ang_mot[7]), - _arm_h[7]*cos(_ang_mot[7]) - (_c[7]*_Kq*sin(_ang_mot[7]))/_Kf;

    A_static.row(4) << _arm*cos(_ang_mot[0]), (_c[0]*_Kq*cos(_ang_mot[0]))/_Kf - _arm_h[0]*sin(_ang_mot[0]),
                       _arm*cos(_ang_mot[1]), (_c[1]*_Kq*cos(_ang_mot[1]))/_Kf - _arm_h[1]*sin(_ang_mot[1]),
                       _arm*cos(_ang_mot[2]), (_c[2]*_Kq*cos(_ang_mot[2]))/_Kf - _arm_h[2]*sin(_ang_mot[2]),
                       _arm*cos(_ang_mot[3]), (_c[3]*_Kq*cos(_ang_mot[3]))/_Kf - _arm_h[3]*sin(_ang_mot[3]),
                       _arm*cos(_ang_mot[4]), (_c[4]*_Kq*cos(_ang_mot[4]))/_Kf - _arm_h[4]*sin(_ang_mot[4]),
                       _arm*cos(_ang_mot[5]), (_c[5]*_Kq*cos(_ang_mot[5]))/_Kf - _arm_h[5]*sin(_ang_mot[5]),
                       _arm*cos(_ang_mot[6]), (_c[6]*_Kq*cos(_ang_mot[6]))/_Kf - _arm_h[6]*sin(_ang_mot[6]),
                       _arm*cos(_ang_mot[7]), (_c[7]*_Kq*cos(_ang_mot[7]))/_Kf - _arm_h[7]*sin(_ang_mot[7]);

    A_static.row(5) << -(_c[0]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[0]),2) + _arm*pow(sin(_ang_mot[0]),2),
                       -(_c[1]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[1]),2) + _arm*pow(sin(_ang_mot[1]),2),
                       -(_c[2]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[2]),2) + _arm*pow(sin(_ang_mot[2]),2),
                       -(_c[3]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[3]),2) + _arm*pow(sin(_ang_mot[3]),2),
                       -(_c[4]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[4]),2) + _arm*pow(sin(_ang_mot[4]),2),
                       -(_c[5]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[5]),2) + _arm*pow(sin(_ang_mot[5]),2),
                       -(_c[6]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[6]),2) + _arm*pow(sin(_ang_mot[6]),2),
                       -(_c[7]*_Kq)/_Kf, _arm*pow(cos(_ang_mot[7]),2) + _arm*pow(sin(_ang_mot[7]),2);


    _rotors_vel.data.resize(8);
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
        _ext_f.force.x = _F_des[0];
        _ext_f.force.y = _F_des[1];
        _ext_f.force.z = _F_des[2];
        _ext_f.torque.x = _M_des[0];
        _ext_f.torque.y = _M_des[1];
        _ext_f.torque.z = _M_des[2];
        // _ext_force_pub.publish(_ext_f);

        // V_dec = A_static_pinv * (FM_des+_arm_ext_f); // <- check
        V_dec = A_static_pinv * FM_des;

        _rotors_vel.data[0] =  sqrt(Kf_inv * sqrt(pow(V_dec[0],2)  + pow(V_dec[1], 2)));
        _rotors_vel.data[1] =  sqrt(Kf_inv * sqrt(pow(V_dec[2],2)  + pow(V_dec[3], 2)));
        _rotors_vel.data[2] =  sqrt(Kf_inv * sqrt(pow(V_dec[4],2)  + pow(V_dec[5], 2)));
        _rotors_vel.data[3] =  sqrt(Kf_inv * sqrt(pow(V_dec[6],2)  + pow(V_dec[7], 2)));
        _rotors_vel.data[4] =  sqrt(Kf_inv * sqrt(pow(V_dec[8],2)  + pow(V_dec[9], 2)));
        _rotors_vel.data[5] =  sqrt(Kf_inv * sqrt(pow(V_dec[10],2) + pow(V_dec[11], 2)));
        _rotors_vel.data[6] =  sqrt(Kf_inv * sqrt(pow(V_dec[12],2) + pow(V_dec[13], 2)));
        _rotors_vel.data[7] =  sqrt(Kf_inv * sqrt(pow(V_dec[14],2) + pow(V_dec[15], 2)));

        _tilt_m4.data = atan2(V_dec[1],  V_dec[0]);
        _tilt_m1.data = atan2(V_dec[3],  V_dec[2]);
        _tilt_m2.data = atan2(V_dec[5],  V_dec[4]);
        _tilt_m3.data = atan2(V_dec[7],  V_dec[6]);

        // for(int i=0; i<8; i++){
        //     //_rotors_vel.data[i] = (_rotors_vel.data[i] * 9.54 * 1100.0)/5000.0;

        //     if(_rotors_vel.data[i] > 500)
        //         _rotors_vel.data[i] = 500;
        // }


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

        _motor_vel_pub.publish(_rotors_vel);
        _motor1_tilt_pub.publish(_tilt_m1);
        _motor2_tilt_pub.publish(_tilt_m2);
        _motor3_tilt_pub.publish(_tilt_m3);
        _motor4_tilt_pub.publish(_tilt_m4);

        r.sleep();
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voliro_controller");
    VOLIRO_CTRL vol;
    vol.run();

    return 0;
}