# ndt2_control

Voliro controller applied on tilting NDT2. This package contains a ROS motion controller to simulate with Gazebo. The models use the [rotors_gazebo_plugins](http://wiki.ros.org/rotors_gazebo_plugins) and [px4_gazebo_standalone](>https://github.com/jocacace/px4_gazebo_standalone) to set/get information from the robot. The NDT drone models is stored in the [mav_description](https://github.com/Simone-DAngelo/mav_description) package.

The controller node receives as input the cartesian setpoints and computes a smooth trajectory in the cartesian space. The outputs are the rotors velocities directly sent to the simulated model in gazebo.

Beside the cartesian control also a image based visual servoing algorithm is implented. With a camera sensor simulated in gazebo an AprilTag marker is detected and a regulation in the image space is implemented to nullify the errors in terms of pixel coordinates.

