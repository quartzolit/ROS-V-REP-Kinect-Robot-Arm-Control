# ROS-V-REP-Kinect-Robot-Arm-Control
Robot Arm Control using Kinematics, Fuzzy Logic based Controller, and Dynamics + PD Controller

openni_tracker ros package is used to get data from kinect sensor and send to ROS as topics

files on V-REP diretaMA and diretaKalman represents foward kinematics using Moving Average Filter and Kalman Filter.
Direta-Inversa MA-KR and KR-KR represents a file that you can switch to Forward and Inverse Kinematics by clicking on a button,
MA means Moving Average Filter and KR means Robust Kalman Filter. Those files don't need a ros package, only the ros_listeners or
bag files for input data.

For fuzzy control we use fuzz_controller ROS package + vrep arm7220fuzzy.tt for simulation.


For Dynamics you can use rosdym ros package + DYMactionlib.ttt . But this method is inefficient and we suggest to use dymvrep package
instead.

dymvrep package needs DYMpython.ttt for simulation.

We also recommend using subset.bag in Forward Kinematics and Fuzzy Control. For Dynamics and Inverse Kinematics use subset2.bag
