echo 'robot_3c3' | sudo -kS chmod 777 /dev/ttyUSB0
echo 'robot_3c3' | sudo -kS chmod 777 /dev/ttyACM0
source devel/setup.bash
roslaunch robot_driver launchRosariaLaser.launch
