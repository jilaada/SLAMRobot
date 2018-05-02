# CMake generated Testfile for 
# Source directory: /home/robot/Desktop/workspace/src/urg_node-indigo-devel
# Build directory: /home/robot/Desktop/workspace/build/urg_node-indigo-devel
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_urg_node_roslint_package "/home/robot/Desktop/workspace/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/robot/Desktop/workspace/build/test_results/urg_node/roslint-urg_node.xml" "--working-dir" "/home/robot/Desktop/workspace/build/urg_node-indigo-devel" "--return-code" "/opt/ros/kinetic/share/roslint/cmake/../../../lib/roslint/test_wrapper /home/robot/Desktop/workspace/build/test_results/urg_node/roslint-urg_node.xml make roslint_urg_node")
add_test(_ctest_urg_node_roslaunch-check_launch_urg_lidar.launch "/home/robot/Desktop/workspace/build/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/robot/Desktop/workspace/build/test_results/urg_node/roslaunch-check_launch_urg_lidar.launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/robot/Desktop/workspace/build/test_results/urg_node" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/robot/Desktop/workspace/build/test_results/urg_node/roslaunch-check_launch_urg_lidar.launch.xml' '/home/robot/Desktop/workspace/src/urg_node-indigo-devel/launch/urg_lidar.launch' ")
