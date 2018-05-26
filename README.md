--------------------------------------------------------------------------------------------------------------
#COMPSYS 726 ASSIGNMENT 1
--------------------------------------------------------------------------------------------------------------
###UPI: 	jecc724
###Name: 	Jilada Eccleston
###ID: 	336439537
--------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------
##To run the code:
1. Navigate to the workspace folder
2. Run "catkin_make" to compile all the files
3. Run "source devel/setup.bash" to set up the environment
4. Run "roslaunch src/robot_driver/launch/launchSimulationSLAM.launch" to launch the simulation file
5. On the RVIZ viewer - add a map with "/map"
--------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------
##Algorithm:
The robot will first turn a random number of times and then begin traversing forward until it cannot go any
further. It will decide in a direction to turn depending on how close an object or wall might be on the left 
or right of the object. It will turn until there is a clear way to go forward and then begin moving forward.
This cycle is repeated.

While the robot is moving the algorithm works by calculating the different between adjacent laser scan 
measurements. These are then given identifier that holds the current state of the surface:
1. increasing
2. decreasing
3. gap

As all the objects appear as convex shapes and the walls all appear concave, a pattern can be recognised in 
the states. This pattern can be used to determine if an object has been located.

If an object has been located, a GridObject will be created and it will begin calculating the shape type, 
dimensions and then the location of the object. In order to convert from the robot laser coordinates, the 
robot odom rotation matrix and the current x and y location of the robot are used.

The output of the GridObject is then converted into a Shape object. This Shape object is then added to the
ShapeList which contains a vector of shapes. During this adding process, if another object is already in that
surrounding area then unless the shape is different then the algorithm will discard that measurement.

The console will display the output ID, shape type, x and y locations as well as the width and length or radius if the object is a circle.  
--------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------
##Location of Source File:
To find the source files for the project navigate to the directory:
	[../workspace/src/robot_driver/src/](../workspace/src/robot_driver/src/)

##Files:
###globals.h
Contains a list of defines for tolerances and other constants used for calculations
###includes.h
Contains a list of packages included for functions to be available to different classes
###Shape.hpp
C++ header file for the Shapes class
###ShapeList.hpp
C++ header file for the ShapeList class
###GridObject.hpp
C++ header file for the GridObject class

###Shape.cpp
Contains function definitions for accessing the attributes for a stored shape. Contains information regarding the shape type, width, length, radius, x and y location, as well as other functions that will allow for the shape to switch types a keep a running average of the X and Y locations
###ShapeList.cpp
Contains function definition for accessing ShapeList functions. Contains a vector of Shape types that holds the objects detected. An algorithm that will add shapes to the vector and determine if the shape is correctly found. This class is also incharge of printing out the shapes.
###GridObject.cpp
Contains function definition for the gridObject class. This class is used to calculate the x and y locations, width, length and radius of the shape as well as the type of shape the object is.
###pioneerLaser.cpp
Main execution node that will subscribe to the laser scanner, odometry and map nodes to determine the movement of the robot as well as the objects observed in the map. Publishes the velocity commands to the robot to simulate movement.
--------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------
