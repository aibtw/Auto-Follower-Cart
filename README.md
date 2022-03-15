# Auto Follower-Cart
_____  

This project is a senior design project carried by Team-06, Electrical and Computer engineering students at KAU, Fall 2021.

## The Destination
_____  

The product being designed is a follower robot, capable of transporting big weights in closed environments like warehouses or even office environment.  
The robot will function in semi-autonomous mode, which requires an operator to be close by, and it will follow the operator (the user)


## Algorithm
_____  
To implement this project, we use ArUco markers, as it provides us the means to do lightweight accurate pose estimation. 
This is necessary to enable the robot to detect and follow the user. 

simply, the camera will receive continues frames, and will send them to the ArUco detector code. Then, pose estimation
and other calculations are done to determine the position and orientation of the user. This information is then interpreted 
as [speed] and [steer] and will be sent to the motors controlling the wheels of the robot. 

Initially, all the code is in one file called TCPLink.py.

## The code
____  
- Initialization
  - Initialize the camera. For our case, we are using the openCV OAK-D cam. initialization done through oakd-init() and 
  set_oakd_props().
  - initialize what's necessary for ArUco detection. This is done through aruco_init().
  - start a TCP connection to communicate with the robot controller.
- Main loop
  - Obtain camera frames, then detect ArUco marker.
  - Perform pose estimation.
  - Find the position of the user in real world using the information of pose estimation.
  - Use this information to control the robot to move forward, backward, left, or right.