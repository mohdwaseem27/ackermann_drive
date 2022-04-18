# ackermann_drive

### To integrate ackermann drive with hardware clone this repo
#### There are 2 files in the pkg one is to calculate PWM for throttle wheels and steering angle from the cmd_vel given by ROS, this node has 1 subscriber (cmd_vel) and 3 publishers (lmotor_pub, rmotor_pub and steering_angle)
#### Publisher of steering angle is in degrees which can directly given to the servo

#### maximum steering angle and maximum rpm of the motor can be changed from the script ackermann.py and the other constants such as wheel circumference and distance between 2 wheels can be changed here

#### The other file in this pkg is odometry.py which calculates the odometry from the wheel rpm, this node contains 1 publiser (odom) and 3 subscribers (lwheel, rwheel, cmd_vel) 
#### lwheel and rwheel subcribers are rpm of left wheel and right wheel
#### All the cinstants can be changed from thhe script odometry.py
