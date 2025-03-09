#!/usr/bin/env python

# python API for ROS files
import rospy

# allows arguments to be passed into the command line
import argparse

#geometry_msgs.msg.Twist is a message providing velocity commands (linear & angular)
from geometry_msgs.msg import Twist

# sensor_msgs.msg.LaserScan is a message represending 720 laser reading from Robot
from sensor_msgs.msg import LaserScan

# the position can be extracted from msg.pose.pose.position
# this function takes the x and y
def callback(msg):
    min_laser_reading = min(msg.ranges)
    print('The min distance is: %s' % (min_laser_reading))

# algorithm run 
def main(x_vel, z_vel):
    sub = rospy.Subscriber('/laser/scan', LaserScan, callback) # using LaserScan, callback is run whenever data is received
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) # using Twist, buffer of 10 messages between
    rospy.init_node('exercise_node', anonymous=True) # the node for this exercise, anonymous = True ensures unique names given
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Twist() # define message to have linear and angular velocities
        msg.linear.x = x_vel # move forward [default 0.3 m/s]
        msg.angular.z = z_vel # turn counterclockwise on z-axis(left) [default: 0.3 rad/s]
        pub.publish(msg) # send over /cmd_vel
        rate.sleep() # wait to maintain 10 Hz

# code below ensures that CTRL + C closes main() completely
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Set robot velocity")
    parser.add_argument('--x_vel', type=float, default=0.3, help="Linear velocity in x direction")
    parser.add_argument('--z_vel', type=float, default=0.0, help="Angular velocity in z direction")
    args = parser.parse_args()

    try:
        main(args.x_vel, args.z_vel)
    except rospy.ROSInterruptException:
        pass