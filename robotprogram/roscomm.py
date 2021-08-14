import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math


# This script contains methods for robot movement and
# methods to communicate with ROS installed on the robot

def rospublisherinit():
    global vel_pub, rate
    rospy.init_node('segmentation_node', anonymous=False)  # define the node publishing the topic as talker
    vel_pub = rospy.Publisher('cmd_vel',
                              Twist,
                              queue_size=10)  # create object which define the topic to publish to and the type is twist msg
    rate = rospy.Rate(10)  # 10hz - used to define a rate so that with sleep() method, can publish at a desired rate


def moveforward():
    forward_cmd = Twist()
    forward_cmd.linear.x = 0.3  # speed in linear x direction in m/s
    forward_cmd.linear.y = 0.0  # speed in linear y direction in m/s
    forward_cmd.linear.z = 0.0  # speed in linear z direction in m/s
    forward_cmd.angular.x = 0.0  # speed in angular x direction in m/s
    forward_cmd.angular.y = 0.0  # speed in angular y direction in m/s
    forward_cmd.angular.z = 0.0  # speed in angular z direction in m/s
    # since the current robot can only move forward or turn, only linear x and angular z is used

    # publish
    rospy.loginfo(forward_cmd)  # log and print the cmd_vel
    vel_pub.publish(forward_cmd)  # publish to cmd_vel topic



def turnleft():
    turn_cmd = Twist()
    turn_cmd.linear.x = 0.0  # speed in linear x direction in m/s
    turn_cmd.linear.y = 0.0  # speed in linear y direction in m/s
    turn_cmd.linear.z = 0.0  # speed in linear z direction in m/s
    turn_cmd.angular.x = 0.0  # speed in angular x direction in m/s
    turn_cmd.angular.y = 0.0  # speed in angular y direction in m/s
    turn_cmd.angular.z = 0.3  # speed in angular z direction in m/s
    # since the current robot can only move forward or turn, only linear x and angular z is used

    # publish
    rospy.loginfo(turn_cmd)  # log and print the cmd_vel
    vel_pub.publish(turn_cmd)  # publish to cmd_vel topic


def turnright():
    turn_cmd = Twist()
    turn_cmd.linear.x = 0.0  # speed in linear x direction in m/s
    turn_cmd.linear.y = 0.0  # speed in linear y direction in m/s
    turn_cmd.linear.z = 0.0  # speed in linear z direction in m/s
    turn_cmd.angular.x = 0.0  # speed in angular x direction in m/s
    turn_cmd.angular.y = 0.0  # speed in angular y direction in m/s
    turn_cmd.angular.z = -0.3  # speed in angular z direction in m/s
    # since the current robot can only move forward or turn, only linear x and angular z is used

    # publish
    rospy.loginfo(turn_cmd)  # log and print the cmd_vel
    vel_pub.publish(turn_cmd)  # publish to cmd_vel topic


def stop():
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0  # speed in linear x direction in m/s
    stop_cmd.linear.y = 0.0  # speed in linear y direction in m/s
    stop_cmd.linear.z = 0.0  # speed in linear z direction in m/s
    stop_cmd.angular.x = 0.0  # speed in angular x direction in m/s
    stop_cmd.angular.y = 0.0  # speed in angular y direction in m/s
    stop_cmd.angular.z = 0.0  # speed in angular z direction in m/s
    # since the current robot can only move forward or turn, only linear x and angular z is used

    # publish
    rospy.loginfo(stop_cmd)  # log and print the cmd_vel
    vel_pub.publish(stop_cmd)  # publish to cmd_vel topic

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z
