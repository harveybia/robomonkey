import rospy
import time
import math

from geometry_msgs.msg import Twist, Pose
from apriltags2_ros.msg import AprilTagDetectionArray, AprilTagDetection
from std_msgs.msg import String
from simple_pid import PID
from tf.transformations import euler_from_quaternion

class SimpleControl:
    def __init__(self):
        self.cmd_vel = Twist()
        self.debug_msg = String()
        self.pose = Pose()
        cmd_pub = rospy.Publisher("task_control/cmd_vel", Twist, queue_size=10)
        debug_pub = rospy.Publisher("task_control/debug", String, queue_size=10)
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, callback=self.on_apriltag)
        r = rospy.Rate(4)
        while not rospy.is_shutdown():
            cmd_pub.publish(self.cmd_vel)
            debug_pub.publish(self.debug_msg)
            r.sleep()

    def on_apriltag(self, tag):
        if(len(tag.detections) > 0):
            self.pose = tag.detections[0].pose.pose.pose
            # self.debug_msg = "found tag"
            self.pid_control()
        else:
            self.debug_msg = "no tag"

    def pid_control(self):
        dist = math.hypot(self.pose.position.x, self.pose.position.y)
        rot = self.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
        if roll < 0:
            theta = -math.pi - roll
        else:
            theta = math.pi - roll
        self.debug_msg = "R: %.2f, P: %.2f, Y: %.2f, T: %.2f" % (roll, pitch, yaw, theta)
        # roll: left/right rotation about camera (center is 3.14/-3.14)
        # pitch: up/down rotation about camera
        # yaw: camera rotation

        lin_pid = PID(1000, 0, 0)
        ang_pid = PID(1, 0, 0)
	lin_pid.setpoint = 2
        lin_vel = lin_pid(dist)
        ang_vel = ang_pid(theta)
	
	# velocity: current value
        self.cmd_vel.linear.x = - lin_vel
        self.cmd_vel.linear.y = 0
	self.cmd_vel.angular.x = dist
	self.cmd_vel.angular.y = theta
        self.cmd_vel.angular.z = ang_vel

    def naive_control(self):
        if (self.pose.position.x < -0.01):
            self.cmd_vel.linear.x = -1.0
            self.cmd_vel.linear.y = 1.0
            self.debug_msg = "left"
        elif (self.pose.position.x > 0.03):
            self.cmd_vel.linear.x = 1.0
            self.cmd_vel.linear.y = -1.0
            self.debug_msg = "right"
        else:
            self.cmd_vel.linear.x = 1.0
            self.cmd_vel.linear.y = 1.0
            self.debug_msg = "forward"
        self.cmd_vel.angular.x = self.pose.position.x

def main():
    rospy.init_node("simple_control")
    control = SimpleControl()
    rospy.spin()
