import rospy
import time

from geometry_msgs.msg import Twist
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import AprilTagDetection
from std_msgs.msg import String

class SimpleControl:
    def __init__(self):
        self.cmd_vel = Twist()
        self.test_msg = String()
        self.pub = rospy.Publisher("task_control/cmd_vel", Twist, queue_size=10)
        self.test = rospy.Publisher("task_control/test", String, queue_size=10)
        rospy.Subscriber("tag_detections", AprilTagDetectionArray, callback=self.on_apriltag)
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.pub.publish(self.cmd_vel)
            self.test.publish(self.test_msg)
            r.sleep()

    def on_apriltag(self, tag):
        if(len(tag.detections) > 0):
            if (tag.detections[0].pose.pose.pose.position.x < -0.01):
                self.cmd_vel.linear.x = -1.0
                self.test_msg = "left"
            elif (tag.detections[0].pose.pose.pose.position.x > 0.03):
                self.cmd_vel.linear.x = 1.0
                self.test_msg = "right"
            else:
                self.cmd_vel.linear.x = 0.0
                self.test_msg = "forward"
            self.cmd_vel.angular.x = tag.detections[0].pose.pose.pose.position.x
        else:
            self.test_msg = "no tag"
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.x = 0.0

def main():
    rospy.init_node("simple_control")
    control = SimpleControl()
    rospy.spin()
