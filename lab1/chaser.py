#! /usr/bin/env python
# ntt_uz

from math import atan2, sqrt
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn

class Chaser:

    def __init__(self) -> None:
        rospy.init_node('chaser_controller')
        rospy.wait_for_service('/spawn')
        spawn_func = rospy.ServiceProxy('/spawn', Spawn)
        spawn_func(4.0, 4.0, 0.0, 'chaser')
        self.current_chaser_pose = Pose(4.0, 4.0, 0.0, None, None)
        self.current_victim_pose = Pose(5.5, 5.5, 0.0, None, None)

        self.speed = float(rospy.get_param('chaser_speed'))

        self.velocity_publisher = rospy.Publisher('/chaser/cmd_vel', Twist, queue_size=10)
        self.chaser_pose_subscriber = rospy.Subscriber('/chaser/pose', Pose, self.update_pose)
        self.victim_pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_victim_pose)

        self.rate = rospy.Rate(10)
        self.distance_tolerance = 0.5

    def start_chase(self):
        while (not rospy.is_shutdown()):
            self.chase()
            # Publish at the desired rate.
            self.rate.sleep()



    def update_pose(self, data):
        self.current_chaser_pose = data
        self.current_chaser_pose.x = round(self.current_chaser_pose.x, 4)
        self.current_chaser_pose.y = round(self.current_chaser_pose.y, 4)

    def update_victim_pose(self, data):
        self.current_victim_pose = data
        self.current_victim_pose.x = round(self.current_victim_pose.x, 4)
        self.current_victim_pose.y = round(self.current_victim_pose.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose.x - self.current_chaser_pose.x), 2) +
                    pow((goal_pose.y - self.current_chaser_pose.y), 2))
   
    def linear_vel(self, goal_pose):
        return self.speed * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.current_chaser_pose.y, goal_pose.x - self.current_chaser_pose.x)

    def angular_vel(self, goal_pose, constant=5):
        return constant * (self.steering_angle(goal_pose) - self.current_chaser_pose.theta)

    def chase(self):
        """Moves the turtle to the goal."""
        goal_pose = self.current_victim_pose

        vel_msg = Twist()

        if self.euclidean_distance(goal_pose) >= self.distance_tolerance:
            rospy.loginfo("DISTANCE " + str(self.euclidean_distance(goal_pose)))
            rospy.loginfo("CHASER POSE " + str(self.current_chaser_pose))
            rospy.loginfo("VICTIM POSE " + str(self.current_victim_pose))

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_pose)

            # Angular velocity in the z-axis.
            vel_msg.angular.z = self.angular_vel(goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)

        else:
            # Stopping our robot after the movement is over.
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)


if __name__ == '__main__':
    chaser = Chaser()
    chaser.start_chase()
