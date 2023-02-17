#!/usr/bin/env python3

import rospy
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import Pose, PoseStamped
import time

def main():
    rospy.init_node('planning_scene_test')

    scene = PlanningSceneInterface()
    p = PoseStamped()
    p.header.frame_id = "coke_can"
    p.pose.position.x = 0
    p.pose.position.y = 0
    p.pose.position.z = -0.0575
    p.pose.orientation.x = 0
    p.pose.orientation.y = 0
    p.pose.orientation.z = 0
    p.pose.orientation.w = 1
    scene.add_mesh("pippo", p, "coke_can.STL")

    while not rospy.is_shutdown():
        scene.add_mesh("pippo", p, "coke_can.STL")
        time.sleep(0.001)


if __name__ == '__main__':
    main()
