#!/usr/bin/env python3

import rospy
import rospkg
import sys
rospack = rospkg.RosPack()
path = rospack.get_path('pybullet_simulation')
path = path + '/src'
sys.path.append(path)
import pybullet_sim_class

if __name__ == '__main__':
    use_moveit = sys.argv[1]
    use_guy = sys.argv[2]
    word_number = sys.argv[3]

    robots = rospy.get_param('/pybullet_simulation/robots')

    for i in range(int(word_number)):
        for robot_name in robots.keys():
            rospy.delete_param('/pybullet_simulation/robots/' + robot_name)
            rospy.set_param('/pybullet_simulation/robots/' + robot_name + str(i), robots[robot_name])
        pybullet_class = pybullet_sim_class.PybulletSim(node_name=robot_name + str(i),
                                                        use_moveit=use_moveit,
                                                        use_guy=use_guy)
