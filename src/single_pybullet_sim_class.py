#!/usr/bin/env python3

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

    pybullet_class = pybullet_sim_class.PybulletSim(node_name='pybullet_class',
                                                    use_moveit=use_moveit,
                                                    use_guy=use_guy)
