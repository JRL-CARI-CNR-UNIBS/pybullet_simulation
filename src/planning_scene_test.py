#!/usr/bin/env python3

import rospy
import time
from moveit_msgs.msg import CollisionObject
import meshio
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from shape_msgs.msg import Mesh
from shape_msgs.msg import MeshTriangle
import stl
import pyassimp

def main():
    rospy.init_node('planning_scene_test')

    print('auuuuu')

    scene = pyassimp.load('coke_can.STL')
    print('Noooo')
    if not scene.meshes:
        print('Unable to load mesh')
        return

    print('auuuuu')
    mesh = Mesh()
    for face in scene.meshes[0].faces:
        triangle = MeshTriangle()
        if len(face.indices) == 3:
            triangle.vertex_indices = [face.indices[0],
                                       face.indices[1],
                                       face.indices[2]]
        mesh.triangles.append(triangle)
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]
        point.y = vertex[1]
        point.z = vertex[2]
        mesh.vertices.append(point)
    print('auuuuu')
    pyassimp.release(scene)
    print('auuuuu')

    p = Pose()
    p.position.x = 0
    p.position.y = 0
    p.position.z = 0
    p.orientation.x = 0
    p.orientation.y = 0
    p.orientation.z = 0
    p.orientation.w = 1

    o = CollisionObject()
    o.header.stamp = rospy.Time.now()
    o.header.frame_id = 'table'
    o.id = 'table_'
    o.meshes.append(mesh)
    o.mesh_poses.append(p)
    o.operation = o.ADD

    while not rospy.is_shutdown():
        pub.publish(object)
        time.sleep(1)


if __name__ == '__main__':
    main()
