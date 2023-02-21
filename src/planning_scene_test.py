#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose, Point
import time

import pyassimp
from shape_msgs.msg import MeshTriangle, Mesh
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
def main():
    rospy.init_node('planning_scene_test')

    scene = pyassimp.load('coke_can.STL')

    mesh = Mesh()
    for face in scene.meshes[0].faces:
        triangle = MeshTriangle()
        if len(face) == 3:
            triangle.vertex_indices = [face[0],
                                       face[1],
                                       face[2]]
        mesh.triangles.append(triangle)
    for vertex in scene.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]
        point.y = vertex[1]
        point.z = vertex[2]
        mesh.vertices.append(point)
    pyassimp.release(scene)

    mesh_pose = Pose()
    mesh_pose.position.x = 0
    mesh_pose.position.y = 0
    mesh_pose.position.z = -0.0575
    mesh_pose.orientation.x = 0
    mesh_pose.orientation.y = 0
    mesh_pose.orientation.z = 0
    mesh_pose.orientation.w = 1

    o = CollisionObject()
    o.header.stamp = rospy.Time.now()
    o.header.frame_id = 'coke_can'
    o.id = 'coke_can_'
    o.meshes.append(mesh)
    o.mesh_poses.append(mesh_pose)
    o.operation = o.ADD

    scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=10)
    apply_service = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
    service = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
    req = PlanningSceneComponents()
    req.components = sum([
    PlanningSceneComponents.WORLD_OBJECT_NAMES,
    PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
    PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS])
    aps = service(req)

    ps = aps.scene
    ps.is_diff = True
    ps.robot_state.is_diff = True
    ps.world.collision_objects.append(o)
    apply_service.call(ps)


#    while not rospy.is_shutdown():
#        apply_service.call(ps)
#        time.sleep(0.001)




















#    psi = PlanningSceneInterface()


#    pub = rospy.Publisher('collision_object', CollisionObject, queue_size=10)

#    no_obj = True
#    i = 0
#    while(no_obj and (not rospy.is_shutdown())):
#        pub.publish(o)
#        i += 1
#        print('pub ' + str(i))
#        names = psi.get_known_object_names()
#        try: names.index("coke_can")
#        except ValueError: continue
#        no_obj = False

#    o.operation = o.MOVE

#    while not rospy.is_shutdown():
##        o.pose.position.x += 0.00001
#        pub.publish(o)
#        time.sleep(0.001)


if __name__ == '__main__':
    main()
