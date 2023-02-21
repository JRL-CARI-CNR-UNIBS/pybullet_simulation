#!/usr/bin/env python3

import sys
import os
import pybullet as p
import pybullet_data
import rospy
import rospkg
import tf
import time
import copy
import pyassimp
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents, AttachedCollisionObject
from geometry_msgs.msg import WrenchStamped, PoseStamped, Pose, Point
from shape_msgs.msg import MeshTriangle, Mesh
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from threading import Thread
from threading import Lock
from rosgraph_msgs.msg import Clock
from pybullet_utils.srv import SpawnModel
from pybullet_utils.srv import DeleteModel
from pybullet_utils.srv import ChangeControlMode
from pybullet_utils.srv import SaveState
from pybullet_utils.srv import RestoreState
from pybullet_utils.srv import DeleteState
from pybullet_utils.srv import SensorReset


def green_p(msg):
    print('\033[92m' + msg + '\033[0m')


def red_p(msg):
    print('\033[91m' + msg + '\033[0m')


def blue_p(msg):
    print('\033[94m' + msg + '\033[0m')


def yellow_p(msg):
    print('\033[93m' + msg + '\033[0m')


def cyan_p(msg):
    print('\033[96m' + msg + '\033[0m')


class JointTargetSubscriber:
    def __init__(self, robot_id, joint_name_to_index, joint_control_mode, controlled_joint_name, robot_name, jt_topic, control_mode_lock):
        self.robot_id              = robot_id
        self.joint_name_to_index   = joint_name_to_index
        self.joint_control_mode    = joint_control_mode
        self.controlled_joint_name = controlled_joint_name
        self.robot_name            = robot_name
        self.jt_topic              = jt_topic
        self.control_mode_lock                  = control_mode_lock
        rospy.Subscriber(self.jt_topic,
                         JointState,
                         self.jointTargetSubscriber)

    def jointTargetSubscriber(self, data):
        self.control_mode_lock.acquire()
        for joint_id in range(len(data.name)):
            if data.name[joint_id] in self.controlled_joint_name[self.robot_name]:
                if (self.joint_control_mode[self.robot_name] == 'position'):
                    p.setJointMotorControl2(self.robot_id[self.robot_name],
                                            self.joint_name_to_index[self.robot_name][data.name[joint_id]],
                                            p.POSITION_CONTROL,
                                            targetPosition=data.position[joint_id])
                elif (self.joint_control_mode[self.robot_name] == 'velocity'):
                    p.setJointMotorControl2(self.robot_id[self.robot_name],
                                            self.joint_name_to_index[self.robot_name][data.name[joint_id]],
                                            p.VELOCITY_CONTROL,
                                            targetVelocity=data.velocity[joint_id])
                elif (self.joint_control_mode[self.robot_name] == 'torque'):
                    p.setJointMotorControl2(bodyIndex=self.robot_id[self.robot_name],
                                            jointIndex=self.joint_name_to_index[self.robot_name][data.name[joint_id]],
                                            controlMode=p.TORQUE_CONTROL,
                                            force=data.effort[joint_id])
        self.control_mode_lock.release()


def change_control_mode(srv, robot_id, joint_name_to_index, controlled_joint_name, joint_control_mode, joint_effort_limits, control_mode_lock):
    control_mode_lock.acquire()
    for robot_name in srv.robot_names:
        if robot_name in robot_id.keys():
            green_p(str(controlled_joint_name[robot_name]))
            for joint_name in controlled_joint_name[robot_name]:
                joint_control_mode[robot_name] = srv.control_mode
                if (joint_control_mode[robot_name] == 'position'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.POSITION_CONTROL,
                                            force=joint_effort_limits[robot_name][joint_name])
                    green_p('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'position' + ', max_force: ' + str(joint_effort_limits[robot_name][joint_name]))
                elif (joint_control_mode[robot_name] == 'velocity'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=joint_effort_limits[robot_name][joint_name])
                    green_p('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'velocity' + ', max_force: ' + str(joint_effort_limits[robot_name][joint_name]))
                elif (joint_control_mode[robot_name] == 'torque'):
                    yellow_p('In torque joint_control')
                    yellow_p('robot_name: ' + robot_name)
                    yellow_p('joint_name: ' + joint_name)
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=0.0)
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.TORQUE_CONTROL,
                                            force=0.0)
                    green_p('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'torque' + ', force: 0')
                else:
                    red_p(srv.control_mode + ' control mode not exists')
                    return 'false'
        else:
            red_p(robot_name + ' robot not exists')
            return 'false'
    control_mode_lock.release()
    return 'true'


def spawn_model(srv, objects, tf_pub_thread, scenes, use_rviz):
    if tf_pub_thread.is_alive() is False:
        tf_pub_thread.start()
    if not srv.model_name:
        red_p('Name list is empty')
        return 'false'
    if not srv.pose:
        red_p('Poses list is empty')
        return 'false'
    if (len(srv.model_name) != len(srv.pose) or len(srv.model_name) != len(srv.object_name)):
        red_p('Object name, model name and pose do not have the same leng')
    for x in range(len(srv.model_name)):
        object_name = srv.object_name[x]
        model_name = srv.model_name[x]
        pose = srv.pose[x]
        if object_name in objects.keys():
            red_p('Already exists an object with this name')
            return 'false'
        green_p('You want to spawn a ' + model_name + ' with the name ' + object_name)
        if rospy.has_param(model_name):
            model_info = rospy.get_param(model_name)

            if 'package_name' in model_info:
                package_name = model_info['package_name']
                rospack = rospkg.RosPack()
                folder_path = rospack.get_path(package_name)
                green_p('  package_path: ' + folder_path)
            else:
                red_p('No param /' + model_name + '/package_name')
                raise SystemExit
            if 'urdf_file_path' in model_info:
                file_type = 'urdf'
                urdf_file_path = model_info['urdf_file_path']
                green_p('  urdf_file_path: ' + urdf_file_path)
                urdf_path = folder_path + '/' + urdf_file_path
                if (urdf_path.find('.urdf') == -1):
                    red_p('  urdf_file_path do not has extension .urdf')
                    raise SystemExit
            elif 'xacro_file_path' in model_info:
                file_type = 'xacro'
                xacro_file_path = model_info['xacro_file_path']
                green_p('  xacro_file_name: ' + xacro_file_path)
                xacro_path = folder_path + '/' + xacro_file_path
                if (xacro_path.find('.xacro') != -1):
                    urdf_path = xacro_path.replace('.xacro', '.urdf')
                else:
                    red_p('  xacro_file_path do not has extension .xacro')
                    raise SystemExit
            else:
                red_p('No param /' + model_name + '/xacro_file_path(or urdf_file_path)')
                raise SystemExit
            if (use_rviz == 'true'):
                if 'mesh_file_path' in model_info:
                    mesh_file_path = model_info['mesh_file_path']
                    green_p('  mesh_file_path: ' + mesh_file_path)
                    mesh_path = folder_path + '/' + mesh_file_path
                else:
                    red_p('No param /' + model_name + '/mesh_file_path')
                    raise SystemExit
                if 'mesh_position_offset' in model_info:
                    mesh_position_offset = model_info['mesh_position_offset']
                    green_p('  mesh_position_offset: ' + str(mesh_position_offset))
                else:
                    red_p('No param /' + model_name + '/mesh_position_offset')
                    raise SystemExit
                if 'mesh_orientation_offset' in model_info:
                    mesh_orientation_offset = model_info['mesh_orientation_offset']
                    green_p('  mesh_orientation_offset: ' + str(mesh_orientation_offset))
                else:
                    red_p('No param /' + model_name + '/mesh_orientation_offset')
                    raise SystemExit
        else:
            red_p('Model param not found')
            return 'false'
        if (file_type == 'xacro'):
            os.system('rosrun xacro xacro ' + xacro_path + ' > ' + urdf_path)

        start_pos = [pose.position.x, pose.position.y, pose.position.z]
        start_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        objects[object_name] = {}
        objects[object_name]['spawned'] = False
        objects[object_name]['object_id']= p.loadURDF(urdf_path, start_pos, start_orientation, 0, 0, flags=p.URDF_USE_INERTIA_FROM_FILE)
        if (use_rviz == 'true'):
            red_p('Spawn in use_rviz')
            mesh_file = pyassimp.load(mesh_path)
            mesh = Mesh()
            for face in mesh_file.meshes[0].faces:
                triangle = MeshTriangle()
                if len(face) == 3:
                    triangle.vertex_indices = [face[0],
                                               face[1],
                                               face[2]]
                mesh.triangles.append(triangle)
            for vertex in mesh_file.meshes[0].vertices:
                point = Point()
                point.x = vertex[0]
                point.y = vertex[1]
                point.z = vertex[2]
                mesh.vertices.append(point)
            pyassimp.release(mesh_file)
            red_p('Mesh generated')

            pose = Pose()
            pose.position.x = 0
            pose.position.y = 0
            pose.position.z = 0
            pose.orientation.x = 0
            pose.orientation.y = 0
            pose.orientation.z = 0
            pose.orientation.w = 1

            mesh_pose = Pose()
            mesh_pose.position.x = mesh_position_offset[0]
            mesh_pose.position.y = mesh_position_offset[1]
            mesh_pose.position.z = mesh_position_offset[2]
            mesh_pose.orientation.x = mesh_orientation_offset[0]
            mesh_pose.orientation.y = mesh_orientation_offset[1]
            mesh_pose.orientation.z = mesh_orientation_offset[2]
            mesh_pose.orientation.w = mesh_orientation_offset[3]
            red_p('Poses set')

            c_obj = CollisionObject()
            c_obj.header.stamp = rospy.Time.now()
            c_obj.header.frame_id = object_name
            c_obj.id = object_name + '_'
            c_obj.pose = pose
            c_obj.meshes.append(mesh)
            c_obj.mesh_poses.append(mesh_pose)
            c_obj.operation = c_obj.ADD

            a_obj = AttachedCollisionObject()
            a_obj.link_name = ''
            a_obj.object = c_obj
            red_p('Collision Objs generated')

            objects[object_name]['object'] = a_obj

            apply_scene_clnt = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
            get_scene_clnt = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
            req = PlanningSceneComponents()
            req.components = sum([
            PlanningSceneComponents.WORLD_OBJECT_NAMES,
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS])
            scenes[0] = get_scene_clnt(req).scene
            scenes[0].is_diff = True
            scenes[0].robot_state.is_diff = True
            scenes[0].world.collision_objects.append(c_obj)
            apply_scene_clnt.call(scenes[0])
            objects[object_name]['spawned'] = True
            objects[object_name]['attached'] = False
            red_p('Scene applied')

    green_p('Model spawned')
    return 'true'


def delete_model(srv, objects, scenes, use_rviz):
    for object_name in srv.object_name:
        if object_name not in objects:
            yellow_p(object_name + ' is not in the scene')
            continue
        id = objects[object_name]['object_id']
        if (use_rviz == 'true'):
            apply_scene_clnt = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
            if objects[object_name]['object'].object in scenes[0].world.collision_objects:
                scenes[0].world.collision_objects.remove(objects[object_name]['object'].object)
            apply_scene_clnt.call(scenes[0])
        del objects[object_name]
        p.removeBody(id)
    return 'true'


def joint_state_publisher(robot_id, js_publishers, joint_states, controlled_joint_name, joint_state_publish_rate, joint_name_to_index, gear_constraint_to_joint, scenes, scenes_lock):
    name = []
    position = []
    velocity = []
    effort = []
    rate = rospy.Rate(joint_state_publish_rate)
    js_msg = JointState()
    while not rospy.is_shutdown():
        for robot_name in js_publishers.keys():
            joint_states[robot_name] = p.getJointStates(robot_id[robot_name], range(p.getNumJoints(robot_id[robot_name])))
            name.clear()
            position.clear()
            velocity.clear()
            effort.clear()
            for joint_name in joint_name_to_index[robot_name].keys():
                if joint_name not in controlled_joint_name[robot_name]:
                    continue
                name.append(joint_name)
                position.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][0])
                velocity.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][1])
                if joint_name in gear_constraint_to_joint.values():
                    constraint_id = list(gear_constraint_to_joint.keys())[list(gear_constraint_to_joint.values()).index(joint_name)]
                    effort.append(p.getConstraintState(constraint_id)[0])
                else:
                    effort.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][3])
            js_msg.header = Header()
            js_msg.header.stamp = rospy.Time.now()
            js_msg.name = name
            js_msg.position = position
            js_msg.velocity = velocity
            js_msg.effort = effort
            js_publishers[robot_name].publish(js_msg)
            scenes_lock.acquire()
            scenes[0].robot_state.joint_state = js_msg
            scenes_lock.release()
        rate.sleep()


def sensor_wrench_publisher(robot_id, sw_publishers, joint_name_to_index, joint_state_publish_rate, sensor_offset, sensor_offset_lock):
    rate = rospy.Rate(joint_state_publish_rate)
    sw_msg = WrenchStamped()
    while not rospy.is_shutdown():
        sensor_offset_lock.acquire()
        for robot_name in sw_publishers.keys():
            joint_ids = []
            for joint_name in sw_publishers[robot_name].keys():
                joint_ids.append(joint_name_to_index[robot_name][joint_name])
            sensor_wrench = p.getJointStates(robot_id[robot_name], joint_ids)
            for joint_name in sw_publishers[robot_name].keys():
                index = list(sw_publishers[robot_name]).index(joint_name)
                sw_msg.header = Header()
                sw_msg.header.stamp = rospy.Time.now()
                sw_msg.wrench.force.x  = sensor_wrench[index][2][0] - sensor_offset[robot_name][joint_name][0]
                sw_msg.wrench.force.y  = sensor_wrench[index][2][1] - sensor_offset[robot_name][joint_name][1]
                sw_msg.wrench.force.z  = sensor_wrench[index][2][2] - sensor_offset[robot_name][joint_name][2]
                sw_msg.wrench.torque.x = sensor_wrench[index][2][3] - sensor_offset[robot_name][joint_name][3]
                sw_msg.wrench.torque.y = sensor_wrench[index][2][4] - sensor_offset[robot_name][joint_name][4]
                sw_msg.wrench.torque.z = sensor_wrench[index][2][5] - sensor_offset[robot_name][joint_name][5]
                sw_publishers[robot_name][joint_name].publish(sw_msg)
        sensor_offset_lock.release()
        rate.sleep()


def sensor_reset(srv, robot_id, sw_publishers, joint_name_to_index, sensor_offset, sensor_offset_lock):
    sensor_offset_lock.acquire()
    robot_name = srv.robot_name
    joint_name = srv.joint_name
    if robot_name not in sw_publishers:
        red_p(robot_name + ' do not have any sensor')
        return 'false'
    if joint_name not in sw_publishers[robot_name]:
        red_p('The joint ' + joint_name + ' do not has a sensor')
        return 'false'
    sensor_wrench = p.getJointStates(robot_id[robot_name], [joint_name_to_index[robot_name][joint_name]])
    sensor_offset[robot_name][joint_name] = [sensor_wrench[0][2][0],
                                             sensor_wrench[0][2][1],
                                             sensor_wrench[0][2][2],
                                             sensor_wrench[0][2][3],
                                             sensor_wrench[0][2][4],
                                             sensor_wrench[0][2][5]]
    sensor_offset_lock.release()
    return 'true'


def tf_publisher(objects, scenes, use_rviz, scenes_lock):
    if rospy.has_param('object_tf_publish_rate'):
        tf_publish_rate = rospy.get_param('object_tf_publish_rate')
        green_p('object_tf_publish_rate: ' + str(tf_publish_rate))
    else:
        tf_publish_rate = 250
        yellow_p('object_tf_publish_rate not set, default value: ' + str(tf_publish_rate))
    if (use_rviz == 'true'):
        apply_scene_clnt = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
#        get_scene_clnt = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
#        req = PlanningSceneComponents()
#        req.components = sum([
#        PlanningSceneComponents.WORLD_OBJECT_NAMES,
#        PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
#        PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS])
#        scene = get_scene_clnt(req).scene
#        scene.is_diff = True
#        scene.robot_state.is_diff = True
    rate = rospy.Rate(tf_publish_rate)
    br = tf.TransformBroadcaster()
    current_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if (current_time != rospy.Time.now().to_sec()):
#            if (use_rviz == 'true'):
#                scene = get_scene_clnt(req).scene
            for object_name in objects:
                if 'object_id' in objects[object_name]:
                    pose = p.getBasePositionAndOrientation(objects[object_name]['object_id'])
                    br.sendTransform((pose[0][0], pose[0][1], pose[0][2]),
                                     (pose[1][0], pose[1][1], pose[1][2], pose[1][3]),
                                     rospy.Time.now(),
                                     object_name,
                                     "world")
                    if ((use_rviz == 'true') and objects[object_name]['spawned']):
                        scenes_lock.acquire()
                        if (rospy.has_param('/' + object_name + '/attached')):
                            if (rospy.has_param('/' + object_name + '/attached_link')):
                                if(rospy.get_param('/' + object_name + '/attached')):
                                    red_p('In attached')
                                    attached_link = rospy.get_param('/' + object_name + '/attached_link')
                                    objects[object_name]['object'].link_name = attached_link
                                    if not objects[object_name]['attached']:
                                        scenes[0].robot_state.attached_collision_objects.append(objects[object_name]['object'])
                                        apply_scene_clnt.call(scenes[0])
                                        objects[object_name]['attached'] = True
                                else:
                                    if objects[object_name]['object'] in scenes[0].robot_state.attached_collision_objects:
                                        scenes[0].robot_state.attached_collision_objects.remove(objects[object_name]['object'])
                                        apply_scene_clnt.call(scenes[0])
                                        objects[object_name]['attached'] = False
                        scenes_lock.release()
            current_time = rospy.Time.now().to_sec()
        rate.sleep()


def save_state(srv, state_id):
    if not srv.state_name:
        red_p('Name is empty')
        return 'false'
    if srv.state_name in state_id.keys():
        red_p(srv.state_name + ', a state whit this name already exists')
        return 'false'
    state_id[srv.state_name] = p.saveState()
    return 'true'


def restore_state(srv, state_id):
    if not srv.state_name:
        red_p('Name is empty')
        return 'false'
    if srv.state_name not in state_id.keys():
        red_p(srv.state_name + ', no state with this name')
        return 'false'
    p.restoreState(state_id[srv.state_name])
    return 'true'


def delete_state(srv, state_id):
    if not srv.state_name:
        red_p('Name list is empty')
        return 'false'
    for state_name in srv.state_name.keys():
        if state_name not in state_id.keys():
            red_p(state_name + ', no state with this name')
        else:
            p.removeState(state_id[srv.state_name])
    return 'true'


def main():
    control_mode_lock = Lock()
    sensor_offset_lock = Lock()
    scenes_lock = Lock()

    use_rviz = sys.argv[1]

    controlled_joint_name = {}
    gear_constraint_to_joint = {}
    robot_id = {}
    objects = {}
    model_id = {}
    model_mesh = {}
    model_mesh_offset = {}
    link_name_to_index = {}
    joint_name_to_index = {}
    joint_state_publish_rate = None
    simulation_step_time = None
    desidered_real_step_time = None
    joint_states = {}
    joint_control_mode = {}
    joint_effort_limits = {}
    jt_pubishers = {}
    state_id = {}
    sw_publishers = {}
    sensor_offset = {}

    rospy.init_node('pybullet_simulation')

    robot_names = []
    green_p('Wait for robot names')
    ready = False
    while not ready:
        if rospy.has_param('/robots'):
            robot_names = rospy.get_param('/robots')
            green_p('Robot:')
            for robot_name in robot_names:
                green_p(' - ' + robot_name)
            ready = True

    js_publishers = {}

    green_p('Topic generated:')
    for robot_name in robot_names:
        js_topic = '/' + robot_name + '/joint_states'
        js_publishers[robot_name] = rospy.Publisher('/' + robot_name + '/joint_states', JointState, queue_size=1)
        green_p(' - ' + js_topic)

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    if rospy.has_param('camera_init'):
        camera_init = rospy.get_param('camera_init')
        all_info = True
        if 'distance' not in camera_init:
            red_p('No param /camera_init/distance')
            all_info = False
        if 'yaw' not in camera_init:
            red_p('No param /camera_init/yaw')
            all_info = False
        if 'pitch' not in camera_init:
            red_p('No param /camera_init/pitch')
            all_info = False
        if 'target_position' not in camera_init:
            red_p('No param /camera_init/target_position')
            all_info = False
        if all_info:
            p.resetDebugVisualizerCamera(cameraDistance      =camera_init['distance'],
                                         cameraYaw           =camera_init['yaw'],
                                         cameraPitch         =camera_init['pitch'],
                                         cameraTargetPosition=camera_init['target_position'])

    if rospy.has_param('/simulation_step_time'):
        simulation_step_time = rospy.get_param('/simulation_step_time')
        green_p('simulation_step_time: ' + str(simulation_step_time))
    else:
        red_p('No param /simulation_step_time')
        raise SystemExit
    if rospy.has_param('/real_step_time'):
        desidered_real_step_time = rospy.get_param('/real_step_time')
        green_p('real_step_time: ' + str(desidered_real_step_time))
    else:
        red_p('No param /real_step_time')
        raise SystemExit
    if rospy.has_param('/joint_state_publish_rate'):
        joint_state_publish_rate = rospy.get_param('/joint_state_publish_rate')
        green_p('joint_state_publish_rate: ' + str(joint_state_publish_rate))
    else:
        red_p('No param /joint_state_publish_rate')
        raise SystemExit

    p.setAdditionalSearchPath('')

    for robot_name in robot_names:
        start_configuration = {}
        green_p('For robot ' + robot_name + ':')
        if rospy.has_param('/' + robot_name):
            robot_info = rospy.get_param('/' + robot_name)
            green_p(' All parameter:')
            green_p('   {')
            for key in robot_info.keys():
                green_p('    ' + key + ': ' + str(robot_info[key]))
            green_p('   }')
            if 'package_name' in robot_info:
                package_name = robot_info['package_name']
                rospack = rospkg.RosPack()
                folder_path = rospack.get_path(package_name)
                green_p('  package_path: ' + folder_path)
            else:
                red_p('No param /' + robot_name + '/package_name')
                raise SystemExit
            if 'urdf_file_path' in robot_info:
                file_type = 'urdf'
                urdf_file_path = robot_info['urdf_file_path']
                green_p('  urdf_file_path: ' + urdf_file_path)
                urdf_path = folder_path + '/' + urdf_file_path
                if (urdf_path.find('.xacro') == -1):
                    red_p('  urdf_file_path do not has extension .urdf')
                    raise SystemExit
            elif 'xacro_file_path' in robot_info:
                file_type = 'xacro'
                xacro_file_path = robot_info['xacro_file_path']
                green_p('  xacro_file_name: ' + xacro_file_path)
                xacro_path = folder_path + '/' + xacro_file_path
                if (xacro_path.find('.xacro') != -1):
                    urdf_path = xacro_path.replace('.xacro', '.urdf')
                else:
                    red_p('  xacro_file_path do not has extension .xacro')
                    raise SystemExit
            else:
                red_p('No param /' + robot_name + '/xacro_file_path(or urdf_file_path)')
                raise SystemExit
            if 'start_position' in robot_info:
                start_pos = robot_info['start_position']
                green_p('  start_pos: [' +
                        str(start_pos[0]) + ',' +
                        str(start_pos[1]) + ',' +
                        str(start_pos[2]) + ']')
            else:
                red_p('No param /' + robot_name + '/start_position')
                raise SystemExit
            if 'start_orientation' in robot_info:
                start_orientation = p.getQuaternionFromEuler(robot_info['start_orientation'])
                green_p('  start_orientation: [' +
                        str(start_orientation[0]) + ',' +
                        str(start_orientation[1]) + ',' +
                        str(start_orientation[2]) + ',' +
                        str(start_orientation[3]) + ']')
            else:
                red_p('No param /' + robot_name + '/start_orientation')
                raise SystemExit
            if 'fixed' in robot_info:
                fixed = robot_info['fixed']
                green_p('  fixed: ' + str(fixed))
            else:
                red_p('No param /' + robot_name + '/fixed')
                raise SystemExit
            if 'controlled_joint_name' in robot_info:
                controlled_joint_name[robot_name] = robot_info['controlled_joint_name']
                array_str = '  controlled_joint_name: ['
                for joint_controlled_name in controlled_joint_name[robot_name]:
                    array_str += joint_controlled_name + ' '
                array_str += ']'
                green_p(array_str)
            else:
                red_p('No param /' + robot_name + '/controller_joint_name')
                raise SystemExit
            if 'start_configuration' in robot_info:
                if (len(controlled_joint_name[robot_name]) == len(robot_info['start_configuration'])):
                    for index in range(len(controlled_joint_name[robot_name])):
                        start_configuration[controlled_joint_name[robot_name][index]] = robot_info['start_configuration'][index]
                else:
                    yellow_p('start_configuration size is wrong')
                    for index in range(len(controlled_joint_name[robot_name])):
                        start_configuration[controlled_joint_name[robot_name][index]] = 0.0
            else:
                yellow_p('No param /' + robot_name + '/start_configuration')
                for index in range(len(controlled_joint_name[robot_name])):
                    start_configuration[controlled_joint_name[robot_name][index]] = 0.0

            if (file_type == 'xacro'):
                os.system("rosrun xacro xacro " + xacro_path + " robot_name:='" + robot_name + "' > " + urdf_path)

            robot_id[robot_name] = p.loadURDF(urdf_path, start_pos, start_orientation, 0, fixed, flags=p.URDF_USE_INERTIA_FROM_FILE)
            green_p('  robot_id: ' + str(robot_id[robot_name]))

            link_name_to_index[robot_name] = {p.getBodyInfo(robot_id[robot_name])[0].decode('UTF-8'): -1, }
            joint_name_to_index[robot_name] = {}
            joint_effort_limits[robot_name] = {}

            green_p('  joint_info: ')
            for joint_id in range(p.getNumJoints(robot_id[robot_name])):
                link_name = p.getJointInfo(robot_id[robot_name], joint_id)[12].decode('UTF-8')
                link_name_to_index[robot_name][link_name] = joint_id
                joint_name = p.getJointInfo(robot_id[robot_name], joint_id)[1].decode('UTF-8')
                joint_name_to_index[robot_name][joint_name] = joint_id
                joint_effort_limits[robot_name][joint_name] = p.getJointInfo(robot_id[robot_name], joint_id)[10]

                green_p('    -jointName: '        + p.getJointInfo(robot_id[robot_name], joint_id)[1].decode('UTF-8'))
                green_p('     jointIndex: '       + str(p.getJointInfo(robot_id[robot_name], joint_id)[0]))
                green_p('     jointType: '        + str(p.getJointInfo(robot_id[robot_name], joint_id)[2]))
                green_p('     qIndex: '           + str(p.getJointInfo(robot_id[robot_name], joint_id)[3]))
                green_p('     uIndex: '           + str(p.getJointInfo(robot_id[robot_name], joint_id)[4]))
                green_p('     flags: '            + str(p.getJointInfo(robot_id[robot_name], joint_id)[5]))
                green_p('     jointDamping: '     + str(p.getJointInfo(robot_id[robot_name], joint_id)[6]))
                green_p('     jointFriction: '    + str(p.getJointInfo(robot_id[robot_name], joint_id)[7]))
                green_p('     jointLowerLimit: '  + str(p.getJointInfo(robot_id[robot_name], joint_id)[8]))
                green_p('     jointUpperLimit: '  + str(p.getJointInfo(robot_id[robot_name], joint_id)[9]))
                green_p('     jointMaxForce: '    + str(p.getJointInfo(robot_id[robot_name], joint_id)[10]))
                green_p('     jointMaxVelocity: ' + str(p.getJointInfo(robot_id[robot_name], joint_id)[11]))
                green_p('     linkName: '         + p.getJointInfo(robot_id[robot_name], joint_id)[12].decode('UTF-8'))
                green_p('     jointAxis: '        + str(p.getJointInfo(robot_id[robot_name], joint_id)[13]))
                green_p('     parentFramePos: '   + str(p.getJointInfo(robot_id[robot_name], joint_id)[14]))
                green_p('     parentFrameOrn: '   + str(p.getJointInfo(robot_id[robot_name], joint_id)[15]))
                green_p('     parentIndex: '      + str(p.getJointInfo(robot_id[robot_name], joint_id)[16]))

            if 'constraints' in robot_info:
                constraints = robot_info['constraints']
                green_p('  constraints: ')
                for constraint in constraints:
                    if 'parent_body' in constraint:
                        parent_body = constraint['parent_body']
                        green_p('    - parent_body: ' + parent_body)
                    else:
                        red_p('No param /' + robot_name + '/constraint/parent_body')
                        raise SystemExit
                    if 'parent_link' in constraint:
                        parent_link = constraint['parent_link']
                        green_p('      parent_link: ' + parent_link)
                    else:
                        print('No param /' + robot_name + '/constraint/parent_link')
                        red_p('No /robots param')
                        raise SystemExit
                    if 'child_body' in constraint:
                        child_body = constraint['child_body']
                        green_p('      child_body: ' + child_body)
                    else:
                        red_p('No param /' + robot_name + '/constraint/child_body')
                        raise SystemExit
                    if 'child_link' in constraint:
                        child_link = constraint['child_link']
                        green_p('      child_link: ' + child_link)
                    else:
                        red_p('No param /' + robot_name + '/constraint/child_link')
                        raise SystemExit
                    if 'type' in constraint:
                        type = constraint['type']
                        green_p('      type: ' + type)
                    else:
                        red_p('No param /' + robot_name + '/constraint/type')
                        raise SystemExit
                    if 'axis' in constraint:
                        axis = constraint['axis']
                        green_p('      axis: [' +
                                str(axis[0]) + ',' +
                                str(axis[1]) + ',' +
                                str(axis[2]) + ']')
                    else:
                        red_p('No param /' + robot_name + '/constraint/axis')
                        raise SystemExit
                    if 'parent_frame_position' in constraint:
                        parent_frame_position = constraint['parent_frame_position']
                        green_p('      parent_frame_position: [' +
                                str(parent_frame_position[0]) + ',' +
                                str(parent_frame_position[1]) + ',' +
                                str(parent_frame_position[2]) + ']')
                    else:
                        red_p('No param /' + robot_name + '/constraint/parent_frame_position')
                        raise SystemExit
                    if 'child_frame_position' in constraint:
                        child_frame_position = constraint['child_frame_position']
                        green_p('      child_frame_position: [' +
                                str(child_frame_position[0]) + ',' +
                                str(child_frame_position[1]) + ',' +
                                str(child_frame_position[2]) + ']')
                    else:
                        red_p('No param /' + robot_name + '/constraint/child_frame_position')
                        raise SystemExit
                    if 'parent_frame_orientation' in constraint:
                        parent_frame_orientation = constraint['parent_frame_orientation']
                        green_p('      parent_frame_orientation: [' +
                                str(parent_frame_orientation[0]) + ',' +
                                str(parent_frame_orientation[1]) + ',' +
                                str(parent_frame_orientation[2]) + ',' +
                                str(parent_frame_orientation[3]) + ']')
                    else:
                        red_p('No param /' + robot_name + '/constraint/parent_frame_orientation')
                        raise SystemExit
                    if 'child_frame_orientation' in constraint:
                        child_frame_orientation = constraint['child_frame_orientation']
                        green_p('      child_frame_orientation: [' +
                                str(child_frame_orientation[0]) + ',' +
                                str(child_frame_orientation[1]) + ',' +
                                str(child_frame_orientation[2]) + ',' +
                                str(child_frame_orientation[3]) + ']')
                    else:
                        red_p('No param /' + robot_name + '/constraint/child_frame_orientation')
                        raise SystemExit
                    if (type == 'prismatic'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[parent_body][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[child_body][child_link],
                                                           p.JOINT_PRISMATIC,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    elif (type == 'fixed'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[parent_body][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[child_body][child_link],
                                                           p.JOINT_FIXED,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    elif (type == 'point2point'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[parent_body][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[child_body][child_link],
                                                           p.JOINT_POINT2POINT,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    elif (type == 'gear'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[parent_body][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[child_body][child_link],
                                                           p.JOINT_GEAR,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    else:
                        red_p('Constraint type not foud')
                        raise SystemExit
                    if (type == 'gear'):
                        gear_constraint_to_joint[constraint_id] = p.getJointInfo(robot_id[child_body], link_name_to_index[child_body][child_link])[1].decode('UTF-8')
                        print(gear_constraint_to_joint)
                        if 'gear_ratio' in constraint:
                            gear_ratio = constraint['gear_ratio']
                            green_p('      gear_ratio: ' + str(gear_ratio))
                            p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                        else:
                            yellow_p('      gear_ratio: not set. Default:1')
                            p.changeConstraint(constraint_id, gearRatio=1)
                        if 'erp' in constraint:
                            erp_var = constraint['erp']
                            green_p('      erp: ' + str(erp_var))
                            p.changeConstraint(constraint_id, erp=erp_var)
                        else:
                            yellow_p('      erp: not set. Default:1')
                            p.changeConstraint(constraint_id, erp=1)
                        if (type == 'gear'):
                            if 'gear_aux_link' in constraint:
                                gear_aux_link = constraint['gear_aux_link']
                                green_p('      gear_aux_link: ' + str(gear_aux_link))
                                p.changeConstraint(constraint_id, gearAuxLink=link_name_to_index[robot_name][gear_aux_link])
                        else:
                            yellow_p('      gear_aux_link: not set.')
                    if 'max_force' in constraint:
                        max_force = constraint['max_force']
                        green_p('      max_force: ' + str(max_force))
                        p.changeConstraint(constraint_id, maxForce=max_force)
                    else:
                        yellow_p('      max_force: not set. Default:100')
                        p.changeConstraint(constraint_id, maxForce=100)
            else:
                yellow_p('No param /' + robot_name + '/constraints')

            if 'sensors' in robot_info:
                sensors = robot_info['sensors']
                if not (isinstance(sensors, list)):
                    yellow_p('Param /' + robot_name + '/sensors is not a list')
                else:
                    sw_publishers[robot_name] = {}
                    sensor_offset[robot_name] = {}
                    for sensor in sensors:
                        p.enableJointForceTorqueSensor(robot_id[robot_name], joint_name_to_index[robot_name][sensor], enableSensor=1)
                        sw_publishers[robot_name][sensor] = rospy.Publisher('/' + robot_name + '/' + sensor + '/wrench', WrenchStamped, queue_size=1)
                        sensor_offset[robot_name][sensor] = [0, 0, 0, 0, 0, 0]
            else:
                yellow_p('No param /' + robot_name + '/sensors')

            if 'link_dynamics' in robot_info:
                links_dyn = robot_info['link_dynamics']
                green_p('  link dynamics: ')
                for link_dyn in links_dyn:
                    if 'link_name' in link_dyn:
                        link_name = link_dyn['link_name']
                        green_p('    - link_name: ' + link_name)
                    else:
                        red_p('No param /' + robot_name + '/link_dynamics/link_name')
                        raise SystemExit
                    current_dyn_info = p.getDynamicsInfo(robot_id[robot_name], link_name_to_index[robot_name][link_name])
                    if 'lateral_friction' in link_dyn:
                        lateral_friction = link_dyn['lateral_friction']
                        green_p('      lateral_friction: ' + str(lateral_friction))
                    else:
                        lateral_friction = current_dyn_info[1]
                        yellow_p('No param /' + robot_name + '/link_dynamics/lateral_friction, current value:' + str(lateral_friction))
                    if 'spinning_friction' in link_dyn:
                        spinning_friction = link_dyn['spinning_friction']
                        green_p('      spinning_friction: ' + str(spinning_friction))
                    else:
                        spinning_friction = current_dyn_info[7]
                        yellow_p('No param /' + robot_name + '/link_dynamics/spinning_friction, current value:' + str(spinning_friction))
                    if 'rolling_friction' in link_dyn:
                        rolling_friction = link_dyn['rolling_friction']
                        green_p('      rolling_friction: ' + str(rolling_friction))
                    else:
                        rolling_friction = current_dyn_info[6]
                        yellow_p('No param /' + robot_name + '/link_dynamics/rolling_friction, current value:' + str(rolling_friction))
                    if 'contact_stiffness' in link_dyn:
                        contact_stiffness = link_dyn['contact_stiffness']
                        green_p('      contact_stiffness: ' + str(contact_stiffness))
                    else:
                        contact_stiffness = current_dyn_info[9]
                        yellow_p('No param /' + robot_name + '/link_dynamics/contact_stiffness, current value:' + str(contact_stiffness))
                    if 'contact_damping' in link_dyn:
                        contact_damping = link_dyn['contact_damping']
                        green_p('      contact_damping: ' + str(contact_damping))
                    else:
                        contact_damping = current_dyn_info[8]
                        yellow_p('No param /' + robot_name + '/link_dynamics/contact_damping, current value:' + str(contact_damping))
                    if 'linear_damping' in link_dyn:
                        linear_damping = link_dyn['linear_damping']
                        green_p('      linear_damping: ' + str(linear_damping))
                    else:
                        linear_damping = 0.4
                        yellow_p('No param /' + robot_name + '/link_dynamics/linear_damping, current value:' + str(linear_damping))
                    if 'angular_damping' in link_dyn:
                        angular_damping = link_dyn['angular_damping']
                        green_p('      angular_damping: ' + str(angular_damping))
                    else:
                        angular_damping = 0.4
                        yellow_p('No param /' + robot_name + '/link_dynamics/angular_damping' + str(angular_damping))
                        raise SystemExit
                    if 'friction_anchor' in link_dyn:
                        friction_anchor = link_dyn['friction_anchor']
                        green_p('      friction_anchor: ' + str(friction_anchor))
                    else:
                        friction_anchor = 0
                        yellow_p('No param /' + robot_name + '/link_dynamics/friction_anchor' + str(friction_anchor))
                    p.changeDynamics(robot_id[robot_name], link_name_to_index[robot_name][link_name],
                                     lateralFriction=lateral_friction,
                                     spinningFriction=spinning_friction,
                                     rollingFriction=rolling_friction,
                                     contactStiffness=contact_stiffness,
                                     contactDamping=contact_damping,
                                     linearDamping=linear_damping,
                                     angularDamping=angular_damping,
                                     frictionAnchor=friction_anchor)

            if 'joint_control_mode' in robot_info:
                joint_control_mode[robot_name] = robot_info['joint_control_mode']
                green_p('  joint_control_mode: ' + joint_control_mode[robot_name])
            else:
                red_p('No param /' + robot_name + '/joint_control_mode')
                raise SystemExit

            jt_topic = '/' + robot_name + '/joint_target'

            jt_pubishers[robot_name] = JointTargetSubscriber(robot_id,
                                                             joint_name_to_index,
                                                             joint_control_mode,
                                                             controlled_joint_name,
                                                             robot_name,
                                                             jt_topic,
                                                             control_mode_lock)

            joint_states[robot_name] = p.getJointStates(robot_id[robot_name], range(p.getNumJoints(robot_id[robot_name])))
            for joint_name in joint_name_to_index[robot_name].keys():
                if joint_name in controlled_joint_name[robot_name]:
                    if (joint_control_mode[robot_name] == 'position'):
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.POSITION_CONTROL,
                                                targetPosition=start_configuration[joint_name])
                        cyan_p(joint_name + ' joint set with position control, target position = 0.0')
                    elif (joint_control_mode[robot_name] == 'velocity'):
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.VELOCITY_CONTROL,
                                                targetVelocity=0.0)
                        cyan_p(joint_name + ' joint set with velocity control, target velocity = 0.0')
                    elif (joint_control_mode[robot_name] == 'torque'):
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.VELOCITY_CONTROL,
                                                force=0.0)
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.TORQUE_CONTROL,
                                                force=0.0)
                        cyan_p(joint_name + ' joint set with torque control, target force = 0.0')
                    else:
                        red_p('/' + robot_name + '/joint_control_mode not in existing types')
                        raise SystemExit
                else:
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=0.0)
                    green_p(joint_name + ' joint set with velocity control, max force = 0.0')
        else:
            red_p('/' + robot_name + ' param not found')
            raise SystemExit

    if rospy.has_param('external_constraint'):
        green_p('External constraints: ')
        constraints = rospy.get_param('external_constraint')
        if isinstance(constraints, list):
            for constraint in constraints:
                if not isinstance(constraint, dict):
                    red_p('Single constraint is not a dictionary')
                    raise SystemExit
                if 'parent_body' in constraint:
                    parent_body = constraint['parent_body']
                    green_p('    - parent_body: ' + parent_body)
                else:
                    red_p('No param /' + robot_name + '/constraint/parent_body')
                    raise SystemExit
                if 'parent_link' in constraint:
                    parent_link = constraint['parent_link']
                    green_p('      parent_link: ' + parent_link)
                else:
                    print('No param /' + robot_name + '/constraint/parent_link')
                    red_p('No /robots param')
                    raise SystemExit
                if 'child_body' in constraint:
                    child_body = constraint['child_body']
                    green_p('      child_body: ' + child_body)
                else:
                    red_p('No param /' + robot_name + '/constraint/child_body')
                    raise SystemExit
                if 'child_link' in constraint:
                    child_link = constraint['child_link']
                    green_p('      child_link: ' + child_link)
                else:
                    red_p('No param /' + robot_name + '/constraint/child_link')
                    raise SystemExit
                if 'type' in constraint:
                    type = constraint['type']
                    green_p('      type: ' + type)
                else:
                    red_p('No param /' + robot_name + '/constraint/type')
                    raise SystemExit
                if 'axis' in constraint:
                    axis = constraint['axis']
                    green_p('      axis: [' +
                            str(axis[0]) + ',' +
                            str(axis[1]) + ',' +
                            str(axis[2]) + ']')
                else:
                    red_p('No param /' + robot_name + '/constraint/axis')
                    raise SystemExit
                if 'parent_frame_position' in constraint:
                    parent_frame_position = constraint['parent_frame_position']
                    green_p('      parent_frame_position: [' +
                            str(parent_frame_position[0]) + ',' +
                            str(parent_frame_position[1]) + ',' +
                            str(parent_frame_position[2]) + ']')
                else:
                    red_p('No param /' + robot_name + '/constraint/parent_frame_position')
                    raise SystemExit
                if 'child_frame_position' in constraint:
                    child_frame_position = constraint['child_frame_position']
                    green_p('      child_frame_position: [' +
                            str(child_frame_position[0]) + ',' +
                            str(child_frame_position[1]) + ',' +
                            str(child_frame_position[2]) + ']')
                else:
                    red_p('No param /' + robot_name + '/constraint/child_frame_position')
                    raise SystemExit
                if 'parent_frame_orientation' in constraint:
                    parent_frame_orientation = constraint['parent_frame_orientation']
                    green_p('      parent_frame_orientation: [' +
                            str(parent_frame_orientation[0]) + ',' +
                            str(parent_frame_orientation[1]) + ',' +
                            str(parent_frame_orientation[2]) + ',' +
                            str(parent_frame_orientation[3]) + ']')
                else:
                    red_p('No param /' + robot_name + '/constraint/parent_frame_orientation')
                    raise SystemExit
                if 'child_frame_orientation' in constraint:
                    child_frame_orientation = constraint['child_frame_orientation']
                    green_p('      child_frame_orientation: [' +
                            str(child_frame_orientation[0]) + ',' +
                            str(child_frame_orientation[1]) + ',' +
                            str(child_frame_orientation[2]) + ',' +
                            str(child_frame_orientation[3]) + ']')
                else:
                    red_p('No param /' + robot_name + '/constraint/child_frame_orientation')
                    raise SystemExit
                if (type == 'prismatic'):
                    constraint_id = p.createConstraint(robot_id[parent_body],
                                                       link_name_to_index[parent_body][parent_link],
                                                       robot_id[child_body],
                                                       link_name_to_index[child_body][child_link],
                                                       p.JOINT_PRISMATIC,
                                                       axis,
                                                       parent_frame_position,
                                                       child_frame_position,
                                                       parent_frame_orientation,
                                                       child_frame_orientation)
                elif (type == 'fixed'):
                    constraint_id = p.createConstraint(robot_id[parent_body],
                                                       link_name_to_index[parent_body][parent_link],
                                                       robot_id[child_body],
                                                       link_name_to_index[child_body][child_link],
                                                       p.JOINT_FIXED,
                                                       axis,
                                                       parent_frame_position,
                                                       child_frame_position,
                                                       parent_frame_orientation,
                                                       child_frame_orientation)
                elif (type == 'point2point'):
                    constraint_id = p.createConstraint(robot_id[parent_body],
                                                       link_name_to_index[parent_body][parent_link],
                                                       robot_id[child_body],
                                                       link_name_to_index[child_body][child_link],
                                                       p.JOINT_POINT2POINT,
                                                       axis,
                                                       parent_frame_position,
                                                       child_frame_position,
                                                       parent_frame_orientation,
                                                       child_frame_orientation)
                elif (type == 'gear'):
                    constraint_id = p.createConstraint(robot_id[parent_body],
                                                       link_name_to_index[parent_body][parent_link],
                                                       robot_id[child_body],
                                                       link_name_to_index[child_body][child_link],
                                                       p.JOINT_GEAR,
                                                       axis,
                                                       parent_frame_position,
                                                       child_frame_position,
                                                       parent_frame_orientation,
                                                       child_frame_orientation)
                else:
                    red_p('Constraint type not foud')
                    raise SystemExit
                if (type == 'gear'):
                    gear_constraint_to_joint[constraint_id] = p.getJointInfo(robot_id[child_body], link_name_to_index[child_body][child_link])[1].decode('UTF-8')
                    print(gear_constraint_to_joint)
                    if 'gear_ratio' in constraint:
                        gear_ratio = constraint['gear_ratio']
                        green_p('      gear_ratio: ' + str(gear_ratio))
                        p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                    else:
                        yellow_p('      gear_ratio: not set. Default:1')
                        p.changeConstraint(constraint_id, gearRatio=1)
                    if 'erp' in constraint:
                        erp_var = constraint['erp']
                        green_p('      erp: ' + str(erp_var))
                        p.changeConstraint(constraint_id, erp=erp_var)
                    else:
                        yellow_p('      erp: not set. Default:1')
                        p.changeConstraint(constraint_id, erp=1)
                    if (type == 'gear'):
                        if 'gear_aux_link' in constraint:
                            gear_aux_link = constraint['gear_aux_link']
                            green_p('      gear_aux_link: ' + str(gear_aux_link))
                            p.changeConstraint(constraint_id, gearAuxLink=link_name_to_index[robot_name][gear_aux_link])
                    else:
                        yellow_p('      gear_aux_link: not set.')
                if 'max_force' in constraint:
                    max_force = constraint['max_force']
                    green_p('      max_force: ' + str(max_force))
                    p.changeConstraint(constraint_id, maxForce=max_force)
                else:
                    yellow_p('      max_force: not set. Default:100')
                    p.changeConstraint(constraint_id, maxForce=100)
        else:
            red_p('external_constraint is not a list')
    else:
        yellow_p('No external constraint')

    rospy.Service('change_control_mode',
                  ChangeControlMode,
                  lambda msg:
                      change_control_mode(msg,
                                          robot_id,
                                          joint_name_to_index,
                                          controlled_joint_name,
                                          joint_control_mode,
                                          joint_effort_limits,
                                          control_mode_lock))

    rospy.Service('pybullet_save_state',
                  SaveState,
                  lambda msg: save_state(msg, state_id))

    rospy.Service('pybullet_restore_state',
                  RestoreState,
                  lambda msg: restore_state(msg, state_id))

    rospy.Service('pybullet_delete_state',
                  DeleteState,
                  lambda msg: delete_state(msg, state_id))
    rospy.Service('pybullet_sensor_reset',
                  SensorReset,
                  lambda msg: sensor_reset(msg,
                                           robot_id,
                                           sw_publishers,
                                           joint_name_to_index,
                                           sensor_offset,
                                           sensor_offset_lock))

    scenes = []
    scene = PlanningScene()
    scenes.append(scene)

    tf_pub_thread = Thread(target=tf_publisher, args=(objects, scenes, use_rviz, scenes_lock))

    rospy.Service('pybullet_spawn_model',
                  SpawnModel,
                  lambda msg:
                      spawn_model(msg,
                                  objects,
                                  tf_pub_thread,
                                  scenes,
                                  use_rviz))
    rospy.Service('pybullet_delete_model',
                  DeleteModel,
                  lambda msg:
                      delete_model(msg,
                                   objects,
                                   scenes,
                                   use_rviz))
    time_pub = rospy.Publisher('/clock', Clock, queue_size=10)

    p.setTimeStep(simulation_step_time)
    p.stepSimulation()
    simulation_time = []
    simulation_time.append(simulation_step_time)
    simulation_time_msg = rospy.Time(simulation_time[0])
    time_pub.publish(simulation_time_msg)

    js_pub_thread = Thread(target=joint_state_publisher, args=(robot_id,
                                                               js_publishers,
                                                               joint_states,
                                                               controlled_joint_name,
                                                               joint_state_publish_rate,
                                                               joint_name_to_index,
                                                               gear_constraint_to_joint,
                                                               scenes,
                                                               scenes_lock))
    js_pub_thread.start()

    sw_pub_thread = Thread(target=sensor_wrench_publisher, args=(robot_id,
                                                                 sw_publishers,
                                                                 joint_name_to_index,
                                                                 joint_state_publish_rate,
                                                                 sensor_offset,
                                                                 sensor_offset_lock))

    sw_pub_thread.start()

    time.sleep(0.1)

    cyan_p('Ready for simulation')

    now = time.time()
    while not rospy.is_shutdown():
        p.stepSimulation()
        simulation_time[0] += simulation_step_time
        simulation_time_msg = rospy.Time(simulation_time[0])
        time_pub.publish(simulation_time_msg)
        elapsed = time.time() - now
#        red_p(str(elapsed))
#        green_p(str(1 / elapsed))
        if (desidered_real_step_time - elapsed > 0):
            time.sleep(desidered_real_step_time - elapsed)
#        print('Time ratio: ' + str(simulation_step_time / (time.time() - now)))
        now = time.time()

    p.disconnect()

    green_p('Waiting for threads...')
    js_pub_thread.join()
    tf_pub_thread.join()
    sw_pub_thread.join()


if __name__ == '__main__':
    main()
