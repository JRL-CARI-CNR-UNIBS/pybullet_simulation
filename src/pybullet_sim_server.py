#!/usr/bin/env python3

import sys
import os
#import pybullet as p
import pybullet
import pybullet_utils.bullet_client as bc
import pybullet_data
import rospy
import rospkg
import tf
import time
import pyassimp
import numpy
import copy
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import CollisionObject, PlanningScene, PlanningSceneComponents, AttachedCollisionObject
from geometry_msgs.msg import WrenchStamped, Pose, Point
from shape_msgs.msg import MeshTriangle, Mesh
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from threading import Thread
from threading import Lock
from rosgraph_msgs.msg import Clock
from pybullet_simulation.srv import SpawnModel
from pybullet_simulation.srv import DeleteModel
from pybullet_simulation.srv import ChangeControlMode
from pybullet_simulation.srv import SaveState
from pybullet_simulation.srv import RestoreState
from pybullet_simulation.srv import DeleteState
from pybullet_simulation.srv import SensorReset



class JointTargetSubscriber:
    def __init__(self, p, joint_target, joint_target_lock, robot_name, jt_topic):
        self.robot_name            = robot_name
        self.jt_topic              = jt_topic
        self.joint_target         = joint_target
        self.joint_target_lock    = joint_target_lock

        rospy.Subscriber(self.jt_topic,
                         JointState,
                         self.jointTargetSubscriber)

    def jointTargetSubscriber(self, data):
        self.joint_target_lock.acquire()

        for joint_id in range(len(data.name)):
            if data.name[joint_id] in self.joint_target[self.robot_name].keys():
                self.joint_target[self.robot_name][data.name[joint_id]]['position'] = data.position[joint_id]
                self.joint_target[self.robot_name][data.name[joint_id]]['velocity'] = data.velocity[joint_id]
                self.joint_target[self.robot_name][data.name[joint_id]]['effort'] = data.effort[joint_id]
        self.joint_target_lock.release()


def joint_target_integration(p,
                             robot_names,
                             robot_id,
                             joint_control_mode,
                             controlled_joint_name,
                             joint_states,
                             joint_state_lock,
                             joint_target,
                             joint_target_lock,
                             joint_control_integral_gain,
                             joint_name_to_index,
                             control_mode_lock,
                             simulation_step_time):

    pos_compensation = {}
    rate = rospy.Rate(2 / simulation_step_time)

    for robot_name in robot_names:
        pos_compensation[robot_name] = {}
        for joint_name in joint_name_to_index[robot_name].keys():
            pos_compensation[robot_name][joint_name] = 0.0

    while not rospy.is_shutdown():
        control_mode_lock.acquire()
        for robot_name in robot_names:
            target_position_str  = 'target_position:  ['
            joint_state_str      = 'joint state:      ['
            pos_compensation_str = 'pos_compensation: ['
            pos_diff_str         = 'pos_diff:         ['

            for joint_name in joint_name_to_index[robot_name].keys():
                if joint_name in controlled_joint_name[robot_name]:
                    if (joint_control_mode[robot_name] == 'position'):
                        p.setJointMotorControl2(bodyIndex=robot_id[robot_name],
                                                jointIndex=joint_name_to_index[robot_name][joint_name],
                                                controlMode=p.POSITION_CONTROL,
                                                targetPosition=joint_target[robot_name][joint_name]['position'] + pos_compensation[robot_name][joint_name])
                    elif (joint_control_mode[robot_name] == 'velocity'):
                        p.setJointMotorControl2(bodyIndex=robot_id[robot_name],
                                                jointIndex=joint_name_to_index[robot_name][joint_name],
                                                controlMode=p.VELOCITY_CONTROL,
                                                targetVelocity=joint_target[robot_name][joint_name]['velocity'])
                    elif (joint_control_mode[robot_name] == 'torque'):
                        p.setJointMotorControl2(bodyIndex=robot_id[robot_name],
                                                jointIndex=joint_name_to_index[robot_name][joint_name],
                                                controlMode=p.TORQUE_CONTROL,
                                                force=joint_target[robot_name][joint_name]['effort'])
                    joint_state_lock.acquire()
                    target_position_str = target_position_str + str(round(joint_target[robot_name][joint_name]['position'], 5)) + ', '
                    joint_state_str = joint_state_str + str(round(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][0], 5)) + ', '
                    pos_diff_str = pos_diff_str + str(round((joint_target[robot_name][joint_name]['position'] - joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][0]), 5)) + ', '
                    pos_compensation[robot_name][joint_name] += joint_control_integral_gain[robot_name][controlled_joint_name[robot_name].index(joint_name)] * (simulation_step_time / 2) * (joint_target[robot_name][joint_name]['position'] - joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][0])
                    pos_compensation_str = pos_compensation_str + str(round(pos_compensation[robot_name][joint_name], 5)) + ', '
                    joint_state_lock.release()
            target_position_str = target_position_str + ']'
            joint_state_str = joint_state_str + ']'
            pos_compensation_str = pos_compensation_str + ']'
            pos_diff_str = pos_diff_str + ']'
        control_mode_lock.release()

        rate.sleep()


def change_control_mode(srv,
                        p,
                        robot_id,
                        joint_name_to_index,
                        controlled_joint_name,
                        joint_control_mode,
                        joint_effort_limits,
                        control_mode_lock):
    control_mode_lock.acquire()
    for robot_name in srv.robot_names:
        if robot_name in robot_id.keys():
            rospy.loginfo(str(controlled_joint_name[robot_name]))
            for joint_name in controlled_joint_name[robot_name]:
                joint_control_mode[robot_name] = srv.control_mode
                if (joint_control_mode[robot_name] == 'position'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.POSITION_CONTROL,
                                            force=joint_effort_limits[robot_name][joint_name])
                    rospy.loginfo('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'position' + ', max_force: ' + str(joint_effort_limits[robot_name][joint_name]))
                elif (joint_control_mode[robot_name] == 'velocity'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=joint_effort_limits[robot_name][joint_name])
                    rospy.loginfo('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'velocity' + ', max_force: ' + str(joint_effort_limits[robot_name][joint_name]))
                elif (joint_control_mode[robot_name] == 'torque'):
                    rospy.logwarn('In torque joint_control')
                    rospy.logwarn('robot_name: ' + robot_name)
                    rospy.logwarn('joint_name: ' + joint_name)
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=0.0)
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.TORQUE_CONTROL,
                                            force=0.0)
                    rospy.loginfo('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'torque' + ', force: 0')
                else:
                    rospy.logerr(srv.control_mode + ' control mode not exists')
                    control_mode_lock.release()
                    return 'false'
        else:
            rospy.logerr(robot_name + ' robot not exists')
            control_mode_lock.release()
            return 'false'
    control_mode_lock.release()
    return 'true'


def spawn_model(srv,
                p,
                objects,
                obj_tf_pub_thread,
                scenes, use_moveit,
                objects_lock,
                scenes_lock,
                pybullet_ns):
    if not obj_tf_pub_thread.is_alive():
        obj_tf_pub_thread.start()
    if not srv.model_name:
        rospy.logerr('Name list is empty')
        return 'false'
    if not srv.pose:
        rospy.logerr('Poses list is empty')
        return 'false'
    if (len(srv.model_name) != len(srv.pose) or len(srv.model_name) != len(srv.object_name)):
        rospy.logerr('Object name, model name and pose do not have the same leng')
    for x in range(len(srv.model_name)):
        object_name = srv.object_name[x]
        model_name = srv.model_name[x]
        pose = srv.pose[x]
        objects_lock.acquire()
        if object_name in objects.keys():
            rospy.logerr('Already exists an object with this name')
            objects_lock.release()
            return 'false'
        objects_lock.release()
        rospy.loginfo('You want to spawn a ' + model_name + ' with the name ' + object_name)

        fixed = 0
        if rospy.has_param('/' + pybullet_ns + '/objects/' + model_name):
            model_info = rospy.get_param('/' + pybullet_ns + '/objects/' + model_name)

            if 'package_name' in model_info:
                package_name = model_info['package_name']
                rospack = rospkg.RosPack()
                folder_path = rospack.get_path(package_name)
                rospy.loginfo('  package_path: ' + folder_path)
            else:
                rospy.logerr('No param /' + model_name + '/package_name')
                raise SystemExit
            if 'urdf_file_path' in model_info:
                file_type = 'urdf'
                urdf_file_path = model_info['urdf_file_path']
                rospy.loginfo('  urdf_file_path: ' + urdf_file_path)
                urdf_path = folder_path + '/' + urdf_file_path
                if (urdf_path.find('.urdf') == -1):
                    rospy.logerr('  urdf_file_path do not has extension .urdf')
                    raise SystemExit
            elif 'xacro_file_path' in model_info:
                file_type = 'xacro'
                xacro_file_path = model_info['xacro_file_path']
                rospy.loginfo('  xacro_file_name: ' + xacro_file_path)
                xacro_path = folder_path + '/' + xacro_file_path
                if (xacro_path.find('.xacro') != -1):
                    urdf_path = xacro_path.replace('.xacro', '.urdf')
                else:
                    rospy.logerr('  xacro_file_path do not has extension .xacro')
                    raise SystemExit
            else:
                rospy.logerr('No param /' + model_name + '/xacro_file_path(or urdf_file_path)')
                raise SystemExit
            if (use_moveit == 'true'):
                if 'mesh_file_path' in model_info:
                    mesh_file_path = model_info['mesh_file_path']
                    rospy.loginfo('  mesh_file_path: ' + mesh_file_path)
                    mesh_path = folder_path + '/' + mesh_file_path
                else:
                    rospy.logerr('No param /' + model_name + '/mesh_file_path')
                    raise SystemExit
                if 'mesh_position_offset' in model_info:
                    mesh_position_offset = model_info['mesh_position_offset']
                    rospy.loginfo('  mesh_position_offset: ' + str(mesh_position_offset))
                else:
                    rospy.logerr('No param /' + model_name + '/mesh_position_offset')
                    raise SystemExit
                if 'mesh_orientation_offset' in model_info:
                    mesh_orientation_offset = model_info['mesh_orientation_offset']
                    rospy.loginfo('  mesh_orientation_offset: ' + str(mesh_orientation_offset))
                else:
                    rospy.logerr('No param /' + model_name + '/mesh_orientation_offset')
                    raise SystemExit
        else:
            rospy.logerr('Model param not found')
            return 'false'
        if (file_type == 'xacro'):
            os.system('rosrun xacro xacro ' + xacro_path + ' > ' + urdf_path)

        if (srv.fixed[x]):
            fixed = 1
        else:
            fixed = 0

        start_pos = [pose.position.x, pose.position.y, pose.position.z]
        start_orientation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        objects_lock.acquire()
        objects[object_name] = {}
        objects[object_name]['spawned'] = False
        objects[object_name]['attached'] = False
        objects[object_name]['object_id'] = p.loadURDF(urdf_path, start_pos, start_orientation, 0, fixed, flags = p.URDF_USE_INERTIA_FROM_FILE)
        objects[object_name]['spawned'] = True

        tfl = tf.TransformListener()

        if (use_moveit == 'true'):
            mesh = Mesh()
            with pyassimp.load(mesh_path) as mesh_file:
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

            objects[object_name]['object'] = a_obj

            get_scene_clnt = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
            req = PlanningSceneComponents()
            req.components = sum([PlanningSceneComponents.WORLD_OBJECT_NAMES,
                                  PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
                                  PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS])
            scenes_lock.acquire()
            scenes[0] = get_scene_clnt.call(req).scene
            scenes[0].robot_state.joint_state.name     = []
            scenes[0].robot_state.joint_state.position = []
            scenes[0].robot_state.joint_state.velocity = []
            scenes[0].robot_state.joint_state.effort   = []
            scenes[0].is_diff = True
            scenes[0].robot_state.is_diff = True
            scenes[0].world.collision_objects.append(c_obj)
            scenes_lock.release()
            objects[object_name]['attached'] = False

        objects_lock.release()

        if (use_moveit == 'true'):
            while not (tfl.frameExists(object_name,)):
                rospy.sleep(0.00001)
    rospy.loginfo('Model spawned')
    return 'true'


def delete_model(srv,
                 p,
                 objects,
                 scenes,
                 use_moveit,
                 objects_lock,
                 scenes_lock):
    for object_name in srv.object_name:
        if object_name not in objects.keys():
            print(objects.keys())
            rospy.logwarn(object_name + ' is not in the scene')
            continue
        id = objects[object_name]['object_id']
        if (use_moveit == 'true'):
            apply_scene_clnt = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
            if objects[object_name]['object'] in scenes[0].robot_state.attached_collision_objects:
                rospy.set_param('/' + object_name + '/attached',False)
                while objects[object_name]['attached'] == True:
                    rospy.sleep(0.1)
            scenes_lock.acquire()
            if objects[object_name]['object'].object in scenes[0].world.collision_objects:
                scenes[0].world.collision_objects.remove(objects[object_name]['object'].object)
                objects_lock.acquire()
                objects[object_name]['object'].object.operation = objects[object_name]['object'].object.REMOVE
                objects_lock.release()
                scenes[0].world.collision_objects.append(objects[object_name]['object'].object)
                apply_scene_clnt.call(scenes[0])
                scenes[0].world.collision_objects.remove(objects[object_name]['object'].object)
            scenes_lock.release()
        objects_lock.acquire()
        del objects[object_name]
        objects_lock.release()
        p.removeBody(id)
    return 'true'


def joint_state_publisher(p,
                          robot_id,
                          js_publishers,
                          joint_states,
                          controlled_joint_name,
                          joint_state_publish_rate,
                          joint_name_to_index,
                          internal_constraint_to_joint,
                          scenes,
                          scenes_lock,
                          joint_state_lock):
    name = []
    position = []
    velocity = []
    effort = []
    rate = rospy.Rate(joint_state_publish_rate)
    js_msg = JointState()
    while not rospy.is_shutdown():
        for robot_name in js_publishers.keys():
            joint_state_lock.acquire()
            joint_states[robot_name] = p.getJointStates(robot_id[robot_name], range(p.getNumJoints(robot_id[robot_name])))
            joint_state_lock.release()
            name.clear()
            position.clear()
            velocity.clear()
            effort.clear()
            for joint_name in joint_name_to_index[robot_name].keys():
                if joint_name not in controlled_joint_name[robot_name]:
                    if joint_name not in internal_constraint_to_joint[robot_name].values():
                        continue
                name.append(joint_name)
                position.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][0])
                velocity.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][1])
                if joint_name in internal_constraint_to_joint[robot_name].values():
                    constraint_id = list(internal_constraint_to_joint[robot_name].keys())[list(internal_constraint_to_joint[robot_name].values()).index(joint_name)]
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


def sensor_wrench_publisher(p,
                            robot_id,
                            sw_publishers,
                            joint_name_to_index,
                            joint_state_publish_rate,
                            sensor_offset,
                            sensor_offset_lock):
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


def sensor_reset(srv,
                 p,
                 robot_id,
                 sw_publishers,
                 joint_name_to_index,
                 sensor_offset,
                 sensor_offset_lock):
    robot_name = srv.robot_name
    joint_name = srv.joint_name
    if robot_name not in sw_publishers:
        rospy.logerr(robot_name + ' do not have any sensor')
        return 'false'
    if joint_name not in sw_publishers[robot_name]:
        rospy.logerr('The joint ' + joint_name + ' do not has a sensor')
        rospy.logerr('The sensorized joints are:')
        for sensorized_joint_name in sw_publishers[robot_name].keys():
            rospy.logerr('  ' + sensorized_joint_name)
        return 'false'
    sensor_offset_lock.acquire()
    sensor_wrench = p.getJointStates(robot_id[robot_name], [joint_name_to_index[robot_name][joint_name]])
    sensor_offset[robot_name][joint_name] = [sensor_wrench[0][2][0],
                                             sensor_wrench[0][2][1],
                                             sensor_wrench[0][2][2],
                                             sensor_wrench[0][2][3],
                                             sensor_wrench[0][2][4],
                                             sensor_wrench[0][2][5]]
    sensor_offset_lock.release()
    return 'true'


def objects_tf_publisher(p,
                         pybullet_ns,
                         objects,
                         scenes,
                         use_moveit,
                         objects_lock,
                         scenes_lock,
                         tf_published):
    if rospy.has_param('/' + pybullet_ns + '/object_tf_publish_rate'):
        tf_publish_rate = rospy.get_param('/' + pybullet_ns + '/object_tf_publish_rate')
        rospy.loginfo('object_tf_publish_rate: ' + str(tf_publish_rate))
    else:
        tf_publish_rate = 250
        rospy.logwarn('object_tf_publish_rate not set, default value: ' + str(tf_publish_rate))
    rate = rospy.Rate(tf_publish_rate)
    if (use_moveit == 'true'):
        apply_scene_clnt = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
    br = tf.TransformBroadcaster()
    current_time = rospy.Time.now().to_sec()
    while not rospy.is_shutdown():
        if (current_time != rospy.Time.now().to_sec()):
            objects_lock.acquire()
            for object_name in objects:
                if 'object_id' in objects[object_name]:
                    pose = p.getBasePositionAndOrientation(objects[object_name]['object_id'])
                    br.sendTransform((pose[0][0], pose[0][1], pose[0][2]),
                                     (pose[1][0], pose[1][1], pose[1][2], pose[1][3]),
                                     rospy.Time.now(),
                                     object_name,
                                     "world")
                    if ((use_moveit == 'true') and objects[object_name]['spawned']):
                        scenes_lock.acquire()

                        if (rospy.has_param('/' + object_name + '/attached')):
                            if (rospy.has_param('/' + object_name + '/attached_link')):
                                if (rospy.has_param('/' + object_name + '/touch_links')):
                                    if(rospy.get_param('/' + object_name + '/attached')):
                                        attached_link = rospy.get_param('/' + object_name + '/attached_link')
                                        touch_links = rospy.get_param('/' + object_name + '/touch_links')
                                        objects[object_name]['object'].link_name = attached_link
                                        objects[object_name]['object'].touch_links = touch_links
                                        if not objects[object_name]['attached']:
                                            rospy.logerr("Attach")
                                            scenes[0].robot_state.attached_collision_objects.append(objects[object_name]['object'])
                                            scenes[0].world.collision_objects.remove(objects[object_name]['object'].object)
                                            rospy.loginfo('Added attached obj')
                                            objects[object_name]['attached'] = True
                                    else:
                                        if objects[object_name]['attached']:
                                            if objects[object_name]['object'] in scenes[0].robot_state.attached_collision_objects:
                                                rospy.logerr("Deattach")
                                                scenes[0].robot_state.attached_collision_objects.remove(objects[object_name]['object'])
                                                objects[object_name]['object'].object.operation = objects[object_name]['object'].object.REMOVE
                                                scenes[0].robot_state.attached_collision_objects.append(objects[object_name]['object'])
                                                apply_scene_clnt.call(scenes[0])
                                                scenes[0].robot_state.attached_collision_objects.remove(objects[object_name]['object'])
                                                objects[object_name]['object'].object.operation = objects[object_name]['object'].object.ADD
                                                scenes[0].world.collision_objects.append(objects[object_name]['object'].object)
                                                rospy.loginfo('Removed attached obj')
                                                objects[object_name]['attached'] = False
                        apply_scene_clnt.call(scenes[0])
                        scenes_lock.release()
            objects_lock.release()
            current_time = rospy.Time.now().to_sec()
        tf_published[0] = True
        rate.sleep()


def save_state(srv,
               p,
               state_id,
               joint_states,
               state_js,
               joint_state_lock,
               scenes,
               state_scene,
               scenes_lock):
    if not srv.state_name:
        rospy.logerr('Name is empty')
        return 'false'
    if srv.state_name in state_id.keys():
        rospy.logerr(srv.state_name + ', a state whit this name already exists')
        return 'false'
    joint_state_lock.acquire()
    state_id[srv.state_name] = p.saveState()
    state_js[srv.state_name] = {}
    state_js[srv.state_name] = copy.copy(joint_states)
    joint_state_lock.release()
    scenes_lock.acquire()
    state_scene[srv.state_name] = {}
    state_scene[srv.state_name] = copy.copy(scenes[0])
    scenes_lock.release()
    rospy.loginfo('state ' + srv.state_name + ' saved')
    return 'true'


def restore_state(srv,
                  p,
                  state_id,
                  state_js,
                  joint_name_to_index,
                  jt_publishers,
                  scenes,
                  state_scene,
                  scenes_lock,
                  objects,
                  use_moveit,
                  objects_lock,
                  tf_published):
    objects_lock.acquire()
    if not srv.state_name:
        rospy.logerr('Name is empty')
        return 'false'

    if srv.state_name not in state_id.keys():
        rospy.logerr(srv.state_name + ', no state with this name')
        return 'false'

    jt_msg = JointState()
    name = []
    position = []
    velocity = []
    effort = []


    for robot_name in state_js[srv.state_name]:
        name.clear()
        position.clear()
        velocity.clear()
        effort.clear()
        for joint_name in joint_name_to_index[robot_name]:
            name.append(joint_name)
            position.append(state_js[srv.state_name][robot_name][joint_name_to_index[robot_name][joint_name]][0])
            velocity.append(state_js[srv.state_name][robot_name][joint_name_to_index[robot_name][joint_name]][1])
            effort.append(state_js[srv.state_name][robot_name][joint_name_to_index[robot_name][joint_name]][3])
        jt_msg.header = Header()
        jt_msg.header.stamp = rospy.Time.now()
        jt_msg.name = name
        jt_msg.position = position
        jt_msg.velocity = velocity
        jt_msg.effort = effort
        jt_publishers[robot_name].publish(jt_msg)
    p.restoreState(state_id[srv.state_name])

    rospy.loginfo('state ' + srv.state_name + ' restored')
    tf_published[0] = False
    objects_lock.release()

    while not tf_published[0]:
        rospy.sleep(0.1)
    return 'true'


def delete_state(srv,
                 p,
                 state_id):
    if not srv.state_name:
        rospy.logwarn('Name list is empty')
        return 'true'
    for state_name in srv.state_name:
        if state_name not in state_id.keys():
            rospy.logwarn(state_name + ', no state with this name')
        else:
            p.removeState(state_id[state_name])
            state_id.pop(state_name)
    return 'true'

def collision_check(p, simulation_step_time):
    rate = rospy.Rate(2 / simulation_step_time)
    number_normal_contact = len(p.getContactPoints())

    while not rospy.is_shutdown():
        contact_points = p.getContactPoints()

        if (len(contact_points) > number_normal_contact):
            print('Collision')
        elif (len(contact_points) < number_normal_contact):
            number_normal_contact = len(contact_points)
            print('New number_normal_contact')

        rate.sleep()

def main():
    control_mode_lock = Lock()
    sensor_offset_lock = Lock()
    scenes_lock = Lock()
    objects_lock = Lock()
    joint_state_lock = Lock()
    joint_target_lock = Lock()

    use_moveit = sys.argv[1]
    use_guy = sys.argv[2]
    tf_published = []
    tf_published.append(False)

    current_robots_target_configuration = {}
    controlled_joint_name = {}
    internal_constraint_to_joint = {}
    robot_id = {}
    objects = {}
    link_name_to_index = {}
    joint_control_integral_gain = {}
    joint_name_to_index = {}
    joint_states = {}
    joint_target = {}
    joint_control_mode = {}
    joint_effort_limits = {}
    joint_state_publish_rate = None
    simulation_step_time = None
    desidered_real_step_time = None
    state_js = {}
    state_id = {}
    state_scene = {}
    sw_publishers = {}
    sensor_offset = {}

    rospy.init_node('pybullet_simulation')
    pybullet_ns = 'pybullet_simulation'

    robots = {}
    environment = {}
    rospy.loginfo('Wait for robot names')
    ready = False
    while not ready:
        if rospy.has_param('/' + pybullet_ns):
            if rospy.has_param('/' + pybullet_ns + '/robots'):
                robots = rospy.get_param('/' + pybullet_ns + '/robots')
                rospy.loginfo('Robot:')
                for robot_name in robots.keys():
                    rospy.loginfo(' - ' + robot_name)
            if rospy.has_param('/' + pybullet_ns + '/environment'):
                environment = rospy.get_param('/' + pybullet_ns + '/environment')
                rospy.loginfo('Environment:')
                for environment_part in environment.keys():
                    rospy.loginfo(' - ' + environment_part)
            ready = True

    js_publishers = {}
    jt_publishers = {}
    jt_subscriber = {}

    rospy.loginfo('Topic generated:')
    for robot_name in robots.keys():
        js_topic = '/' + robot_name + '/joint_states'
        js_publishers[robot_name] = rospy.Publisher('/' + robot_name + '/joint_states', JointState, queue_size=1)
        jt_publishers[robot_name] = rospy.Publisher('/' + robot_name + '/joint_target', JointState, queue_size=1)
        rospy.loginfo(' - ' + js_topic)

#    p.connect(p.GUI)  # or p.DIRECT for non-graphical version
    if use_guy:
        p = bc.BulletClient(connection_mode=pybullet.GUI)
    else:
        p = bc.BulletClient(connection_mode=pybullet.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane_transparent.urdf")
    p.setGravity(0, 0, -9.8)
    if rospy.has_param('/' + pybullet_ns + '/camera_init'):
        camera_init = rospy.get_param('/' + pybullet_ns + '/camera_init')
        all_info = True
        if 'distance' not in camera_init:
            rospy.logerr('No param /camera_init/distance')
            all_info = False
        if 'yaw' not in camera_init:
            rospy.logerr('No param /camera_init/yaw')
            all_info = False
        if 'pitch' not in camera_init:
            rospy.logerr('No param /camera_init/pitch')
            all_info = False
        if 'target_position' not in camera_init:
            rospy.logerr('No param /camera_init/target_position')
            all_info = False
        if all_info:
            p.resetDebugVisualizerCamera(cameraDistance      =camera_init['distance'],
                                         cameraYaw           =camera_init['yaw'],
                                         cameraPitch         =camera_init['pitch'],
                                         cameraTargetPosition=camera_init['target_position'])

    if rospy.has_param('/' + pybullet_ns + '/simulation_step_time'):
        simulation_step_time = rospy.get_param('/' + pybullet_ns + '/simulation_step_time')
        rospy.loginfo('simulation_step_time: ' + str(simulation_step_time))
    else:
        rospy.logerr('No param /simulation_step_time')
        raise SystemExit
    if rospy.has_param('/' + pybullet_ns + '/real_step_time'):
        desidered_real_step_time = rospy.get_param('/' + pybullet_ns + '/real_step_time')
        rospy.loginfo('real_step_time: ' + str(desidered_real_step_time))
    else:
        rospy.logerr('No param /real_step_time')
        raise SystemExit
    if rospy.has_param('/' + pybullet_ns + '/joint_state_publish_rate'):
        joint_state_publish_rate = rospy.get_param('/' + pybullet_ns + '/joint_state_publish_rate')
        rospy.loginfo('joint_state_publish_rate: ' + str(joint_state_publish_rate))
    else:
        rospy.logerr('No param /joint_state_publish_rate')
        raise SystemExit

    # p.setAdditionalSearchPath('')

    for robot_name in robots.keys():
        start_configuration = {}
        rospy.loginfo('For robot ' + robot_name + ':')
        internal_constraint_to_joint[robot_name] = {}

        robot_info = robots[robot_name]
        rospy.loginfo(' All parameter:')
        rospy.loginfo('   {')
        for key in robot_info.keys():
            if key == 'controlled_joint_name':
                rospy.loginfo('    ' + key + ': [')
                for value in robot_info[key]:
                    rospy.loginfo('                            ' + str(value))
                rospy.loginfo('                            ]')
                continue
            if key == 'constraints':
                rospy.loginfo('    ' + key + ': ')
                for constr in robot_info[key]:
                    for param_name in constr.keys():
                        if param_name == 'parent_body':
                            rospy.loginfo('      -' + param_name + ': ' + str(constr[param_name]))
                        else:
                            rospy.loginfo('       ' + param_name + ': ' + str(constr[param_name]))
                continue
            rospy.loginfo('    ' + key + ': ' + str(robot_info[key]))
        rospy.loginfo('   }')
        if 'package_name' in robot_info:
            package_name = robot_info['package_name']
            rospack = rospkg.RosPack()
            folder_path = rospack.get_path(package_name)
            rospy.loginfo('  package_path: ' + folder_path)
        else:
            rospy.logerr('No param /' + robot_name + '/package_name')
            raise SystemExit
        if 'urdf_file_path' in robot_info:
            file_type = 'urdf'
            urdf_file_path = robot_info['urdf_file_path']
            rospy.loginfo('  urdf_file_path: ' + urdf_file_path)
            urdf_path = folder_path + '/' + urdf_file_path
            if (urdf_path.find('.urdf') == -1):
                rospy.logerr('  urdf_file_path do not has extension .urdf')
                raise SystemExit
        elif 'xacro_file_path' in robot_info:
            file_type = 'xacro'
            xacro_file_path = robot_info['xacro_file_path']
            rospy.loginfo('  xacro_file_name: ' + xacro_file_path)
            xacro_path = folder_path + '/' + xacro_file_path
            if (xacro_path.find('.xacro') != -1):
                urdf_path = xacro_path.replace('.xacro', '.urdf')
            else:
                rospy.logerr('  xacro_file_path do not has extension .xacro')
                raise SystemExit
        else:
            rospy.logerr('No param /' + robot_name + '/xacro_file_path(or urdf_file_path)')
            raise SystemExit
        if 'start_position' in robot_info:
            start_pos = robot_info['start_position']
        else:
            rospy.logerr('No param /' + robot_name + '/start_position')
            raise SystemExit
        if 'start_orientation' in robot_info:
            start_orientation = p.getQuaternionFromEuler(robot_info['start_orientation'])
        else:
            rospy.logerr('No param /' + robot_name + '/start_orientation')
            raise SystemExit
        if 'fixed' in robot_info:
            fixed = robot_info['fixed']
        else:
            rospy.logerr('No param /' + robot_name + '/fixed')
            raise SystemExit
        if 'controlled_joint_name' in robot_info:
            controlled_joint_name[robot_name] = robot_info['controlled_joint_name']
        else:
            rospy.logwarn('No param /' + robot_name + '/controller_joint_name')
            controlled_joint_name[robot_name] = []
        if 'joint_control_integral_gain' in robot_info:
            joint_control_integral_gain[robot_name] = robot_info['joint_control_integral_gain']
        else:
            rospy.logwarn('No param /' + robot_name + '/joint_control_integral_gain, default value 0.0 for every joint')
            joint_control_integral_gain[robot_name] = numpy.zeros(len(controlled_joint_name[robot_name]))
        if 'start_configuration' in robot_info:
            if (len(controlled_joint_name[robot_name]) == len(robot_info['start_configuration'])):
                for index in range(len(controlled_joint_name[robot_name])):
                    start_configuration[controlled_joint_name[robot_name][index]] = robot_info['start_configuration'][index]
            else:
                rospy.logwarn('start_configuration size is wrong')
                for index in range(len(controlled_joint_name[robot_name])):
                    start_configuration[controlled_joint_name[robot_name][index]] = 0.0
        else:
            rospy.logwarn('No param /' + robot_name + '/start_configuration')
            for index in range(len(controlled_joint_name[robot_name])):
                start_configuration[controlled_joint_name[robot_name][index]] = 0.0

        current_robots_target_configuration[robot_name] = start_configuration

        if (file_type == 'xacro'):
            if 'xacro_params' in robot_info:
                xacro_params = robot_info['xacro_params']
                command_str = "rosrun xacro xacro " + xacro_path + " robot_name:='" + robot_name + "' "
                for xacro_param_name in xacro_params.keys():
                    command_str = command_str + xacro_param_name + ":='" + xacro_params[xacro_param_name] + "' "
                command_str = command_str + "> " + urdf_path
                os.system(command_str)
            else:
                os.system("rosrun xacro xacro " + xacro_path + " robot_name:='" + robot_name + "' > " + urdf_path)

        file = open(urdf_path, "r")
        urdf_str = file.read()
        file.close()

        index = 0
        end = False
        while not end:
            if (urdf_str.find("package://", index, len(urdf_str)) == -1):
                end = True
                continue
            else:
                index = urdf_str.find("package://", index, len(urdf_str))
                index_start_name = index + 10
                index_end_name = urdf_str.find("/", index_start_name, len(urdf_str))

                package_name = urdf_str[index_start_name:index_end_name]
                package_path = rospack.get_path(package_name)

                urdf_str = urdf_str[:index] + package_path + urdf_str[index_end_name:]

        file = open(urdf_path, "w")
        file.write(urdf_str)
        file.close()

        robot_id[robot_name] = p.loadURDF(urdf_path, start_pos, start_orientation, 0, fixed, flags=p.URDF_USE_INERTIA_FROM_FILE)
        rospy.loginfo('  robot_id: ' + str(robot_id[robot_name]))

        link_name_to_index[robot_name] = {p.getBodyInfo(robot_id[robot_name])[0].decode('UTF-8'): -1, }
        joint_name_to_index[robot_name] = {}
        joint_effort_limits[robot_name] = {}

        rospy.loginfo('  joint_info: ')
        for joint_id in range(p.getNumJoints(robot_id[robot_name])):
            link_name = p.getJointInfo(robot_id[robot_name], joint_id)[12].decode('UTF-8')
            link_name_to_index[robot_name][link_name] = joint_id
            joint_name = p.getJointInfo(robot_id[robot_name], joint_id)[1].decode('UTF-8')
            joint_name_to_index[robot_name][joint_name] = joint_id
            joint_effort_limits[robot_name][joint_name] = p.getJointInfo(robot_id[robot_name], joint_id)[10]

            rospy.loginfo('    -jointName: '        + p.getJointInfo(robot_id[robot_name], joint_id)[1].decode('UTF-8'))
            rospy.loginfo('     jointIndex: '       + str(p.getJointInfo(robot_id[robot_name], joint_id)[0]))
            rospy.loginfo('     jointType: '        + str(p.getJointInfo(robot_id[robot_name], joint_id)[2]))
            rospy.loginfo('     qIndex: '           + str(p.getJointInfo(robot_id[robot_name], joint_id)[3]))
            rospy.loginfo('     uIndex: '           + str(p.getJointInfo(robot_id[robot_name], joint_id)[4]))
            rospy.loginfo('     flags: '            + str(p.getJointInfo(robot_id[robot_name], joint_id)[5]))
            rospy.loginfo('     jointDamping: '     + str(p.getJointInfo(robot_id[robot_name], joint_id)[6]))
            rospy.loginfo('     jointFriction: '    + str(p.getJointInfo(robot_id[robot_name], joint_id)[7]))
            rospy.loginfo('     jointLowerLimit: '  + str(p.getJointInfo(robot_id[robot_name], joint_id)[8]))
            rospy.loginfo('     jointUpperLimit: '  + str(p.getJointInfo(robot_id[robot_name], joint_id)[9]))
            rospy.loginfo('     jointMaxForce: '    + str(p.getJointInfo(robot_id[robot_name], joint_id)[10]))
            rospy.loginfo('     jointMaxVelocity: ' + str(p.getJointInfo(robot_id[robot_name], joint_id)[11]))
            rospy.loginfo('     linkName: '         + p.getJointInfo(robot_id[robot_name], joint_id)[12].decode('UTF-8'))
            rospy.loginfo('     jointAxis: '        + str(p.getJointInfo(robot_id[robot_name], joint_id)[13]))
            rospy.loginfo('     parentFramePos: '   + str(p.getJointInfo(robot_id[robot_name], joint_id)[14]))
            rospy.loginfo('     parentFrameOrn: '   + str(p.getJointInfo(robot_id[robot_name], joint_id)[15]))
            rospy.loginfo('     parentIndex: '      + str(p.getJointInfo(robot_id[robot_name], joint_id)[16]))

        if 'constraints' in robot_info:
            constraints = robot_info['constraints']
            rospy.loginfo('  constraints: ')
            for constraint in constraints:
                if 'parent_body' in constraint:
                    parent_body = constraint['parent_body']
                    rospy.loginfo('    - parent_body: ' + parent_body)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_body')
                    raise SystemExit
                if 'parent_link' in constraint:
                    parent_link = constraint['parent_link']
                    rospy.loginfo('      parent_link: ' + parent_link)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_link')
                    raise SystemExit
                if 'child_body' in constraint:
                    child_body = constraint['child_body']
                    rospy.loginfo('      child_body: ' + child_body)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_body')
                    raise SystemExit
                if 'child_link' in constraint:
                    child_link = constraint['child_link']
                    rospy.loginfo('      child_link: ' + child_link)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_link')
                    raise SystemExit
                if 'type' in constraint:
                    param_type = constraint['type']
                    rospy.loginfo('      type: ' + param_type)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/type')
                    raise SystemExit
                if 'axis' in constraint:
                    axis = constraint['axis']
                    rospy.loginfo('      axis: [' +
                            str(axis[0]) + ',' +
                            str(axis[1]) + ',' +
                            str(axis[2]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/axis')
                    raise SystemExit
                if 'parent_frame_position' in constraint:
                    parent_frame_position = constraint['parent_frame_position']
                    rospy.loginfo('      parent_frame_position: [' +
                            str(parent_frame_position[0]) + ',' +
                            str(parent_frame_position[1]) + ',' +
                            str(parent_frame_position[2]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_frame_position')
                    raise SystemExit
                if 'child_frame_position' in constraint:
                    child_frame_position = constraint['child_frame_position']
                    rospy.loginfo('      child_frame_position: [' +
                            str(child_frame_position[0]) + ',' +
                            str(child_frame_position[1]) + ',' +
                            str(child_frame_position[2]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_frame_position')
                    raise SystemExit
                if 'parent_frame_orientation' in constraint:
                    parent_frame_orientation = constraint['parent_frame_orientation']
                    rospy.loginfo('      parent_frame_orientation: [' +
                            str(parent_frame_orientation[0]) + ',' +
                            str(parent_frame_orientation[1]) + ',' +
                            str(parent_frame_orientation[2]) + ',' +
                            str(parent_frame_orientation[3]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_frame_orientation')
                    raise SystemExit
                if 'child_frame_orientation' in constraint:
                    child_frame_orientation = constraint['child_frame_orientation']
                    rospy.loginfo('      child_frame_orientation: [' +
                            str(child_frame_orientation[0]) + ',' +
                            str(child_frame_orientation[1]) + ',' +
                            str(child_frame_orientation[2]) + ',' +
                            str(child_frame_orientation[3]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_frame_orientation')
                    raise SystemExit

                if (param_type == 'prismatic'):
                    constraint_type = p.JOINT_PRISMATIC
                elif (param_type == 'fixed'):
                    constraint_type = p.JOINT_FIXED
                elif (param_type == 'point2point'):
                    constraint_type = p.JOINT_POINT2POINT
                elif (param_type == 'gear'):
                    constraint_type = p.JOINT_GEAR
                else:
                    rospy.logerr('Constraint type not foud')
                    raise SystemExit
                constraint_id = p.createConstraint(robot_id[parent_body],
                                                   link_name_to_index[parent_body][parent_link],
                                                   robot_id[child_body],
                                                   link_name_to_index[child_body][child_link],
                                                   constraint_type,
                                                   axis,
                                                   parent_frame_position,
                                                   child_frame_position,
                                                   parent_frame_orientation,
                                                   child_frame_orientation)
                internal_constraint_to_joint[robot_name][constraint_id] = p.getJointInfo(robot_id[child_body], link_name_to_index[child_body][child_link])[1].decode('UTF-8')
                if (param_type == 'gear'):
                    if 'gear_ratio' in constraint:
                        gear_ratio = constraint['gear_ratio']
                        rospy.loginfo('      gear_ratio: ' + str(gear_ratio))
                        p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                    else:
                        rospy.logwarn('      gear_ratio: not set. Default:1')
                        p.changeConstraint(constraint_id, gearRatio=1)
                    if 'gear_aux_link' in constraint:
                        gear_aux_link = constraint['gear_aux_link']
                        rospy.loginfo('      gear_aux_link: ' + str(gear_aux_link))
                        p.changeConstraint(constraint_id, gearAuxLink=link_name_to_index[robot_name][gear_aux_link])
                    else:
                        rospy.logwarn('      gear_aux_link: not set.')
                if 'erp' in constraint:
                    erp_var = constraint['erp']
                    rospy.loginfo('      erp: ' + str(erp_var))
                else:
                    rospy.logwarn('      erp: not set. Default:1')
                    erp_var = 1
                p.changeConstraint(constraint_id, erp=erp_var)
                if 'max_force' in constraint:
                    max_force = constraint['max_force']
                    rospy.loginfo('      max_force: ' + str(max_force))
                else:
                    rospy.logwarn('      max_force: not set. Default:100')
                    max_force = 100
                p.changeConstraint(constraint_id, maxForce=max_force)
        else:
            rospy.logwarn('No param /' + robot_name + '/constraints')

        if 'sensors' in robot_info:
            sensors = robot_info['sensors']
            if not (isinstance(sensors, list)):
                rospy.logwarn('Param /' + robot_name + '/sensors is not a list')
            else:
                sw_publishers[robot_name] = {}
                sensor_offset[robot_name] = {}
                for sensor in sensors:
                    p.enableJointForceTorqueSensor(robot_id[robot_name], joint_name_to_index[robot_name][sensor], enableSensor=1)
                    sw_publishers[robot_name][sensor] = rospy.Publisher('/' + robot_name + '/' + sensor + '/wrench', WrenchStamped, queue_size=1)
                    sensor_offset[robot_name][sensor] = [0, 0, 0, 0, 0, 0]
        else:
            rospy.logwarn('No param /' + robot_name + '/sensors')

        if 'link_dynamics' in robot_info:
            links_dyn = robot_info['link_dynamics']
            rospy.loginfo('  link dynamics: ')
            for link_dyn in links_dyn:
                if 'link_name' in link_dyn:
                    link_name = link_dyn['link_name']
                    rospy.loginfo('    - link_name: ' + link_name)
                else:
                    rospy.logerr('No param /' + robot_name + '/link_dynamics/link_name')
                    raise SystemExit
                current_dyn_info = p.getDynamicsInfo(robot_id[robot_name], link_name_to_index[robot_name][link_name])
                if 'lateral_friction' in link_dyn:
                    lateral_friction = link_dyn['lateral_friction']
                    rospy.loginfo('      lateral_friction: ' + str(lateral_friction))
                else:
                    lateral_friction = current_dyn_info[1]
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/lateral_friction, current value:' + str(lateral_friction))
                if 'spinning_friction' in link_dyn:
                    spinning_friction = link_dyn['spinning_friction']
                    rospy.loginfo('      spinning_friction: ' + str(spinning_friction))
                else:
                    spinning_friction = current_dyn_info[7]
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/spinning_friction, current value:' + str(spinning_friction))
                if 'rolling_friction' in link_dyn:
                    rolling_friction = link_dyn['rolling_friction']
                    rospy.loginfo('      rolling_friction: ' + str(rolling_friction))
                else:
                    rolling_friction = current_dyn_info[6]
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/rolling_friction, current value:' + str(rolling_friction))
                if 'contact_stiffness' in link_dyn:
                    contact_stiffness = link_dyn['contact_stiffness']
                    rospy.loginfo('      contact_stiffness: ' + str(contact_stiffness))
                else:
                    contact_stiffness = current_dyn_info[9]
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/contact_stiffness, current value:' + str(contact_stiffness))
                if 'contact_damping' in link_dyn:
                    contact_damping = link_dyn['contact_damping']
                    rospy.loginfo('      contact_damping: ' + str(contact_damping))
                else:
                    contact_damping = current_dyn_info[8]
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/contact_damping, current value:' + str(contact_damping))
                if 'linear_damping' in link_dyn:
                    linear_damping = link_dyn['linear_damping']
                    rospy.loginfo('      linear_damping: ' + str(linear_damping))
                else:
                    linear_damping = 0.4
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/linear_damping, current value:' + str(linear_damping))
                if 'angular_damping' in link_dyn:
                    angular_damping = link_dyn['angular_damping']
                    rospy.loginfo('      angular_damping: ' + str(angular_damping))
                else:
                    angular_damping = 0.4
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/angular_damping' + str(angular_damping))
                    raise SystemExit
                if 'friction_anchor' in link_dyn:
                    friction_anchor = link_dyn['friction_anchor']
                    rospy.loginfo('      friction_anchor: ' + str(friction_anchor))
                else:
                    friction_anchor = 0
                    rospy.logwarn('No param /' + robot_name + '/link_dynamics/friction_anchor' + str(friction_anchor))
                p.changeDynamics(robot_id[robot_name], link_name_to_index[robot_name][link_name],
                                 lateralFriction=lateral_friction,
                                 spinningFriction=spinning_friction,
                                 rollingFriction=rolling_friction,
                                 contactStiffness=contact_stiffness,
                                 contactDamping=contact_damping,
                                 linearDamping=linear_damping,
                                 angularDamping=angular_damping,
                                 frictionAnchor=friction_anchor)
        else:
            rospy.logwarn('No param /' + robot_name + '/link_dynamics')
        if 'joint_control_mode' in robot_info:
            joint_control_mode[robot_name] = robot_info['joint_control_mode']
            rospy.loginfo('  joint_control_mode: ' + joint_control_mode[robot_name])
        else:
            if (len(controlled_joint_name[robot_name]) != 0):
                rospy.logerr('No param /' + robot_name + '/joint_control_mode')
                raise SystemExit
            else:
                rospy.logwarn('No param /' + robot_name + '/joint_control_mode')
                joint_control_mode[robot_name] = 'Nothing'

        jt_topic = '/' + robot_name + '/joint_target'

        joint_state_lock.acquire()
        joint_states[robot_name] = p.getJointStates(robot_id[robot_name], range(p.getNumJoints(robot_id[robot_name])))
        joint_state_lock.release()
        joint_target[robot_name] = {}
        for joint_name in joint_name_to_index[robot_name].keys():
            if joint_name in controlled_joint_name[robot_name]:
                joint_target[robot_name][joint_name] = {}
                if (joint_control_mode[robot_name] == 'position'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.POSITION_CONTROL,
                                            targetPosition=start_configuration[joint_name])
                    joint_target[robot_name][joint_name]['position'] = start_configuration[joint_name]
                    rospy.loginfo(joint_name + ' joint set with position control, target position = ' + str(start_configuration[joint_name]))
                elif (joint_control_mode[robot_name] == 'velocity'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            targetVelocity=0.0)
                    joint_target[robot_name][joint_name]['velocity'] = 0.0
                    rospy.loginfo(joint_name + ' joint set with velocity control, target velocity = 0.0')
                elif (joint_control_mode[robot_name] == 'torque'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=0.0)
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.TORQUE_CONTROL,
                                            force=0.0)
                    joint_target[robot_name][joint_name]['effort'] = 0.0
                    rospy.loginfo(joint_name + ' joint set with torque control, target force = 0.0')
                else:
                    rospy.logerr('/' + robot_name + '/joint_control_mode not in existing types')
                    raise SystemExit
            else:
                p.setJointMotorControl2(robot_id[robot_name],
                                        joint_name_to_index[robot_name][joint_name],
                                        p.VELOCITY_CONTROL,
                                        force=0.0)
                rospy.loginfo(joint_name + ' joint set with velocity control, max force = 0.0')

        jt_subscriber[robot_name] = JointTargetSubscriber(p, joint_target,
                                                          joint_target_lock,
                                                          robot_name,
                                                          jt_topic)
        if 'gripper' in robot_info:
            gripper_jt_topic = '/' + robot_info['gripper'] + '/joint_target'
            jt_subscriber[robot_info['gripper']] = JointTargetSubscriber(joint_target,
                                                                         joint_target_lock,
                                                                         robot_name,
                                                                         gripper_jt_topic)

    for env_part in environment.keys():
        start_configuration = {}
        rospy.loginfo('For env_part ' + env_part + ':')
        internal_constraint_to_joint[env_part] = {}

        env_info = environment[env_part]
        rospy.loginfo(' All parameter:')
        rospy.loginfo('   {')
        for key in env_info.keys():
            rospy.loginfo('    ' + key + ': ' + str(env_info[key]))
        rospy.loginfo('   }')
        if 'package_name' in env_info:
            package_name = env_info['package_name']
            rospack = rospkg.RosPack()
            folder_path = rospack.get_path(package_name)
            rospy.loginfo('  package_path: ' + folder_path)
        else:
            rospy.logerr('No param /' + env_part + '/package_name')
            raise SystemExit
        if 'urdf_file_path' in env_info:
            file_type = 'urdf'
            urdf_file_path = env_info['urdf_file_path']
            rospy.loginfo('  urdf_file_path: ' + urdf_file_path)
            urdf_path = folder_path + '/' + urdf_file_path
            if (urdf_path.find('.urdf') == -1):
                rospy.logerr('  urdf_file_path do not has extension .urdf')
                raise SystemExit
        elif 'xacro_file_path' in env_info:
            file_type = 'xacro'
            xacro_file_path = env_info['xacro_file_path']
            rospy.loginfo('  xacro_file_name: ' + xacro_file_path)
            xacro_path = folder_path + '/' + xacro_file_path
            if (xacro_path.find('.xacro') != -1):
                urdf_path = xacro_path.replace('.xacro', '.urdf')
            else:
                rospy.logerr('  xacro_file_path do not has extension .xacro')
                raise SystemExit
        else:
            rospy.logerr('No param /' + env_part + '/xacro_file_path(or urdf_file_path)')
            raise SystemExit
        if 'start_position' in env_info:
            start_pos = env_info['start_position']
            rospy.loginfo('  start_pos: [' +
                    str(start_pos[0]) + ',' +
                    str(start_pos[1]) + ',' +
                    str(start_pos[2]) + ']')
        else:
            rospy.logerr('No param /' + env_part + '/start_position')
            raise SystemExit
        if 'start_orientation' in env_info:
            start_orientation = p.getQuaternionFromEuler(env_info['start_orientation'])
            rospy.loginfo('  start_orientation: [' +
                    str(start_orientation[0]) + ',' +
                    str(start_orientation[1]) + ',' +
                    str(start_orientation[2]) + ',' +
                    str(start_orientation[3]) + ']')
        else:
            rospy.logerr('No param /' + env_part + '/start_orientation')
            raise SystemExit
        if 'fixed' in env_info:
            fixed = env_info['fixed']
            rospy.loginfo('  fixed: ' + str(fixed))
        else:
            rospy.logerr('No param /' + env_part + '/fixed')
            raise SystemExit
        if 'controlled_joint_name' in env_info:
            controlled_joint_name[env_part] = env_info['controlled_joint_name']
            array_str = '  controlled_joint_name: ['
            for joint_controlled_name in controlled_joint_name[env_part]:
                array_str += joint_controlled_name + ' '
            array_str += ']'
            rospy.loginfo(array_str)
        else:
            rospy.logwarn('No param /' + env_part + '/controller_joint_name')
            controlled_joint_name[env_part] = []
        if 'joint_control_integral_gain' in env_info:
            joint_control_integral_gain[env_part] = env_info['joint_control_integral_gain']
            array_str = '  joint_control_integral_gain: ['
            for integral_gain in joint_control_integral_gain[env_part]:
                array_str += str(integral_gain) + ' '
            array_str += ']'
            rospy.loginfo(array_str)
        else:
            rospy.logwarn('No param /' + env_part + '/joint_control_integral_gain, default value 0.0 for every joint')
            joint_control_integral_gain[env_part] = numpy.zeros(len(controlled_joint_name[robot_name]))
        if 'start_configuration' in env_info:
            if (len(controlled_joint_name[env_part]) == len(env_info['start_configuration'])):
                for index in range(len(controlled_joint_name[env_part])):
                    start_configuration[controlled_joint_name[env_part][index]] = env_info['start_configuration'][index]
            else:
                rospy.logwarn('start_configuration size is wrong')
                for index in range(len(controlled_joint_name[env_part])):
                    start_configuration[controlled_joint_name[env_part][index]] = 0.0
        else:
            rospy.logwarn('No param /' + env_part + '/start_configuration')
            for index in range(len(controlled_joint_name[env_part])):
                start_configuration[controlled_joint_name[env_part][index]] = 0.0

        current_robots_target_configuration[env_part] = start_configuration

        if (file_type == 'xacro'):
            if 'xacro_params' in env_info:
                xacro_params = env_info['xacro_params']
                command_str = "rosrun xacro xacro " + xacro_path + " env_part:='" + env_part + "' "
                for xacro_param_name in xacro_params.keys():
                    command_str = command_str + xacro_param_name + ":='" + xacro_params[xacro_param_name] + "' "
                command_str = command_str + "> " + urdf_path
                os.system(command_str)
            else:
                os.system("rosrun xacro xacro " + xacro_path + " env_part:='" + env_part + "' > " + urdf_path)

        file = open(urdf_path, "r")
        urdf_str = file.read()
        file.close()

        index = 0
        end = False
        while not end:
            if (urdf_str.find("package://", index, len(urdf_str)) == -1):
                end = True
                continue
            else:
                index = urdf_str.find("package://", index, len(urdf_str))
                index_start_name = index + 10
                index_end_name = urdf_str.find("/", index_start_name, len(urdf_str))

                package_name = urdf_str[index_start_name:index_end_name]
                package_path = rospack.get_path(package_name)

                urdf_str = urdf_str[:index] + package_path + urdf_str[index_end_name:]

        file = open(urdf_path, "w")
        file.write(urdf_str)
        file.close()

        robot_id[env_part] = p.loadURDF(urdf_path, start_pos, start_orientation, 0, fixed, flags=p.URDF_USE_INERTIA_FROM_FILE)
        rospy.loginfo('  robot_id: ' + str(robot_id[env_part]))

        link_name_to_index[env_part] = {p.getBodyInfo(robot_id[env_part])[0].decode('UTF-8'): -1, }
        joint_name_to_index[env_part] = {}
        joint_effort_limits[env_part] = {}

        rospy.loginfo('  joint_info: ')
        for joint_id in range(p.getNumJoints(robot_id[env_part])):
            link_name = p.getJointInfo(robot_id[env_part], joint_id)[12].decode('UTF-8')
            link_name_to_index[env_part][link_name] = joint_id
            joint_name = p.getJointInfo(robot_id[env_part], joint_id)[1].decode('UTF-8')
            joint_name_to_index[env_part][joint_name] = joint_id
            joint_effort_limits[env_part][joint_name] = p.getJointInfo(robot_id[env_part], joint_id)[10]

            rospy.loginfo('    -jointName: '        + p.getJointInfo(robot_id[env_part], joint_id)[1].decode('UTF-8'))
            rospy.loginfo('     jointIndex: '       + str(p.getJointInfo(robot_id[env_part], joint_id)[0]))
            rospy.loginfo('     jointType: '        + str(p.getJointInfo(robot_id[env_part], joint_id)[2]))
            rospy.loginfo('     qIndex: '           + str(p.getJointInfo(robot_id[env_part], joint_id)[3]))
            rospy.loginfo('     uIndex: '           + str(p.getJointInfo(robot_id[env_part], joint_id)[4]))
            rospy.loginfo('     flags: '            + str(p.getJointInfo(robot_id[env_part], joint_id)[5]))
            rospy.loginfo('     jointDamping: '     + str(p.getJointInfo(robot_id[env_part], joint_id)[6]))
            rospy.loginfo('     jointFriction: '    + str(p.getJointInfo(robot_id[env_part], joint_id)[7]))
            rospy.loginfo('     jointLowerLimit: '  + str(p.getJointInfo(robot_id[env_part], joint_id)[8]))
            rospy.loginfo('     jointUpperLimit: '  + str(p.getJointInfo(robot_id[env_part], joint_id)[9]))
            rospy.loginfo('     jointMaxForce: '    + str(p.getJointInfo(robot_id[env_part], joint_id)[10]))
            rospy.loginfo('     jointMaxVelocity: ' + str(p.getJointInfo(robot_id[env_part], joint_id)[11]))
            rospy.loginfo('     linkName: '         + p.getJointInfo(robot_id[env_part], joint_id)[12].decode('UTF-8'))
            rospy.loginfo('     jointAxis: '        + str(p.getJointInfo(robot_id[env_part], joint_id)[13]))
            rospy.loginfo('     parentFramePos: '   + str(p.getJointInfo(robot_id[env_part], joint_id)[14]))
            rospy.loginfo('     parentFrameOrn: '   + str(p.getJointInfo(robot_id[env_part], joint_id)[15]))
            rospy.loginfo('     parentIndex: '      + str(p.getJointInfo(robot_id[env_part], joint_id)[16]))


        if 'constraints' in env_info:
            constraints = env_info['constraints']
            rospy.loginfo('  constraints: ')
            for constraint in constraints:
                if 'parent_body' in constraint:
                    parent_body = constraint['parent_body']
                    rospy.loginfo('    - parent_body: ' + parent_body)
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/parent_body')
                    raise SystemExit
                if 'parent_link' in constraint:
                    parent_link = constraint['parent_link']
                    rospy.loginfo('      parent_link: ' + parent_link)
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/parent_link')
                    raise SystemExit
                if 'child_body' in constraint:
                    child_body = constraint['child_body']
                    rospy.loginfo('      child_body: ' + child_body)
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/child_body')
                    raise SystemExit
                if 'child_link' in constraint:
                    child_link = constraint['child_link']
                    rospy.loginfo('      child_link: ' + child_link)
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/child_link')
                    raise SystemExit
                if 'type' in constraint:
                    param_type = constraint['type']
                    rospy.loginfo('      type: ' + param_type)
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/type')
                    raise SystemExit
                if 'axis' in constraint:
                    axis = constraint['axis']
                    rospy.loginfo('      axis: [' +
                            str(axis[0]) + ',' +
                            str(axis[1]) + ',' +
                            str(axis[2]) + ']')
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/axis')
                    raise SystemExit
                if 'parent_frame_position' in constraint:
                    parent_frame_position = constraint['parent_frame_position']
                    rospy.loginfo('      parent_frame_position: [' +
                            str(parent_frame_position[0]) + ',' +
                            str(parent_frame_position[1]) + ',' +
                            str(parent_frame_position[2]) + ']')
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/parent_frame_position')
                    raise SystemExit
                if 'child_frame_position' in constraint:
                    child_frame_position = constraint['child_frame_position']
                    rospy.loginfo('      child_frame_position: [' +
                            str(child_frame_position[0]) + ',' +
                            str(child_frame_position[1]) + ',' +
                            str(child_frame_position[2]) + ']')
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/child_frame_position')
                    raise SystemExit
                if 'parent_frame_orientation' in constraint:
                    parent_frame_orientation = constraint['parent_frame_orientation']
                    rospy.loginfo('      parent_frame_orientation: [' +
                            str(parent_frame_orientation[0]) + ',' +
                            str(parent_frame_orientation[1]) + ',' +
                            str(parent_frame_orientation[2]) + ',' +
                            str(parent_frame_orientation[3]) + ']')
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/parent_frame_orientation')
                    raise SystemExit
                if 'child_frame_orientation' in constraint:
                    child_frame_orientation = constraint['child_frame_orientation']
                    rospy.loginfo('      child_frame_orientation: [' +
                            str(child_frame_orientation[0]) + ',' +
                            str(child_frame_orientation[1]) + ',' +
                            str(child_frame_orientation[2]) + ',' +
                            str(child_frame_orientation[3]) + ']')
                else:
                    rospy.logerr('No param /' + env_part + '/constraint/child_frame_orientation')
                    raise SystemExit

                if (param_type == 'prismatic'):
                    constraint_type = p.JOINT_PRISMATIC
                elif (param_type == 'fixed'):
                    constraint_type = p.JOINT_FIXED
                elif (param_type == 'point2point'):
                    constraint_type = p.JOINT_POINT2POINT
                elif (param_type == 'gear'):
                    constraint_type = p.JOINT_GEAR
                else:
                    rospy.logerr('Constraint type not foud')
                    raise SystemExit
                constraint_id = p.createConstraint(robot_id[parent_body],
                                                   link_name_to_index[parent_body][parent_link],
                                                   robot_id[child_body],
                                                   link_name_to_index[child_body][child_link],
                                                   constraint_type,
                                                   axis,
                                                   parent_frame_position,
                                                   child_frame_position,
                                                   parent_frame_orientation,
                                                   child_frame_orientation)
                internal_constraint_to_joint[env_part][constraint_id] = p.getJointInfo(robot_id[child_body], link_name_to_index[child_body][child_link])[1].decode('UTF-8')
                if (param_type == 'gear'):
                    if 'gear_ratio' in constraint:
                        gear_ratio = constraint['gear_ratio']
                        rospy.loginfo('      gear_ratio: ' + str(gear_ratio))
                        p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                    else:
                        rospy.logwarn('      gear_ratio: not set. Default:1')
                        p.changeConstraint(constraint_id, gearRatio=1)
                    if 'gear_aux_link' in constraint:
                        gear_aux_link = constraint['gear_aux_link']
                        rospy.loginfo('      gear_aux_link: ' + str(gear_aux_link))
                        p.changeConstraint(constraint_id, gearAuxLink=link_name_to_index[robot_name][gear_aux_link])
                    else:
                        rospy.logwarn('      gear_aux_link: not set.')
                if 'erp' in constraint:
                    erp_var = constraint['erp']
                    rospy.loginfo('      erp: ' + str(erp_var))
                else:
                    rospy.logwarn('      erp: not set. Default:1')
                    erp_var = 1
                p.changeConstraint(constraint_id, erp=erp_var)
                if 'max_force' in constraint:
                    max_force = constraint['max_force']
                    rospy.loginfo('      max_force: ' + str(max_force))
                else:
                    rospy.logwarn('      max_force: not set. Default:100')
                    max_force = 100
                p.changeConstraint(constraint_id, maxForce=max_force)
        else:
            rospy.logwarn('No param /' + env_part + '/constraints')

        if 'link_dynamics' in env_info:
            links_dyn = env_info['link_dynamics']
            rospy.loginfo('  link dynamics: ')
            for link_dyn in links_dyn:
                if 'link_name' in link_dyn:
                    link_name = link_dyn['link_name']
                    rospy.loginfo('    - link_name: ' + link_name)
                else:
                    rospy.logerr('No param /' + env_part + '/link_dynamics/link_name')
                    raise SystemExit
                current_dyn_info = p.getDynamicsInfo(robot_id[env_part], link_name_to_index[robot_name][link_name])
                if 'lateral_friction' in link_dyn:
                    lateral_friction = link_dyn['lateral_friction']
                    rospy.loginfo('      lateral_friction: ' + str(lateral_friction))
                else:
                    lateral_friction = current_dyn_info[1]
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/lateral_friction, current value:' + str(lateral_friction))
                if 'spinning_friction' in link_dyn:
                    spinning_friction = link_dyn['spinning_friction']
                    rospy.loginfo('      spinning_friction: ' + str(spinning_friction))
                else:
                    spinning_friction = current_dyn_info[7]
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/spinning_friction, current value:' + str(spinning_friction))
                if 'rolling_friction' in link_dyn:
                    rolling_friction = link_dyn['rolling_friction']
                    rospy.loginfo('      rolling_friction: ' + str(rolling_friction))
                else:
                    rolling_friction = current_dyn_info[6]
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/rolling_friction, current value:' + str(rolling_friction))
                if 'contact_stiffness' in link_dyn:
                    contact_stiffness = link_dyn['contact_stiffness']
                    rospy.loginfo('      contact_stiffness: ' + str(contact_stiffness))
                else:
                    contact_stiffness = current_dyn_info[9]
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/contact_stiffness, current value:' + str(contact_stiffness))
                if 'contact_damping' in link_dyn:
                    contact_damping = link_dyn['contact_damping']
                    rospy.loginfo('      contact_damping: ' + str(contact_damping))
                else:
                    contact_damping = current_dyn_info[8]
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/contact_damping, current value:' + str(contact_damping))
                if 'linear_damping' in link_dyn:
                    linear_damping = link_dyn['linear_damping']
                    rospy.loginfo('      linear_damping: ' + str(linear_damping))
                else:
                    linear_damping = 0.4
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/linear_damping, current value:' + str(linear_damping))
                if 'angular_damping' in link_dyn:
                    angular_damping = link_dyn['angular_damping']
                    rospy.loginfo('      angular_damping: ' + str(angular_damping))
                else:
                    angular_damping = 0.4
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/angular_damping' + str(angular_damping))
                    raise SystemExit
                if 'friction_anchor' in link_dyn:
                    friction_anchor = link_dyn['friction_anchor']
                    rospy.loginfo('      friction_anchor: ' + str(friction_anchor))
                else:
                    friction_anchor = 0
                    rospy.logwarn('No param /' + env_part + '/link_dynamics/friction_anchor' + str(friction_anchor))
                p.changeDynamics(robot_id[env_part], link_name_to_index[env_part][link_name],
                                 lateralFriction=lateral_friction,
                                 spinningFriction=spinning_friction,
                                 rollingFriction=rolling_friction,
                                 contactStiffness=contact_stiffness,
                                 contactDamping=contact_damping,
                                 linearDamping=linear_damping,
                                 angularDamping=angular_damping,
                                 frictionAnchor=friction_anchor)
        else:
            rospy.logwarn('No param /' + env_part + '/link_dynamics')



    if rospy.has_param('/' + pybullet_ns + '/external_constraint'):
        rospy.loginfo('External constraints: ')
        constraints = rospy.get_param('/' + pybullet_ns + '/external_constraint')
        if isinstance(constraints, list):
            for constraint in constraints:
                if not isinstance(constraint, dict):
                    rospy.logerr('Single constraint is not a dictionary')
                    raise SystemExit
                if 'parent_body' in constraint:
                    parent_body = constraint['parent_body']
                    rospy.loginfo('    - parent_body: ' + parent_body)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_body')
                    raise SystemExit
                if 'parent_link' in constraint:
                    parent_link = constraint['parent_link']
                    rospy.loginfo('      parent_link: ' + parent_link)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_link')
                    raise SystemExit
                if 'child_body' in constraint:
                    child_body = constraint['child_body']
                    rospy.loginfo('      child_body: ' + child_body)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_body')
                    raise SystemExit
                if 'child_link' in constraint:
                    child_link = constraint['child_link']
                    rospy.loginfo('      child_link: ' + child_link)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_link')
                    raise SystemExit
                if 'type' in constraint:
                    param_type = constraint['type']
                    rospy.loginfo('      type: ' + param_type)
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/type')
                    raise SystemExit
                if 'axis' in constraint:
                    axis = constraint['axis']
                    rospy.loginfo('      axis: [' +
                            str(axis[0]) + ',' +
                            str(axis[1]) + ',' +
                            str(axis[2]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/axis')
                    raise SystemExit
                if 'parent_frame_position' in constraint:
                    parent_frame_position = constraint['parent_frame_position']
                    rospy.loginfo('      parent_frame_position: [' +
                            str(parent_frame_position[0]) + ',' +
                            str(parent_frame_position[1]) + ',' +
                            str(parent_frame_position[2]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_frame_position')
                    raise SystemExit
                if 'child_frame_position' in constraint:
                    child_frame_position = constraint['child_frame_position']
                    rospy.loginfo('      child_frame_position: [' +
                            str(child_frame_position[0]) + ',' +
                            str(child_frame_position[1]) + ',' +
                            str(child_frame_position[2]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_frame_position')
                    raise SystemExit
                if 'parent_frame_orientation' in constraint:
                    parent_frame_orientation = constraint['parent_frame_orientation']
                    rospy.loginfo('      parent_frame_orientation: [' +
                            str(parent_frame_orientation[0]) + ',' +
                            str(parent_frame_orientation[1]) + ',' +
                            str(parent_frame_orientation[2]) + ',' +
                            str(parent_frame_orientation[3]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/parent_frame_orientation')
                    raise SystemExit
                if 'child_frame_orientation' in constraint:
                    child_frame_orientation = constraint['child_frame_orientation']
                    rospy.loginfo('      child_frame_orientation: [' +
                            str(child_frame_orientation[0]) + ',' +
                            str(child_frame_orientation[1]) + ',' +
                            str(child_frame_orientation[2]) + ',' +
                            str(child_frame_orientation[3]) + ']')
                else:
                    rospy.logerr('No param /' + robot_name + '/constraint/child_frame_orientation')
                    raise SystemExit
                if (param_type == 'prismatic'):
                    constraint_type = p.JOINT_PRISMATIC
                elif (param_type == 'fixed'):
                    constraint_type = p.JOINT_FIXED
                elif (param_type == 'point2point'):
                    constraint_type = p.JOINT_POINT2POINT
                elif (param_type == 'gear'):
                    constraint_type = p.JOINT_GEAR
                else:
                    rospy.logerr('Constraint type not foud')
                    raise SystemExit
                constraint_id = p.createConstraint(robot_id[parent_body],
                                                   link_name_to_index[parent_body][parent_link],
                                                   robot_id[child_body],
                                                   link_name_to_index[child_body][child_link],
                                                   constraint_type,
                                                   axis,
                                                   parent_frame_position,
                                                   child_frame_position,
                                                   parent_frame_orientation,
                                                   child_frame_orientation)

                internal_constraint_to_joint[robot_name][constraint_id] = p.getJointInfo(robot_id[child_body], link_name_to_index[child_body][child_link])[1].decode('UTF-8')

                if (param_type == 'gear'):
                    if 'gear_ratio' in constraint:
                        gear_ratio = constraint['gear_ratio']
                        rospy.loginfo('      gear_ratio: ' + str(gear_ratio))
                        p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                    else:
                        rospy.logwarn('      gear_ratio: not set. Default:1')
                        p.changeConstraint(constraint_id, gearRatio=1)
                    if 'gear_aux_link' in constraint:
                        gear_aux_link = constraint['gear_aux_link']
                        rospy.loginfo('      gear_aux_link: ' + str(gear_aux_link))
                        p.changeConstraint(constraint_id, gearAuxLink=link_name_to_index[robot_name][gear_aux_link])
                    else:
                        rospy.logwarn('      gear_aux_link: not set.')
                if 'erp' in constraint:
                    erp_var = constraint['erp']
                    rospy.loginfo('      erp: ' + str(erp_var))
                else:
                    rospy.logwarn('      erp: not set. Default:1')
                    erp_var = 1
                p.changeConstraint(constraint_id, erp=erp_var)
                if 'max_force' in constraint:
                    max_force = constraint['max_force']
                    rospy.loginfo('      max_force: ' + str(max_force))
                    p.changeConstraint(constraint_id, maxForce=max_force)
                else:
                    rospy.logwarn('      max_force: not set. Default:100')
                    p.changeConstraint(constraint_id, maxForce=100)
        else:
            rospy.logerr('external_constraint is not a list')
            raise SystemExit
    else:
        rospy.logwarn('No external constraint')

    rospy.Service('change_control_mode',
                  ChangeControlMode,
                  lambda msg:
                      change_control_mode(msg,
                                          p,
                                          robot_id,
                                          joint_name_to_index,
                                          controlled_joint_name,
                                          joint_control_mode,
                                          joint_effort_limits,
                                          control_mode_lock))

    rospy.Service('pybullet_save_state',
                  SaveState,
                  lambda msg: save_state(msg,
                                         p,
                                         state_id,
                                         joint_states,
                                         state_js,
                                         joint_state_lock,
                                         scenes,
                                         state_scene,scenes_lock))

    rospy.Service('pybullet_restore_state',
                  RestoreState,
                  lambda msg: restore_state(msg,
                                            p,
                                            state_id,
                                            state_js,
                                            joint_name_to_index,
                                            jt_publishers,
                                            scenes,
                                            state_scene,
                                            scenes_lock,
                                            objects,
                                            use_moveit,
                                            objects_lock,
                                            tf_published))

    rospy.Service('pybullet_delete_state',
                  DeleteState,
                  lambda msg: delete_state(msg, p, state_id))

    rospy.Service('pybullet_sensor_reset',
                  SensorReset,
                  lambda msg: sensor_reset(msg,
                                           p,
                                           robot_id,
                                           sw_publishers,
                                           joint_name_to_index,
                                           sensor_offset,
                                           sensor_offset_lock))

    scenes = []
    scene = PlanningScene()
    scenes.append(scene)

    obj_tf_pub_thread = Thread(target=objects_tf_publisher,
                               args=(p,
                                     pybullet_ns,
                                     objects,
                                     scenes,
                                     use_moveit,
                                     objects_lock,
                                     scenes_lock,
                                     tf_published))
    obj_tf_pub_thread.start()

    rospy.Service('pybullet_spawn_model',
                  SpawnModel,
                  lambda msg:
                      spawn_model(msg,
                                  p,
                                  objects,
                                  obj_tf_pub_thread,
                                  scenes,
                                  use_moveit,
                                  objects_lock,
                                  scenes_lock,
                                  pybullet_ns))
    rospy.Service('pybullet_delete_model',
                  DeleteModel,
                  lambda msg:
                      delete_model(msg,
                                   p,
                                   objects,
                                   scenes,
                                   use_moveit,
                                   objects_lock,
                                   scenes_lock))
    time_pub = rospy.Publisher('/clock', Clock, queue_size=10)

    p.setTimeStep(simulation_step_time)
    p.stepSimulation()
    simulation_time = []
    simulation_time.append(simulation_step_time)
    simulation_time_msg = rospy.Time(simulation_time[0])
    time_pub.publish(simulation_time_msg)

    js_pub_thread = Thread(target=joint_state_publisher, args=(p, robot_id,
                                                               js_publishers,
                                                               joint_states,
                                                               controlled_joint_name,
                                                               joint_state_publish_rate,
                                                               joint_name_to_index,
                                                               internal_constraint_to_joint,
                                                               scenes,
                                                               scenes_lock,
                                                               joint_state_lock))
    js_pub_thread.start()

    jt_integ_thread = Thread(target=joint_target_integration, args=(p, robots.keys(),
                                                                    robot_id,
                                                                    joint_control_mode,
                                                                    controlled_joint_name,
                                                                    joint_states,
                                                                    joint_state_lock,
                                                                    joint_target,
                                                                    joint_target_lock,
                                                                    joint_control_integral_gain,
                                                                    joint_name_to_index,
                                                                    control_mode_lock,
                                                                    simulation_step_time))
    jt_integ_thread.start()

    sw_pub_thread = Thread(target=sensor_wrench_publisher, args=(p, robot_id,
                                                                 sw_publishers,
                                                                 joint_name_to_index,
                                                                 joint_state_publish_rate,
                                                                 sensor_offset,
                                                                 sensor_offset_lock))

    sw_pub_thread.start()

#    collision_thread = Thread(target=collision_check, args=(p, simulation_step_time,))
#    collision_thread.start()

    time.sleep(0.1)

    rospy.loginfo('Ready for simulation')

    now = time.time()
    while not rospy.is_shutdown():
        p.stepSimulation()
        simulation_time[0] += simulation_step_time
        simulation_time_msg = rospy.Time(simulation_time[0])
        time_pub.publish(simulation_time_msg)
        elapsed = time.time() - now
        if (desidered_real_step_time - elapsed > 0):
            time.sleep(desidered_real_step_time - elapsed)
        now = time.time()

    p.disconnect()

    rospy.loginfo('Waiting for threads...')
    js_pub_thread.join()
    obj_tf_pub_thread.join()
    sw_pub_thread.join()
#    collision_thread.join()


if __name__ == '__main__':
    main()
