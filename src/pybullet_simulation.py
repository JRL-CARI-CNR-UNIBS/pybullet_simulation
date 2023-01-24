#!/usr/bin/env python3

import os
import pybullet as p
import pybullet_data
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from threading import Thread
from rosgraph_msgs.msg import Clock
from pybullet_utils.srv import SpawnModel
from pybullet_utils.srv import ChangeControlMode


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def change_control_mode(srv, robot_id, joint_name_to_index, controlled_joint_name, joint_control_mode, joint_effort_limits):
    print(srv.robot_names)
    print(' ')
    print(robot_id.keys())
    for robot_name in srv.robot_names:
        if robot_name in robot_id.keys():
            print(controlled_joint_name[robot_name])
            for joint_name in controlled_joint_name[robot_name]:
                joint_control_mode[robot_name] = srv.control_mode
                if (joint_control_mode[robot_name] == 'position'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.POSITION_CONTROL,
                                            force=joint_effort_limits[robot_name][joint_name])
                    print('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + joint_control_mode[robot_name] + ', max_force: ' + str(joint_effort_limits[robot_name][joint_name]))
                elif (joint_control_mode[robot_name] == 'velocity'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=joint_effort_limits[robot_name][joint_name])
                    print('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + joint_control_mode[robot_name] + ', max_force: ' + str(joint_effort_limits[robot_name][joint_name]))
                elif (joint_control_mode[robot_name] == 'torque'):
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=0.0)
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.TORQUE_CONTROL,
                                            force=0.0)
                    print('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + joint_control_mode[robot_name] + ', force: 0')
                else:
                    print(srv.control_mode + ' control mode not exists')
                    return 'false'
        else:
            print(robot_name + ' robot not exists')
            return 'false'
    return 'true'


def spawn_models(srv):
    for x in range(len(srv.models_names)):
        model_name = srv.models_names[x]
        print(model_name)
        if rospy.has_param(model_name):
            model_info = rospy.get_param(model_name)
            if 'foulder_path' in model_info:
                foulder_path = model_info['foulder_path']
                p.setAdditionalSearchPath(model_info['foulder_path'])
                print(bcolors.OKGREEN + '  foulder_path: ' + foulder_path + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + model_name + '/foulder_path' + bcolors.ENDC)
                return 'false'
            if 'urdf_file_name' in model_info:
                file_type = 'urdf'
                file_name = model_info['urdf_file_name']
                print(bcolors.OKGREEN + '  urdf_file_name: ' + file_name + bcolors.ENDC)
            elif 'xacro_file_name' in model_info:
                file_type = 'xacro'
                file_name = model_info['xacro_file_name']
                print(bcolors.OKGREEN + '  xacro_file_name: ' + file_name + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + model_name + '/xacro_file_name(or urdf_file_name)' + bcolors.ENDC)
                return 'false'
            if 'position' in model_info:
                startPos = model_info['position']
                print(bcolors.OKGREEN + '  startPos: ['
                                      + str(startPos[0]) + ','
                                      + str(startPos[1]) + ','
                                      + str(startPos[2]) + ']'
                                      + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + model_name + '/position' + bcolors.ENDC)
                return 'false'
            if 'orientation' in model_info:
                startOrientation = p.getQuaternionFromEuler(model_info['orientation'])
                print(bcolors.OKGREEN + '  orientation: ['
                                      + str(startOrientation[0]) + ','
                                      + str(startOrientation[1]) + ','
                                      + str(startOrientation[2]) + ']'
                                      + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + model_name + '/orientation' + bcolors.ENDC)
                return 'false'
            if 'fixed' in model_info:
                fixed = model_info['fixed']
                print(bcolors.OKGREEN + '  fixed: ' + str(fixed) + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + model_name + '/fixed' + bcolors.ENDC)
                return 'false'
        else:
            print('Model param not found')
            return 'false'
        if (file_type == 'xacro'):
            xacro_file_name = foulder_path + '/' + file_name + '.xacro'
            urdf_file_name = foulder_path + '/' + file_name + '.urdf'
            os.system('rosrun xacro xacro ' + xacro_file_name + ' > ' + urdf_file_name)

        file_name += '.urdf'
        p.loadURDF(file_name, startPos, startOrientation, 0, fixed)
    print('Model spawned')
    return 'true'


def jointStatePublisher(robot_id, js_publishers, joint_states, joint_state_publish_rate, joint_name_to_index, gear_constraint_to_joint):
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
                name.append(joint_name)
                position.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][0])
                velocity.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][1])
                if joint_name in gear_constraint_to_joint.values():
                    constraint_id = list(gear_constraint_to_joint.keys())[list(gear_constraint_to_joint.values()).index(joint_name)]
                    effort.append(p.getConstraintState(constraint_id)[0])
                else:
                    effort.append(joint_states[robot_name][joint_name_to_index[robot_name][joint_name]][3])
            js_msg.header = Header()
            js_msg.header.stamp = rospy.Time.now()  # ???
            js_msg.name = name
            js_msg.position = position
            js_msg.velocity = velocity
            js_msg.effort = effort
            js_publishers[robot_name].publish(js_msg)
        rate.sleep()


def jointTargetSubscriber(data, robot_id, joint_name_to_index, joint_control_mode, controlled_joint_name):
    for joint_id in range(len(data.name)):
        if data.name[joint_id] in controlled_joint_name:
            if (joint_control_mode == 'position'):
                p.setJointMotorControl2(robot_id, joint_name_to_index[data.name[joint_id]],
                                        p.POSITION_CONTROL, targetPosition=data.position[joint_id])
            elif (joint_control_mode == 'velocity'):
                p.setJointMotorControl2(robot_id, joint_name_to_index[data.name[joint_id]],
                                        p.VELOCITY_CONTROL, targetVelocity=data.velocity[joint_id])
            elif (joint_control_mode == 'torque'):
                p.setJointMotorControl2(bodyIndex=robot_id,
                                        jointIndex=joint_name_to_index[data.name[joint_id]],
                                        controlMode=p.TORQUE_CONTROL,
                                        force=data.effort[joint_id])
#                print(data.name + ' force: ' + data.effort[joint_id])
#            elif (joint_control_mode == 'pd'):
#                p.setJointMotorControl2(robot_id, joint_name_to_index[data.name[joint_id]],
#                                        p.PD_CONTROL, data.position[joint_id])


def main():

    controlled_joint_name = {}
    gear_constraint_to_joint = {}
    robot_id = {}
    link_name_to_index = {}
    joint_name_to_index = {}
    joint_state_publish_rate = None
    simulation_step_time = None
    desidered_real_step_time = None
    joint_states = {}
    joint_control_mode = {}
    joint_effort_limits = {}
    rospy.init_node('pybullet_simulation')

    rospy.Service('pybullet_spawn_model', SpawnModel, spawn_models)
    robot_names = []
    print(bcolors.OKGREEN + 'Wait for robot names' + bcolors.ENDC)
    ready = False
    while not ready:
        if rospy.has_param('/robots'):
            robot_names = rospy.get_param('/robots')
            print(bcolors.OKGREEN + 'Robot:' + bcolors.ENDC)
            for robot_name in robot_names:
                print(bcolors.OKGREEN + ' - ' + robot_name + bcolors.ENDC)
            ready = True

    rospy.set_param('use_sim_time', True)

    js_publishers = {}

    print(bcolors.OKGREEN + 'Topic generated:' + bcolors.ENDC)
    for robot_name in robot_names:
        js_topic = '/' + robot_name + '/joint_states'
        js_publishers[robot_name] = rospy.Publisher('/' + robot_name + '/joint_states', JointState, queue_size=1)
        print(bcolors.OKGREEN + ' - ' + js_topic + bcolors.ENDC)

    physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
#    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    p.setGravity(0, 0, -9.8)
    if rospy.has_param('camera_init'):
        camera_init = rospy.get_param('camera_init')
        all_info = True
        if 'distance' not in camera_init:
            print(bcolors.FAIL + 'No param /camera_init/distance' + bcolors.ENDC)
            all_info = False
        if 'yaw' not in camera_init:
            print(bcolors.FAIL + 'No param /camera_init/yaw' + bcolors.ENDC)
            all_info = False
        if 'pitch' not in camera_init:
            print(bcolors.FAIL + 'No param /camera_init/pitch' + bcolors.ENDC)
            all_info = False
        if 'target_position' not in camera_init:
            print(bcolors.FAIL + 'No param /camera_init/target_position' + bcolors.ENDC)
            all_info = False
        if all_info:
            p.resetDebugVisualizerCamera(cameraDistance      =camera_init['distance'],
                                         cameraYaw           =camera_init['yaw'],
                                         cameraPitch         =camera_init['pitch'],
                                         cameraTargetPosition=camera_init['target_position'])

    if rospy.has_param('/simulation_step_time'):
        simulation_step_time = rospy.get_param('/simulation_step_time')
        print(bcolors.OKGREEN + 'simulation_step_time: ' + str(simulation_step_time) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + 'No param /simulation_step_time' + bcolors.ENDC)
        raise SystemExit
    if rospy.has_param('/real_step_time'):
        desidered_real_step_time = rospy.get_param('/real_step_time')
        print(bcolors.OKGREEN + 'real_step_time: ' + str(desidered_real_step_time) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + 'No param /real_step_time' + bcolors.ENDC)
        raise SystemExit
    if rospy.has_param('/joint_state_publish_rate'):
        joint_state_publish_rate = rospy.get_param('/joint_state_publish_rate')
        print(bcolors.OKGREEN + 'joint_state_publish_rate: ' + str(joint_state_publish_rate) + bcolors.ENDC)
    else:
        print(bcolors.FAIL + 'No param /joint_state_publish_rate' + bcolors.ENDC)
        raise SystemExit

    for robot_name in robot_names:
        print(bcolors.OKGREEN + 'For robot ' + robot_name + ':' + bcolors.ENDC)
        if rospy.has_param('/' + robot_name):
            robot_info = rospy.get_param('/' + robot_name)
            for key in robot_info.keys():
                print(bcolors.OKGREEN + key + ': ' + str(robot_info[key]) + bcolors.ENDC)
#            print(robot_info)
            if 'foulder_path' in robot_info:
                foulder_path = robot_info['foulder_path']
                p.setAdditionalSearchPath(robot_info['foulder_path'])
                print(bcolors.OKGREEN + '  foulder_path: ' + foulder_path + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + robot_name + '/foulder_path' + bcolors.ENDC)
                raise SystemExit
            if 'urdf_file_name' in robot_info:
                file_type = 'urdf'
                file_name = robot_info['urdf_file_name']
                print(bcolors.OKGREEN + '  urdf_file_name: ' + file_name + bcolors.ENDC)
            elif 'xacro_file_name' in robot_info:
                file_type = 'xacro'
                file_name = robot_info['xacro_file_name']
                print(bcolors.OKGREEN + '  xacro_file_name: ' + file_name + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + robot_name + '/xacro_file_name(or urdf_file_name)' + bcolors.ENDC)
                raise SystemExit
            if 'start_position' in robot_info:
                startPos = robot_info['start_position']
                print(bcolors.OKGREEN + '  startPos: ['
                                      + str(startPos[0]) + ','
                                      + str(startPos[1]) + ','
                                      + str(startPos[2]) + ']'
                                      + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + robot_name + '/start_position' + bcolors.ENDC)
                raise SystemExit
            if 'start_orientation' in robot_info:
                startOrientation = p.getQuaternionFromEuler(robot_info['start_orientation'])
                print(bcolors.OKGREEN + '  start_orientation: ['
                                      + str(startOrientation[0]) + ','
                                      + str(startOrientation[1]) + ','
                                      + str(startOrientation[2]) + ']'
                                      + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + robot_name + '/start_orientation' + bcolors.ENDC)
                raise SystemExit
            if 'fixed' in robot_info:
                fixed = robot_info['fixed']
                print(bcolors.OKGREEN + '  fixed: ' + str(fixed) + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + robot_name + '/fixed' + bcolors.ENDC)
                raise SystemExit
            if 'controlled_joint_name' in robot_info:
                controlled_joint_name[robot_name] = robot_info['controlled_joint_name']
                array_str = bcolors.OKGREEN + '  controlled_joint_name: ['
                for joint_controlled_name in controlled_joint_name[robot_name]:
                    array_str += joint_controlled_name + ' '
                array_str += ']' + bcolors.ENDC
                print(array_str)
            else:
                print(bcolors.FAIL + 'No param /' + robot_name + '/controller_joint_name' + bcolors.ENDC)
                raise SystemExit

            if (file_type == 'xacro'):
                xacro_file_name = foulder_path + '/' + file_name + '.xacro'
                urdf_file_name = foulder_path + '/' + file_name + '.urdf'
                os.system('rosrun xacro xacro ' + xacro_file_name + ' > ' + urdf_file_name)

            file_name += '.urdf'
            robot_id[robot_name] = p.loadURDF(file_name, startPos, startOrientation, 0, fixed, flags=p.URDF_USE_INERTIA_FROM_FILE)
            print(bcolors.OKGREEN + '  robot_id: ' + str(robot_id[robot_name]) + bcolors.ENDC)

            link_name_to_index[robot_name] = {p.getBodyInfo(robot_id[robot_name])[0].decode('UTF-8'): -1, }
            joint_name_to_index[robot_name] = {}
            joint_effort_limits[robot_name] = {}

            print(bcolors.OKGREEN + '  joint_info: ' + bcolors.ENDC)
            for joint_id in range(p.getNumJoints(robot_id[robot_name])):
                link_name = p.getJointInfo(robot_id[robot_name], joint_id)[12].decode('UTF-8')
                link_name_to_index[robot_name][link_name] = joint_id
                joint_name = p.getJointInfo(robot_id[robot_name], joint_id)[1].decode('UTF-8')
                joint_name_to_index[robot_name][joint_name] = joint_id
                joint_effort_limits[robot_name][joint_name] = p.getJointInfo(robot_id[robot_name], joint_id)[10]

                print(bcolors.OKGREEN + '    -jointName: '        + p.getJointInfo(robot_id[robot_name], joint_id)[1].decode('UTF-8') + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointIndex: '       + str(p.getJointInfo(robot_id[robot_name], joint_id)[0])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointType: '        + str(p.getJointInfo(robot_id[robot_name], joint_id)[2])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     qIndex: '           + str(p.getJointInfo(robot_id[robot_name], joint_id)[3])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     uIndex: '           + str(p.getJointInfo(robot_id[robot_name], joint_id)[4])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     flags: '            + str(p.getJointInfo(robot_id[robot_name], joint_id)[5])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointDamping: '     + str(p.getJointInfo(robot_id[robot_name], joint_id)[6])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointFriction: '    + str(p.getJointInfo(robot_id[robot_name], joint_id)[7])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointLowerLimit: '  + str(p.getJointInfo(robot_id[robot_name], joint_id)[8])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointUpperLimit: '  + str(p.getJointInfo(robot_id[robot_name], joint_id)[9])  + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointMaxForce: '    + str(p.getJointInfo(robot_id[robot_name], joint_id)[10]) + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointMaxVelocity: ' + str(p.getJointInfo(robot_id[robot_name], joint_id)[11]) + bcolors.ENDC)
                print(bcolors.OKGREEN + '     linkName: '         + p.getJointInfo(robot_id[robot_name], joint_id)[12].decode('UTF-8') + bcolors.ENDC)
                print(bcolors.OKGREEN + '     jointAxis: '        + str(p.getJointInfo(robot_id[robot_name], joint_id)[13]) + bcolors.ENDC)
                print(bcolors.OKGREEN + '     parentFramePos: '   + str(p.getJointInfo(robot_id[robot_name], joint_id)[14]) + bcolors.ENDC)
                print(bcolors.OKGREEN + '     parentFrameOrn: '   + str(p.getJointInfo(robot_id[robot_name], joint_id)[15]) + bcolors.ENDC)
                print(bcolors.OKGREEN + '     parentIndex: '      + str(p.getJointInfo(robot_id[robot_name], joint_id)[16]) + bcolors.ENDC)

            if 'constraints' in robot_info:
                constraints = robot_info['constraints']
                print(bcolors.OKGREEN + '  constraints: ' + bcolors.ENDC)
                for constraint in constraints:
                    if 'parent_body' in constraint:
                        parent_body = constraint['parent_body']
                        print(bcolors.OKGREEN + '    - parent_body: ' + parent_body + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/parent_body' + bcolors.ENDC)
                        raise SystemExit
                    if 'parent_link' in constraint:
                        parent_link = constraint['parent_link']
                        print(bcolors.OKGREEN + '      parent_link: ' + parent_link + bcolors.ENDC)
                    else:
                        print('No param /' + robot_name + '/constraint/parent_link')
                        print(bcolors.FAIL + 'No /robots param' + bcolors.ENDC)
                        raise SystemExit
                    if 'child_body' in constraint:
                        child_body = constraint['child_body']
                        print(bcolors.OKGREEN + '      child_body: ' + child_body + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/child_body' + bcolors.ENDC)
                        raise SystemExit
                    if 'child_link' in constraint:
                        child_link = constraint['child_link']
                        print(bcolors.OKGREEN + '      child_link: ' + child_link + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/child_link' + bcolors.ENDC)
                        raise SystemExit
                    if 'type' in constraint:
                        type = constraint['type']
                        print(bcolors.OKGREEN + '      type: ' + type + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/type' + bcolors.ENDC)
                        raise SystemExit
                    if 'axis' in constraint:
                        axis = constraint['axis']
                        print(bcolors.OKGREEN + '      axis: ['
                                              + str(axis[0]) + ','
                                              + str(axis[1]) + ','
                                              + str(axis[2]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/axis' + bcolors.ENDC)
                        raise SystemExit
                    if 'parent_frame_position' in constraint:
                        parent_frame_position = constraint['parent_frame_position']
                        print(bcolors.OKGREEN + '      parent_frame_position: ['
                                              + str(parent_frame_position[0]) + ','
                                              + str(parent_frame_position[1]) + ','
                                              + str(parent_frame_position[2]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/parent_frame_position' + bcolors.ENDC)
                        raise SystemExit
                    if 'child_frame_position' in constraint:
                        child_frame_position = constraint['child_frame_position']
                        print(bcolors.OKGREEN + '      child_frame_position: ['
                                              + str(child_frame_position[0]) + ','
                                              + str(child_frame_position[1]) + ','
                                              + str(child_frame_position[2]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/child_frame_position' + bcolors.ENDC)
                        raise SystemExit
                    if 'parent_frame_orientation' in constraint:
                        parent_frame_orientation = constraint['parent_frame_orientation']
                        print(bcolors.OKGREEN + '      parent_frame_orientation: ['
                                              + str(parent_frame_orientation[0]) + ','
                                              + str(parent_frame_orientation[1]) + ','
                                              + str(parent_frame_orientation[2]) + ','
                                              + str(parent_frame_orientation[3]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/parent_frame_orientation' + bcolors.ENDC)
                        raise SystemExit
                    if 'child_frame_orientation' in constraint:
                        child_frame_orientation = constraint['child_frame_orientation']
                        print(bcolors.OKGREEN + '      child_frame_orientation: ['
                                              + str(child_frame_orientation[0]) + ','
                                              + str(child_frame_orientation[1]) + ','
                                              + str(child_frame_orientation[2]) + ','
                                              + str(child_frame_orientation[3]) + ']'
                                              + bcolors.ENDC)
                    else:
                        print(bcolors.FAIL + 'No param /' + robot_name + '/constraint/child_frame_orientation' + bcolors.ENDC)
                        raise SystemExit
                    if (type == 'prismatic'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[robot_name][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[robot_name][child_link],
                                                           p.JOINT_PRISMATIC,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    elif (type == 'fixed'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[robot_name][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[robot_name][child_link],
                                                           p.JOINT_FIXED,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    elif (type == 'point2point'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[robot_name][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[robot_name][child_link],
                                                           p.JOINT_POINT2POINT,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    elif (type == 'gear'):
                        constraint_id = p.createConstraint(robot_id[parent_body],
                                                           link_name_to_index[robot_name][parent_link],
                                                           robot_id[child_body],
                                                           link_name_to_index[robot_name][child_link],
                                                           p.JOINT_GEAR,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                    else:
                        print(bcolors.FAIL + 'Constraint type not foud' + bcolors.ENDC)
                        raise SystemExit
                    if (type == 'gear'):
                        gear_constraint_to_joint[constraint_id] = p.getJointInfo(robot_id[child_body], link_name_to_index[child_body][child_link])[1].decode('UTF-8')
                        print(gear_constraint_to_joint)
                        if 'gear_ratio' in constraint:
                            gear_ratio = constraint['gear_ratio']
                            print(bcolors.OKGREEN + '      gear_ratio: ' + str(gear_ratio) + bcolors.ENDC)
                            p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                        else:
                            print(bcolors.WARNING + '      gear_ratio: not set. Default:1' + bcolors.ENDC)
                            p.changeConstraint(constraint_id, gearRatio=1)
                        if 'erp' in constraint:
                            erp_var = constraint['erp']
                            print(bcolors.OKGREEN + '      erp: ' + str(erp_var) + bcolors.ENDC)
                            p.changeConstraint(constraint_id, erp=erp_var)
                        else:
                            print(bcolors.WARNING + '      erp: not set. Default:1' + bcolors.ENDC)
                            p.changeConstraint(constraint_id, erp=1)
                        if (type == 'gear'):
                            if 'gear_aux_link' in constraint:
                                gear_aux_link = constraint['gear_aux_link']
                                print(bcolors.OKGREEN + '      gear_aux_link: ' + str(gear_aux_link) + bcolors.ENDC)
                                p.changeConstraint(constraint_id, gearAuxLink=link_name_to_index[robot_name][gear_aux_link])
                        else:
                            print(bcolors.WARNING + '      gear_aux_link: not set.' + bcolors.ENDC)
                    if 'max_force' in constraint:
                        max_force = constraint['max_force']
                        print(bcolors.OKGREEN + '      max_force: ' + str(max_force) + bcolors.ENDC)
                        p.changeConstraint(constraint_id, maxForce=max_force)
                    else:
                        print(bcolors.WARNING + '      max_force: not set. Default:100' + bcolors.ENDC)
                        p.changeConstraint(constraint_id, maxForce=100)
            else:
                print(bcolors.WARNING + 'No param /' + robot_name + '/constraints' + bcolors.ENDC)
            if 'joint_control_mode' in robot_info:
                joint_control_mode[robot_name] = robot_info['joint_control_mode']
                print(bcolors.OKGREEN + '  joint_control_mode: ' + joint_control_mode[robot_name] + bcolors.ENDC)
            else:
                print(bcolors.FAIL + 'No param /' + robot_name + '/joint_control_mode' + bcolors.ENDC)
                raise SystemExit

            jt_topic = '/' + robot_name + '/joint_target'
#            rospy.Subscriber(jt_topic, JointState, jointTargetSubscriber, (robot_id[robot_name], joint_name_to_index[robot_name], joint_control_mode[robot_name], controlled_joint_name[robot_name]), queue_size=1)
            rospy.Subscriber(jt_topic, JointState,
                             lambda msg: jointTargetSubscriber(msg,
                                                               robot_id[robot_name],
                                                               joint_name_to_index[robot_name],
                                                               joint_control_mode[robot_name],
                                                               controlled_joint_name[robot_name]),
                             queue_size=1)

            joint_states[robot_name] = p.getJointStates(robot_id[robot_name], range(p.getNumJoints(robot_id[robot_name])))
            for joint_name in joint_name_to_index[robot_name].keys():
                if joint_name in controlled_joint_name[robot_name]:
                    if (joint_control_mode[robot_name] == 'position'):
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.POSITION_CONTROL,
                                                targetPosition=0.0)
                        print(bcolors.OKCYAN + joint_name + ' joint set with position control, target position = 0.0' + bcolors.ENDC)
                    elif (joint_control_mode[robot_name] == 'velocity'):
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.VELOCITY_CONTROL,
                                                targetVelocity=0.0)
                        print(bcolors.OKCYAN + joint_name + ' joint set with velocity control, target velocity = 0.0' + bcolors.ENDC)
                    elif (joint_control_mode[robot_name] == 'torque'):
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.VELOCITY_CONTROL,
                                                force=0.0)
                        p.setJointMotorControl2(robot_id[robot_name],
                                                joint_name_to_index[robot_name][joint_name],
                                                p.TORQUE_CONTROL,
                                                force=0.0)
                        print(bcolors.OKCYAN + joint_name + ' joint set with torque control, target force = 0.0' + bcolors.ENDC)
#                    elif (joint_control_mode[robot_name] == 'pd'):
#                        p.setJointMotorControl2(robot_id[robot_name],
#                                                joint_name_to_index[robot_name][joint_name],
#                                                p.PD_CONTROL)
                    else:
                        print(bcolors.FAIL + '/' + robot_name + '/joint_control_mode not in existing types' + bcolors.ENDC)
                        raise SystemExit
                else:
                    p.setJointMotorControl2(robot_id[robot_name],
                                            joint_name_to_index[robot_name][joint_name],
                                            p.VELOCITY_CONTROL,
                                            force=0.0)
                    print(bcolors.OKGREEN + joint_name + ' joint set with velocity control, max force = 0.0' + bcolors.ENDC)
        else:
            print(bcolors.FAIL + '/' + robot_name + ' param not found' + bcolors.ENDC)
            raise SystemExit

    rospy.Service('change_control_mode', ChangeControlMode, lambda msg: change_control_mode(msg, robot_id, joint_name_to_index, controlled_joint_name, joint_control_mode, joint_effort_limits))

    rate = rospy.Rate(1 / desidered_real_step_time)
    time_pub = rospy.Publisher('/clock', Clock, queue_size=10)
    real_time = rospy.Time.to_sec(rospy.Time.now())

    p.setTimeStep(simulation_step_time)
    p.stepSimulation()                 #
    simulation_time = simulation_step_time
    simulation_time_msg = rospy.Time(simulation_time)
    time_pub.publish(simulation_time_msg)

    js_pub_thread = Thread(target=jointStatePublisher, args=(robot_id, js_publishers, joint_states, joint_state_publish_rate, joint_name_to_index, gear_constraint_to_joint))
    js_pub_thread.start()

    while not rospy.is_shutdown():
        p.stepSimulation()
        simulation_time += simulation_step_time
        simulation_time_msg = rospy.Time(simulation_time)
        time_pub.publish(simulation_time_msg)
        real_step_time = rospy.Time.to_sec(rospy.Time.now()) - real_time
        rate.sleep()
        real_time = rospy.Time.to_sec(rospy.Time.now())
        if (real_step_time > desidered_real_step_time):
            print(bcolors.WARNING + str(real_step_time) + bcolors.ENDC)
        if (real_step_time > simulation_step_time):
            print(bcolors.FAIL + str(real_step_time) + bcolors.ENDC)
    p.disconnect()
    rospy.set_param('use_sim_time', False)

    print(bcolors.OKGREEN + 'Waiting for the thread...' + bcolors.ENDC)
    js_pub_thread.join()


if __name__ == '__main__':
    main()
