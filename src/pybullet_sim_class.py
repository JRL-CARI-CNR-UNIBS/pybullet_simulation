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

class PybulletSim:
    def __init__(
        self,
        node_name: str = '',
        use_moveit: bool = False,
        use_guy: bool = True,
    ) -> None:
        self.node_name = node_name
        self.use_moveit = use_moveit
        self.use_guy = use_guy
        self.control_mode_lock = Lock()
        self.sensor_offset_lock = Lock()
        self.scene_lock = Lock()
        self.objects_lock = Lock()
        self.joint_state_lock = Lock()
        self.joint_target_lock = Lock()

        self.tf_published = False

        self.current_robots_target_configuration = {}
        self.controlled_joint_name = {}
        self.internal_constraint_to_joint = {}
        self.robot_id = {}
        self.objects = {}
        self.link_name_to_index = {}
        self.joint_control_integral_gain = {}
        self.joint_name_to_index = {}
        self.joint_states = {}
        self.joint_target = {}
        self.joint_control_mode = {}
        self.joint_effort_limits = {}
        self.joint_state_publish_rate = None
        self.simulation_step_time = None
        self.desidered_real_step_time = None
        self.state_js = {}
        self.state_id = {}
        self.state_scene = {}
        self.sw_publishers = {}
        self.sensor_offset = {}
        self.scene = None
        self.js_publishers = {}
        self.jt_publishers = {}
        self.jt_subscriber = {}
        self.robots = {}

        rospy.init_node(self.node_name)

        self.pybullet_ns = 'pybullet_simulation'

        environment = {}
        rospy.loginfo('Wait for robot names')
        ready = False
        while not ready:
            if rospy.has_param('/' + self.pybullet_ns):
                if rospy.has_param('/' + self.pybullet_ns + '/robots'):
                    self.robots = rospy.get_param('/' + self.pybullet_ns + '/robots')
                    rospy.loginfo('Robot:')
                    for robot_name in self.robots.keys():
                        rospy.loginfo(' - ' + robot_name)
                        for param_name in self.robots[robot_name].keys():
                            rospy.loginfo('   - ' + param_name)
                if rospy.has_param('/' + self.pybullet_ns + '/environment'):
                    environment = rospy.get_param('/' + self.pybullet_ns + '/environment')
                    rospy.loginfo('Environment:')
                    for environment_part in environment.keys():
                        rospy.loginfo(' - ' + environment_part)
                ready = True


        rospy.loginfo('Topic generated:')
        for robot_name in self.robots.keys():
            js_topic = '/' + robot_name + '/joint_states'
            self.js_publishers[robot_name] = rospy.Publisher('/' + robot_name + '/joint_states', JointState, queue_size=1)
            self.jt_publishers[robot_name] = rospy.Publisher('/' + robot_name + '/joint_target', JointState, queue_size=1)
            rospy.loginfo(' - ' + js_topic)

            if self.use_guy:
                self.p = bc.BulletClient(connection_mode=pybullet.GUI)
            else:
                self.p = bc.BulletClient(connection_mode=pybullet.DIRECT)

            self.p.setAdditionalSearchPath(pybullet_data.getDataPath())
            self.p.loadURDF("plane_transparent.urdf")
            self.p.setGravity(0, 0, -9.8)
            if rospy.has_param('/' + self.pybullet_ns + '/camera_init'):
                camera_init = rospy.get_param('/' + self.pybullet_ns + '/camera_init')
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
                    self.p.resetDebugVisualizerCamera(cameraDistance      =camera_init['distance'],
                                                 cameraYaw           =camera_init['yaw'],
                                                 cameraPitch         =camera_init['pitch'],
                                                 cameraTargetPosition=camera_init['target_position'])

            if rospy.has_param('/' + self.pybullet_ns + '/simulation_step_time'):
                self.simulation_step_time = rospy.get_param('/' + self.pybullet_ns + '/simulation_step_time')
                rospy.loginfo('simulation_step_time: ' + str(self.simulation_step_time))
            else:
                rospy.logerr('No param /simulation_step_time')
                raise SystemExit
            if rospy.has_param('/' + self.pybullet_ns + '/real_step_time'):
                self.desidered_real_step_time = rospy.get_param('/' + self.pybullet_ns + '/real_step_time')
                rospy.loginfo('real_step_time: ' + str(self.desidered_real_step_time))
            else:
                rospy.logerr('No param /real_step_time')
                raise SystemExit
            if rospy.has_param('/' + self.pybullet_ns + '/joint_state_publish_rate'):
                self.joint_state_publish_rate = rospy.get_param('/' + self.pybullet_ns + '/joint_state_publish_rate')
                rospy.loginfo('joint_state_publish_rate: ' + str(self.joint_state_publish_rate))
            else:
                rospy.logerr('No param /joint_state_publish_rate')
                raise SystemExit


            for robot_name in self.robots.keys():
                start_configuration = {}
                rospy.loginfo('For robot ' + robot_name + ':')
                self.internal_constraint_to_joint[robot_name] = {}

                robot_info = self.robots[robot_name]
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
                    start_orientation = self.p.getQuaternionFromEuler(robot_info['start_orientation'])
                else:
                    rospy.logerr('No param /' + robot_name + '/start_orientation')
                    raise SystemExit
                if 'fixed' in robot_info:
                    fixed = robot_info['fixed']
                else:
                    rospy.logerr('No param /' + robot_name + '/fixed')
                    raise SystemExit
                if 'controlled_joint_name' in robot_info:
                    self.controlled_joint_name[robot_name] = robot_info['controlled_joint_name']
                else:
                    rospy.logwarn('No param /' + robot_name + '/controller_joint_name')
                    self.controlled_joint_name[robot_name] = []
                if 'joint_control_integral_gain' in robot_info:
                    self.joint_control_integral_gain[robot_name] = robot_info['joint_control_integral_gain']
                else:
                    rospy.logwarn('No param /' + robot_name + '/joint_control_integral_gain, default value 0.0 for every joint')
                    self.joint_control_integral_gain[robot_name] = numpy.zeros(len(self.controlled_joint_name[robot_name]))
                if 'start_configuration' in robot_info:
                    if (len(self.controlled_joint_name[robot_name]) == len(robot_info['start_configuration'])):
                        for index in range(len(self.controlled_joint_name[robot_name])):
                            start_configuration[self.controlled_joint_name[robot_name][index]] = robot_info['start_configuration'][index]
                    else:
                        rospy.logwarn('start_configuration size is wrong')
                        for index in range(len(self.controlled_joint_name[robot_name])):
                            start_configuration[self.controlled_joint_name[robot_name][index]] = 0.0
                else:
                    rospy.logwarn('No param /' + robot_name + '/start_configuration')
                    for index in range(len(self.controlled_joint_name[robot_name])):
                        start_configuration[self.controlled_joint_name[robot_name][index]] = 0.0

                self.current_robots_target_configuration[robot_name] = start_configuration

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

                self.robot_id[robot_name] = self.p.loadURDF(urdf_path, start_pos, start_orientation, 0, fixed, flags=self.p.URDF_USE_INERTIA_FROM_FILE)
                rospy.loginfo('  robot_id: ' + str(self.robot_id[robot_name]))

                self.link_name_to_index[robot_name] = {self.p.getBodyInfo(self.robot_id[robot_name])[0].decode('UTF-8'): -1, }
                self.joint_name_to_index[robot_name] = {}
                self.joint_effort_limits[robot_name] = {}

                rospy.loginfo('  joint_info: ')
                for joint_id in range(self.p.getNumJoints(self.robot_id[robot_name])):
                    link_name = self.p.getJointInfo(self.robot_id[robot_name], joint_id)[12].decode('UTF-8')
                    self.link_name_to_index[robot_name][link_name] = joint_id
                    joint_name = self.p.getJointInfo(self.robot_id[robot_name], joint_id)[1].decode('UTF-8')
                    self.joint_name_to_index[robot_name][joint_name] = joint_id
                    self.joint_effort_limits[robot_name][joint_name] = self.p.getJointInfo(self.robot_id[robot_name], joint_id)[10]

                    rospy.loginfo('    -jointName: '        + self.p.getJointInfo(self.robot_id[robot_name], joint_id)[1].decode('UTF-8'))
                    rospy.loginfo('     jointIndex: '       + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[0]))
                    rospy.loginfo('     jointType: '        + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[2]))
                    rospy.loginfo('     qIndex: '           + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[3]))
                    rospy.loginfo('     uIndex: '           + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[4]))
                    rospy.loginfo('     flags: '            + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[5]))
                    rospy.loginfo('     jointDamping: '     + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[6]))
                    rospy.loginfo('     jointFriction: '    + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[7]))
                    rospy.loginfo('     jointLowerLimit: '  + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[8]))
                    rospy.loginfo('     jointUpperLimit: '  + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[9]))
                    rospy.loginfo('     jointMaxForce: '    + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[10]))
                    rospy.loginfo('     jointMaxVelocity: ' + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[11]))
                    rospy.loginfo('     linkName: '         + self.p.getJointInfo(self.robot_id[robot_name], joint_id)[12].decode('UTF-8'))
                    rospy.loginfo('     jointAxis: '        + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[13]))
                    rospy.loginfo('     parentFramePos: '   + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[14]))
                    rospy.loginfo('     parentFrameOrn: '   + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[15]))
                    rospy.loginfo('     parentIndex: '      + str(self.p.getJointInfo(self.robot_id[robot_name], joint_id)[16]))

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
                            constraint_type = self.p.JOINT_PRISMATIC
                        elif (param_type == 'fixed'):
                            constraint_type = self.p.JOINT_FIXED
                        elif (param_type == 'point2point'):
                            constraint_type = self.p.JOINT_POINT2POINT
                        elif (param_type == 'gear'):
                            constraint_type = self.p.JOINT_GEAR
                        else:
                            rospy.logerr('Constraint type not foud')
                            raise SystemExit
                        constraint_id = self.p.createConstraint(self.robot_id[parent_body],
                                                           self.link_name_to_index[parent_body][parent_link],
                                                           self.robot_id[child_body],
                                                           self.link_name_to_index[child_body][child_link],
                                                           constraint_type,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                        self.internal_constraint_to_joint[robot_name][constraint_id] = self.p.getJointInfo(self.robot_id[child_body], self.link_name_to_index[child_body][child_link])[1].decode('UTF-8')
                        if (param_type == 'gear'):
                            if 'gear_ratio' in constraint:
                                gear_ratio = constraint['gear_ratio']
                                rospy.loginfo('      gear_ratio: ' + str(gear_ratio))
                                self.p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                            else:
                                rospy.logwarn('      gear_ratio: not set. Default:1')
                                self.p.changeConstraint(constraint_id, gearRatio=1)
                            if 'gear_aux_link' in constraint:
                                gear_aux_link = constraint['gear_aux_link']
                                rospy.loginfo('      gear_aux_link: ' + str(gear_aux_link))
                                self.p.changeConstraint(constraint_id, gearAuxLink=self.link_name_to_index[robot_name][gear_aux_link])
                            else:
                                rospy.logwarn('      gear_aux_link: not set.')
                        if 'erp' in constraint:
                            erp_var = constraint['erp']
                            rospy.loginfo('      erp: ' + str(erp_var))
                        else:
                            rospy.logwarn('      erp: not set. Default:1')
                            erp_var = 1
                        self.p.changeConstraint(constraint_id, erp=erp_var)
                        if 'max_force' in constraint:
                            max_force = constraint['max_force']
                            rospy.loginfo('      max_force: ' + str(max_force))
                        else:
                            rospy.logwarn('      max_force: not set. Default:100')
                            max_force = 100
                        self.p.changeConstraint(constraint_id, maxForce=max_force)
                else:
                    rospy.logwarn('No param /' + robot_name + '/constraints')

                if 'sensors' in robot_info:
                    sensors = robot_info['sensors']
                    if not (isinstance(sensors, list)):
                        rospy.logwarn('Param /' + robot_name + '/sensors is not a list')
                    else:
                        self.sw_publishers[robot_name] = {}
                        self.sensor_offset[robot_name] = {}
                        for sensor in sensors:
                            self.p.enableJointForceTorqueSensor(self.robot_id[robot_name], self.joint_name_to_index[robot_name][sensor], enableSensor=1)
                            self.sw_publishers[robot_name][sensor] = rospy.Publisher('/' + robot_name + '/' + sensor + '/wrench', WrenchStamped, queue_size=1)
                            self.sensor_offset[robot_name][sensor] = [0, 0, 0, 0, 0, 0]
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
                        current_dyn_info = self.p.getDynamicsInfo(self.robot_id[robot_name], self.link_name_to_index[robot_name][link_name])
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
                        self.p.changeDynamics(self.robot_id[robot_name], self.link_name_to_index[robot_name][link_name],
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
                    self.joint_control_mode[robot_name] = robot_info['joint_control_mode']
                    rospy.loginfo('  joint_control_mode: ' + self.joint_control_mode[robot_name])
                else:
                    if (len(self.controlled_joint_name[robot_name]) != 0):
                        rospy.logerr('No param /' + robot_name + '/joint_control_mode')
                        raise SystemExit
                    else:
                        rospy.logwarn('No param /' + robot_name + '/joint_control_mode')
                        self.joint_control_mode[robot_name] = 'Nothing'

                jt_topic = '/' + robot_name + '/joint_target'

                self.joint_state_lock.acquire()
                self.joint_states[robot_name] = self.p.getJointStates(self.robot_id[robot_name], range(self.p.getNumJoints(self.robot_id[robot_name])))
                self.joint_state_lock.release()
                self.joint_target[robot_name] = {}
                for joint_name in self.joint_name_to_index[robot_name].keys():
                    if joint_name in self.controlled_joint_name[robot_name]:
                        self.joint_target[robot_name][joint_name] = {}
                        if (self.joint_control_mode[robot_name] == 'position'):
                            self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                    self.joint_name_to_index[robot_name][joint_name],
                                                    self.p.POSITION_CONTROL,
                                                    targetPosition=start_configuration[joint_name])
                            self.joint_target[robot_name][joint_name]['position'] = start_configuration[joint_name]
                            rospy.loginfo(joint_name + ' joint set with position control, target position = ' + str(start_configuration[joint_name]))
                        elif (self.joint_control_mode[robot_name] == 'velocity'):
                            self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                    self.joint_name_to_index[robot_name][joint_name],
                                                    self.p.VELOCITY_CONTROL,
                                                    targetVelocity=0.0)
                            self.joint_target[robot_name][joint_name]['velocity'] = 0.0
                            rospy.loginfo(joint_name + ' joint set with velocity control, target velocity = 0.0')
                        elif (self.joint_control_mode[robot_name] == 'torque'):
                            self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                    self.joint_name_to_index[robot_name][joint_name],
                                                    self.p.VELOCITY_CONTROL,
                                                    force=0.0)
                            self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                    self.joint_name_to_index[robot_name][joint_name],
                                                    self.p.TORQUE_CONTROL,
                                                    force=0.0)
                            self.joint_target[robot_name][joint_name]['effort'] = 0.0
                            rospy.loginfo(joint_name + ' joint set with torque control, target force = 0.0')
                        else:
                            rospy.logerr('/' + robot_name + '/joint_control_mode not in existing types')
                            raise SystemExit
                    else:
                        self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                self.joint_name_to_index[robot_name][joint_name],
                                                self.p.VELOCITY_CONTROL,
                                                force=0.0)
                        rospy.loginfo(joint_name + ' joint set with velocity control, max force = 0.0')

                self.jt_subscriber[robot_name] = JointTargetSubscriber(self.p,
                                                                  self.joint_target,
                                                                  self.joint_target_lock,
                                                                  robot_name,
                                                                  jt_topic)
                if 'gripper' in robot_info:
                    gripper_jt_topic = '/' + robot_info['gripper'] + '/joint_target'
                    self.jt_subscriber[robot_info['gripper']] = JointTargetSubscriber()

            for env_part in environment.keys():
                start_configuration = {}
                rospy.loginfo('For env_part ' + env_part + ':')
                self.internal_constraint_to_joint[env_part] = {}

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
                    start_orientation = self.p.getQuaternionFromEuler(env_info['start_orientation'])
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
                    self.controlled_joint_name[env_part] = env_info['controlled_joint_name']
                    array_str = '  controlled_joint_name: ['
                    for joint_controlled_name in self.controlled_joint_name[env_part]:
                        array_str += joint_controlled_name + ' '
                    array_str += ']'
                    rospy.loginfo(array_str)
                else:
                    rospy.logwarn('No param /' + env_part + '/controller_joint_name')
                    self.controlled_joint_name[env_part] = []
                if 'joint_control_integral_gain' in env_info:
                    self.joint_control_integral_gain[env_part] = env_info['joint_control_integral_gain']
                    array_str = '  joint_control_integral_gain: ['
                    for integral_gain in self.joint_control_integral_gain[env_part]:
                        array_str += str(integral_gain) + ' '
                    array_str += ']'
                    rospy.loginfo(array_str)
                else:
                    rospy.logwarn('No param /' + env_part + '/joint_control_integral_gain, default value 0.0 for every joint')
                    self.joint_control_integral_gain[env_part] = numpy.zeros(len(self.controlled_joint_name[robot_name]))
                if 'start_configuration' in env_info:
                    if (len(self.controlled_joint_name[env_part]) == len(env_info['start_configuration'])):
                        for index in range(len(self.controlled_joint_name[env_part])):
                            start_configuration[self.controlled_joint_name[env_part][index]] = env_info['start_configuration'][index]
                    else:
                        rospy.logwarn('start_configuration size is wrong')
                        for index in range(len(self.controlled_joint_name[env_part])):
                            start_configuration[self.controlled_joint_name[env_part][index]] = 0.0
                else:
                    rospy.logwarn('No param /' + env_part + '/start_configuration')
                    for index in range(len(self.controlled_joint_name[env_part])):
                        start_configuration[self.controlled_joint_name[env_part][index]] = 0.0

                self.current_robots_target_configuration[env_part] = start_configuration

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

                self.robot_id[env_part] = self.p.loadURDF(urdf_path, start_pos, start_orientation, 0, fixed, flags=self.p.URDF_USE_INERTIA_FROM_FILE)
                rospy.loginfo('  robot_id: ' + str(self.robot_id[env_part]))

                self.link_name_to_index[env_part] = {self.p.getBodyInfo(self.robot_id[env_part])[0].decode('UTF-8'): -1, }
                self.joint_name_to_index[env_part] = {}
                self.joint_effort_limits[env_part] = {}

                rospy.loginfo('  joint_info: ')
                for joint_id in range(self.p.getNumJoints(self.robot_id[env_part])):
                    link_name = self.p.getJointInfo(self.robot_id[env_part], joint_id)[12].decode('UTF-8')
                    self.link_name_to_index[env_part][link_name] = joint_id
                    joint_name = self.p.getJointInfo(self.robot_id[env_part], joint_id)[1].decode('UTF-8')
                    self.joint_name_to_index[env_part][joint_name] = joint_id
                    self.joint_effort_limits[env_part][joint_name] = self.p.getJointInfo(self.robot_id[env_part], joint_id)[10]

                    rospy.loginfo('    -jointName: '        + self.p.getJointInfo(self.robot_id[env_part], joint_id)[1].decode('UTF-8'))
                    rospy.loginfo('     jointIndex: '       + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[0]))
                    rospy.loginfo('     jointType: '        + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[2]))
                    rospy.loginfo('     qIndex: '           + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[3]))
                    rospy.loginfo('     uIndex: '           + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[4]))
                    rospy.loginfo('     flags: '            + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[5]))
                    rospy.loginfo('     jointDamping: '     + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[6]))
                    rospy.loginfo('     jointFriction: '    + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[7]))
                    rospy.loginfo('     jointLowerLimit: '  + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[8]))
                    rospy.loginfo('     jointUpperLimit: '  + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[9]))
                    rospy.loginfo('     jointMaxForce: '    + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[10]))
                    rospy.loginfo('     jointMaxVelocity: ' + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[11]))
                    rospy.loginfo('     linkName: '         + self.p.getJointInfo(self.robot_id[env_part], joint_id)[12].decode('UTF-8'))
                    rospy.loginfo('     jointAxis: '        + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[13]))
                    rospy.loginfo('     parentFramePos: '   + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[14]))
                    rospy.loginfo('     parentFrameOrn: '   + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[15]))
                    rospy.loginfo('     parentIndex: '      + str(self.p.getJointInfo(self.robot_id[env_part], joint_id)[16]))


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
                            constraint_type = self.p.JOINT_PRISMATIC
                        elif (param_type == 'fixed'):
                            constraint_type = self.p.JOINT_FIXED
                        elif (param_type == 'point2point'):
                            constraint_type = self.p.JOINT_POINT2POINT
                        elif (param_type == 'gear'):
                            constraint_type = self.p.JOINT_GEAR
                        else:
                            rospy.logerr('Constraint type not foud')
                            raise SystemExit
                        constraint_id = self.p.createConstraint(self.robot_id[parent_body],
                                                           self.link_name_to_index[parent_body][parent_link],
                                                           self.robot_id[child_body],
                                                           self.link_name_to_index[child_body][child_link],
                                                           constraint_type,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)
                        self.internal_constraint_to_joint[env_part][constraint_id] = self.p.getJointInfo(self.robot_id[child_body], self.link_name_to_index[child_body][child_link])[1].decode('UTF-8')
                        if (param_type == 'gear'):
                            if 'gear_ratio' in constraint:
                                gear_ratio = constraint['gear_ratio']
                                rospy.loginfo('      gear_ratio: ' + str(gear_ratio))
                                self.p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                            else:
                                rospy.logwarn('      gear_ratio: not set. Default:1')
                                self.p.changeConstraint(constraint_id, gearRatio=1)
                            if 'gear_aux_link' in constraint:
                                gear_aux_link = constraint['gear_aux_link']
                                rospy.loginfo('      gear_aux_link: ' + str(gear_aux_link))
                                self.p.changeConstraint(constraint_id, gearAuxLink=self.link_name_to_index[robot_name][gear_aux_link])
                            else:
                                rospy.logwarn('      gear_aux_link: not set.')
                        if 'erp' in constraint:
                            erp_var = constraint['erp']
                            rospy.loginfo('      erp: ' + str(erp_var))
                        else:
                            rospy.logwarn('      erp: not set. Default:1')
                            erp_var = 1
                        self.p.changeConstraint(constraint_id, erp=erp_var)
                        if 'max_force' in constraint:
                            max_force = constraint['max_force']
                            rospy.loginfo('      max_force: ' + str(max_force))
                        else:
                            rospy.logwarn('      max_force: not set. Default:100')
                            max_force = 100
                        self.p.changeConstraint(constraint_id, maxForce=max_force)
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
                        current_dyn_info = self.p.getDynamicsInfo(self.robot_id[env_part], self.link_name_to_index[robot_name][link_name])
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
                        self.p.changeDynamics(self.robot_id[env_part], self.link_name_to_index[env_part][link_name],
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



            if rospy.has_param('/' + self.pybullet_ns + '/external_constraint'):
                rospy.loginfo('External constraints: ')
                constraints = rospy.get_param('/' + self.pybullet_ns + '/external_constraint')
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
                            constraint_type = self.p.JOINT_PRISMATIC
                        elif (param_type == 'fixed'):
                            constraint_type = self.p.JOINT_FIXED
                        elif (param_type == 'point2point'):
                            constraint_type = self.p.JOINT_POINT2POINT
                        elif (param_type == 'gear'):
                            constraint_type = self.p.JOINT_GEAR
                        else:
                            rospy.logerr('Constraint type not foud')
                            raise SystemExit
                        constraint_id = self.p.createConstraint(self.robot_id[parent_body],
                                                           self.link_name_to_index[parent_body][parent_link],
                                                           self.robot_id[child_body],
                                                           self.link_name_to_index[child_body][child_link],
                                                           constraint_type,
                                                           axis,
                                                           parent_frame_position,
                                                           child_frame_position,
                                                           parent_frame_orientation,
                                                           child_frame_orientation)

                        self.internal_constraint_to_joint[robot_name][constraint_id] = self.p.getJointInfo(self.robot_id[child_body], self.link_name_to_index[child_body][child_link])[1].decode('UTF-8')

                        if (param_type == 'gear'):
                            if 'gear_ratio' in constraint:
                                gear_ratio = constraint['gear_ratio']
                                rospy.loginfo('      gear_ratio: ' + str(gear_ratio))
                                self.p.changeConstraint(constraint_id, gearRatio=gear_ratio)
                            else:
                                rospy.logwarn('      gear_ratio: not set. Default:1')
                                self.p.changeConstraint(constraint_id, gearRatio=1)
                            if 'gear_aux_link' in constraint:
                                gear_aux_link = constraint['gear_aux_link']
                                rospy.loginfo('      gear_aux_link: ' + str(gear_aux_link))
                                self.p.changeConstraint(constraint_id, gearAuxLink=self.link_name_to_index[robot_name][gear_aux_link])
                            else:
                                rospy.logwarn('      gear_aux_link: not set.')
                        if 'erp' in constraint:
                            erp_var = constraint['erp']
                            rospy.loginfo('      erp: ' + str(erp_var))
                        else:
                            rospy.logwarn('      erp: not set. Default:1')
                            erp_var = 1
                        self.p.changeConstraint(constraint_id, erp=erp_var)
                        if 'max_force' in constraint:
                            max_force = constraint['max_force']
                            rospy.loginfo('      max_force: ' + str(max_force))
                            self.p.changeConstraint(constraint_id, maxForce=max_force)
                        else:
                            rospy.logwarn('      max_force: not set. Default:100')
                            self.p.changeConstraint(constraint_id, maxForce=100)
                else:
                    rospy.logerr('external_constraint is not a list')
                    raise SystemExit
            else:
                rospy.logwarn('No external constraint')

            rospy.Service('change_control_mode',
                          ChangeControlMode,
                          self.change_control_mode)

            rospy.Service('pybullet_save_state',
                          SaveState,
                          self.save_state)

            rospy.Service('pybullet_restore_state',
                          RestoreState,
                          self.restore_state)

            rospy.Service('pybullet_delete_state',
                          DeleteState,
                          self.delete_state)

            rospy.Service('pybullet_sensor_reset',
                          SensorReset,
                          self.sensor_reset)

            self.scene = PlanningScene()

            obj_tf_pub_thread = Thread(target=self.objects_tf_publisher)
            obj_tf_pub_thread.start()

            rospy.Service('pybullet_spawn_model',
                          SpawnModel,
                          self.spawn_model)
            rospy.Service('pybullet_delete_model',
                          DeleteModel,
                          self.delete_model)
            time_pub = rospy.Publisher('/clock', Clock, queue_size=10)

            self.p.setTimeStep(self.simulation_step_time)
            self.p.stepSimulation()
            simulation_time = []
            simulation_time.append(self.simulation_step_time)
            simulation_time_msg = rospy.Time(simulation_time[0])
            time_pub.publish(simulation_time_msg)

            js_pub_thread = Thread(target=self.joint_state_publisher)
            js_pub_thread.start()

            jt_integ_thread = Thread(target=self.joint_target_integration)
            jt_integ_thread.start()

            sw_pub_thread = Thread(target=self.sensor_wrench_publisher)

            sw_pub_thread.start()

        #    collision_thread = Thread(target=collision_check, args=(p, self.simulation_step_time,))
        #    collision_thread.start()

            time.sleep(0.1)

            rospy.loginfo('Ready for simulation')

            now = time.time()
            while not rospy.is_shutdown():
                self.p.stepSimulation()
                simulation_time[0] += self.simulation_step_time
                simulation_time_msg = rospy.Time(simulation_time[0])
                time_pub.publish(simulation_time_msg)
                elapsed = time.time() - now
                if (self.desidered_real_step_time - elapsed > 0):
                    time.sleep(self.desidered_real_step_time - elapsed)
                now = time.time()

            self.p.disconnect()

            rospy.loginfo('Waiting for threads...')
            js_pub_thread.join()
            obj_tf_pub_thread.join()
            sw_pub_thread.join()
        #    collision_thread.join()


    def joint_target_integration(self):

        robot_names = self.robots.keys()
        pos_compensation = {}
        rate = rospy.Rate(2 / self.simulation_step_time)

        for robot_name in robot_names:
            pos_compensation[robot_name] = {}
            for joint_name in self.joint_name_to_index[robot_name].keys():
                pos_compensation[robot_name][joint_name] = 0.0

        while not rospy.is_shutdown():
            self.control_mode_lock.acquire()
            for robot_name in robot_names:
                target_position_str  = 'target_position:  ['
                joint_state_str      = 'joint state:      ['
                pos_compensation_str = 'pos_compensation: ['
                pos_diff_str         = 'pos_diff:         ['

                for joint_name in self.joint_name_to_index[robot_name].keys():
                    if joint_name in self.controlled_joint_name[robot_name]:
                        if (self.joint_control_mode[robot_name] == 'position'):
                            self.p.setJointMotorControl2(bodyIndex=self.robot_id[robot_name],
                                                    jointIndex=self.joint_name_to_index[robot_name][joint_name],
                                                    controlMode=self.p.POSITION_CONTROL,
                                                    targetPosition=self.joint_target[robot_name][joint_name]['position'] + pos_compensation[robot_name][joint_name])
                        elif (self.joint_control_mode[robot_name] == 'velocity'):
                            self.p.setJointMotorControl2(bodyIndex=self.robot_id[robot_name],
                                                    jointIndex=self.joint_name_to_index[robot_name][joint_name],
                                                    controlMode=self.p.VELOCITY_CONTROL,
                                                    targetVelocity=self.joint_target[robot_name][joint_name]['velocity'])
                        elif (self.joint_control_mode[robot_name] == 'torque'):
                            self.p.setJointMotorControl2(bodyIndex=self.robot_id[robot_name],
                                                    jointIndex=self.joint_name_to_index[robot_name][joint_name],
                                                    controlMode=self.p.TORQUE_CONTROL,
                                                    force=self.joint_target[robot_name][joint_name]['effort'])
                        self.joint_state_lock.acquire()
                        target_position_str = target_position_str + str(round(self.joint_target[robot_name][joint_name]['position'], 5)) + ', '
                        joint_state_str = joint_state_str + str(round(self.joint_states[robot_name][self.joint_name_to_index[robot_name][joint_name]][0], 5)) + ', '
                        pos_diff_str = pos_diff_str + str(round((self.joint_target[robot_name][joint_name]['position'] - self.joint_states[robot_name][self.joint_name_to_index[robot_name][joint_name]][0]), 5)) + ', '
                        pos_compensation[robot_name][joint_name] += self.joint_control_integral_gain[robot_name][self.controlled_joint_name[robot_name].index(joint_name)] * (self.simulation_step_time / 2) * (self.joint_target[robot_name][joint_name]['position'] - self.joint_states[robot_name][self.joint_name_to_index[robot_name][joint_name]][0])
                        pos_compensation_str = pos_compensation_str + str(round(pos_compensation[robot_name][joint_name], 5)) + ', '
                        self.joint_state_lock.release()
                target_position_str = target_position_str + ']'
                joint_state_str = joint_state_str + ']'
                pos_compensation_str = pos_compensation_str + ']'
                pos_diff_str = pos_diff_str + ']'
            self.control_mode_lock.release()

            rate.sleep()


    def change_control_mode(self, srv):
        self.control_mode_lock.acquire()
        for robot_name in srv.robot_names:
            if robot_name in self.robot_id.keys():
                rospy.loginfo(str(self.controlled_joint_name[robot_name]))
                for joint_name in self.controlled_joint_name[robot_name]:
                    self.joint_control_mode[robot_name] = srv.control_mode
                    if (self.joint_control_mode[robot_name] == 'position'):
                        self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                self.joint_name_to_index[robot_name][joint_name],
                                                self.p.POSITION_CONTROL,
                                                force=self.joint_effort_limits[robot_name][joint_name])
                        rospy.loginfo('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'position' + ', max_force: ' + str(self.joint_effort_limits[robot_name][joint_name]))
                    elif (self.joint_control_mode[robot_name] == 'velocity'):
                        self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                self.joint_name_to_index[robot_name][joint_name],
                                                self.p.VELOCITY_CONTROL,
                                                force=self.joint_effort_limits[robot_name][joint_name])
                        rospy.loginfo('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'velocity' + ', max_force: ' + str(self.joint_effort_limits[robot_name][joint_name]))
                    elif (self.joint_control_mode[robot_name] == 'torque'):
                        rospy.logwarn('In torque joint_control')
                        rospy.logwarn('robot_name: ' + robot_name)
                        rospy.logwarn('joint_name: ' + joint_name)
                        self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                self.joint_name_to_index[robot_name][joint_name],
                                                self.p.VELOCITY_CONTROL,
                                                force=0.0)
                        self.p.setJointMotorControl2(self.robot_id[robot_name],
                                                self.joint_name_to_index[robot_name][joint_name],
                                                self.p.TORQUE_CONTROL,
                                                force=0.0)
                        rospy.loginfo('Robot: ' + robot_name + ', Joint: ' + joint_name + ', ' + 'torque' + ', force: 0')
                    else:
                        rospy.logerr(srv.control_mode + ' control mode not exists')
                        self.control_mode_lock.release()
                        return 'false'
            else:
                rospy.logerr(robot_name + ' robot not exists')
                self.control_mode_lock.release()
                return 'false'
        self.control_mode_lock.release()
        return 'true'


    def spawn_model(self, srv):
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
            self.objects_lock.acquire()
            if object_name in self.objects.keys():
                rospy.logerr('Already exists an object with this name')
                self.objects_lock.release()
                return 'false'
            self.objects_lock.release()
            rospy.loginfo('You want to spawn a ' + model_name + ' with the name ' + object_name)

            fixed = 0
            if rospy.has_param('/' + self.pybullet_ns + '/objects/' + model_name):
                model_info = rospy.get_param('/' + self.pybullet_ns + '/objects/' + model_name)

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
                if (self.use_moveit == 'true'):
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
            self.objects_lock.acquire()
            self.objects[object_name] = {}
            self.objects[object_name]['spawned'] = False
            self.objects[object_name]['attached'] = False
            self.objects[object_name]['object_id'] = self.p.loadURDF(urdf_path, start_pos, start_orientation, 0, fixed, flags = self.p.URDF_USE_INERTIA_FROM_FILE)
            self.objects[object_name]['spawned'] = True

            tfl = tf.TransformListener()

            if (self.use_moveit == 'true'):
                mesh = Mesh()
                with pyassimself.p.load(mesh_path) as mesh_file:
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

                self.objects[object_name]['object'] = a_obj

                get_scene_clnt = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
                req = PlanningSceneComponents()
                req.components = sum([PlanningSceneComponents.WORLD_OBJECT_NAMES,
                                      PlanningSceneComponents.WORLD_OBJECT_GEOMETRY,
                                      PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS])
                self.scene_lock.acquire()
                self.scene = get_scene_clnt.call(req).scene
                self.scene.robot_state.joint_state.name     = []
                self.scene.robot_state.joint_state.position = []
                self.scene.robot_state.joint_state.velocity = []
                self.scene.robot_state.joint_state.effort   = []
                self.scene.is_diff = True
                self.scene.robot_state.is_diff = True
                self.scene.world.collision_objects.append(c_obj)
                self.scene_lock.release()
                self.objects[object_name]['attached'] = False

            self.objects_lock.release()

            if (self.use_moveit == 'true'):
                while not (tfl.frameExists(object_name,)):
                    rospy.sleep(0.00001)
        rospy.loginfo('Model spawned')
        return 'true'


    def delete_model(self, srv):
        for object_name in srv.object_name:
            if object_name not in self.objects.keys():
                print(self.objects.keys())
                rospy.logwarn(object_name + ' is not in the scene')
                continue
            id = self.objects[object_name]['object_id']
            if (self.use_moveit == 'true'):
                apply_scene_clnt = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
                if self.objects[object_name]['object'] in self.scene.robot_state.attached_collision_objects:
                    rospy.set_param('/' + object_name + '/attached',False)
                    while self.objects[object_name]['attached'] == True:
                        rospy.sleep(0.1)
                self.scene_lock.acquire()
                if self.objects[object_name]['object'].object in self.scene.world.collision_objects:
                    self.scene.world.collision_objects.remove(self.objects[object_name]['object'].object)
                    self.objects_lock.acquire()
                    self.objects[object_name]['object'].object.operation = self.objects[object_name]['object'].object.REMOVE
                    self.objects_lock.release()
                    self.scene.world.collision_objects.append(self.objects[object_name]['object'].object)
                    apply_scene_clnt.call(self.scene)
                    self.scene.world.collision_objects.remove(self.objects[object_name]['object'].object)
                self.scene_lock.release()
            self.objects_lock.acquire()
            del self.objects[object_name]
            self.objects_lock.release()
            self.p.removeBody(id)
        return 'true'


    def joint_state_publisher(self):
        name = []
        position = []
        velocity = []
        effort = []
        rate = rospy.Rate(self.joint_state_publish_rate)
        js_msg = JointState()
        while not rospy.is_shutdown():
            for robot_name in self.js_publishers.keys():
                self.joint_state_lock.acquire()
                self.joint_states[robot_name] = self.p.getJointStates(self.robot_id[robot_name], range(self.p.getNumJoints(self.robot_id[robot_name])))
                self.joint_state_lock.release()
                name.clear()
                position.clear()
                velocity.clear()
                effort.clear()
                for joint_name in self.joint_name_to_index[robot_name].keys():
                    if joint_name not in self.controlled_joint_name[robot_name]:
                        if joint_name not in self.internal_constraint_to_joint[robot_name].values():
                            continue
                    name.append(joint_name)
                    position.append(self.joint_states[robot_name][self.joint_name_to_index[robot_name][joint_name]][0])
                    velocity.append(self.joint_states[robot_name][self.joint_name_to_index[robot_name][joint_name]][1])
                    if joint_name in self.internal_constraint_to_joint[robot_name].values():
                        constraint_id = list(self.internal_constraint_to_joint[robot_name].keys())[list(self.internal_constraint_to_joint[robot_name].values()).index(joint_name)]
                        effort.append(self.p.getConstraintState(constraint_id)[0])
                    else:
                        effort.append(self.joint_states[robot_name][self.joint_name_to_index[robot_name][joint_name]][3])
                js_msg.header = Header()
                js_msg.header.stamp = rospy.Time.now()
                js_msg.name = name
                js_msg.position = position
                js_msg.velocity = velocity
                js_msg.effort = effort
                self.js_publishers[robot_name].publish(js_msg)
                self.scene_lock.acquire()
                self.scene.robot_state.joint_state = js_msg
                self.scene_lock.release()
            rate.sleep()


    def sensor_wrench_publisher(self):
        rate = rospy.Rate(self.joint_state_publish_rate)
        sw_msg = WrenchStamped()
        while not rospy.is_shutdown():
            self.sensor_offset_lock.acquire()
            for robot_name in self.sw_publishers.keys():
                joint_ids = []
                for joint_name in self.sw_publishers[robot_name].keys():
                    joint_ids.append(self.joint_name_to_index[robot_name][joint_name])
                sensor_wrench = self.p.getJointStates(self.robot_id[robot_name], joint_ids)
                for joint_name in self.sw_publishers[robot_name].keys():
                    index = list(self.sw_publishers[robot_name]).index(joint_name)
                    sw_msg.header = Header()
                    sw_msg.header.stamp = rospy.Time.now()
                    sw_msg.wrench.force.x  = sensor_wrench[index][2][0] - self.sensor_offset[robot_name][joint_name][0]
                    sw_msg.wrench.force.y  = sensor_wrench[index][2][1] - self.sensor_offset[robot_name][joint_name][1]
                    sw_msg.wrench.force.z  = sensor_wrench[index][2][2] - self.sensor_offset[robot_name][joint_name][2]
                    sw_msg.wrench.torque.x = sensor_wrench[index][2][3] - self.sensor_offset[robot_name][joint_name][3]
                    sw_msg.wrench.torque.y = sensor_wrench[index][2][4] - self.sensor_offset[robot_name][joint_name][4]
                    sw_msg.wrench.torque.z = sensor_wrench[index][2][5] - self.sensor_offset[robot_name][joint_name][5]
                    self.sw_publishers[robot_name][joint_name].publish(sw_msg)
            self.sensor_offset_lock.release()
            rate.sleep()


    def sensor_reset(self, srv):
        robot_name = srv.robot_name
        joint_name = srv.joint_name
        if robot_name not in self.sw_publishers:
            rospy.logerr(robot_name + ' do not have any sensor')
            return 'false'
        if joint_name not in self.sw_publishers[robot_name]:
            rospy.logerr('The joint ' + joint_name + ' do not has a sensor')
            rospy.logerr('The sensorized joints are:')
            for sensorized_joint_name in self.sw_publishers[robot_name].keys():
                rospy.logerr('  ' + sensorized_joint_name)
            return 'false'
        self.sensor_offset_lock.acquire()
        sensor_wrench = self.p.getJointStates(self.robot_id[robot_name], [self.joint_name_to_index[robot_name][joint_name]])
        self.sensor_offset[robot_name][joint_name] = [sensor_wrench[0][2][0],
                                                 sensor_wrench[0][2][1],
                                                 sensor_wrench[0][2][2],
                                                 sensor_wrench[0][2][3],
                                                 sensor_wrench[0][2][4],
                                                 sensor_wrench[0][2][5]]
        self.sensor_offset_lock.release()
        return 'true'


    def objects_tf_publisher(self):
        if rospy.has_param('/' + self.pybullet_ns + '/object_tf_publish_rate'):
            tf_publish_rate = rospy.get_param('/' + self.pybullet_ns + '/object_tf_publish_rate')
            rospy.loginfo('object_tf_publish_rate: ' + str(tf_publish_rate))
        else:
            tf_publish_rate = 250
            rospy.logwarn('object_tf_publish_rate not set, default value: ' + str(tf_publish_rate))
        rate = rospy.Rate(tf_publish_rate)
        if (self.use_moveit == 'true'):
            apply_scene_clnt = rospy.ServiceProxy('apply_planning_scene', ApplyPlanningScene)
        br = tf.TransformBroadcaster()
        current_time = rospy.Time.now().to_sec()
        while not rospy.is_shutdown():
            if (current_time != rospy.Time.now().to_sec()):
                self.objects_lock.acquire()
                for object_name in self.objects:
                    if 'object_id' in self.objects[object_name]:
                        pose = self.p.getBasePositionAndOrientation(self.objects[object_name]['object_id'])
                        br.sendTransform((pose[0][0], pose[0][1], pose[0][2]),
                                         (pose[1][0], pose[1][1], pose[1][2], pose[1][3]),
                                         rospy.Time.now(),
                                         object_name,
                                         "world")
                        if ((self.use_moveit == 'true') and self.objects[object_name]['spawned']):
                            self.scene_lock.acquire()

                            if (rospy.has_param('/' + object_name + '/attached')):
                                if (rospy.has_param('/' + object_name + '/attached_link')):
                                    if (rospy.has_param('/' + object_name + '/touch_links')):
                                        if(rospy.get_param('/' + object_name + '/attached')):
                                            attached_link = rospy.get_param('/' + object_name + '/attached_link')
                                            touch_links = rospy.get_param('/' + object_name + '/touch_links')
                                            self.objects[object_name]['object'].link_name = attached_link
                                            self.objects[object_name]['object'].touch_links = touch_links
                                            if not self.objects[object_name]['attached']:
                                                rospy.logerr("Attach")
                                                self.scene.robot_state.attached_collision_objects.append(self.objects[object_name]['object'])
                                                self.scene.world.collision_objects.remove(self.objects[object_name]['object'].object)
                                                rospy.loginfo('Added attached obj')
                                                self.objects[object_name]['attached'] = True
                                        else:
                                            if self.objects[object_name]['attached']:
                                                if self.objects[object_name]['object'] in self.scene.robot_state.attached_collision_objects:
                                                    rospy.logerr("Deattach")
                                                    self.scene.robot_state.attached_collision_objects.remove(self.objects[object_name]['object'])
                                                    self.objects[object_name]['object'].object.operation = self.objects[object_name]['object'].object.REMOVE
                                                    self.scene.robot_state.attached_collision_objects.append(self.objects[object_name]['object'])
                                                    apply_scene_clnt.call(self.scene)
                                                    self.scene.robot_state.attached_collision_objects.remove(self.objects[object_name]['object'])
                                                    self.objects[object_name]['object'].object.operation = self.objects[object_name]['object'].object.ADD
                                                    self.scene.world.collision_objects.append(self.objects[object_name]['object'].object)
                                                    rospy.loginfo('Removed attached obj')
                                                    self.objects[object_name]['attached'] = False
                            apply_scene_clnt.call(self.scene)
                            self.scene_lock.release()
                self.objects_lock.release()
                current_time = rospy.Time.now().to_sec()
            self.tf_published = True
            rate.sleep()


    def save_state(self, srv):
        if not srv.state_name:
            rospy.logerr('Name is empty')
            return 'false'
        if srv.state_name in self.state_id.keys():
            rospy.logerr(srv.state_name + ', a state whit this name already exists')
            return 'false'
        self.joint_state_lock.acquire()
        self.state_id[srv.state_name] = self.p.saveState()
        self.state_js[srv.state_name] = {}
        self.state_js[srv.state_name] = copy.copy(self.joint_states)
        self.joint_state_lock.release()
        self.scene_lock.acquire()
        self.state_scene[srv.state_name] = {}
        self.state_scene[srv.state_name] = copy.copy(self.scene)
        self.scene_lock.release()
        rospy.loginfo('state ' + srv.state_name + ' saved')
        return 'true'


    def restore_state(self, srv):
        self.objects_lock.acquire()
        if not srv.state_name:
            rospy.logerr('Name is empty')
            return 'false'

        if srv.state_name not in self.state_id.keys():
            rospy.logerr(srv.state_name + ', no state with this name')
            return 'false'

        jt_msg = JointState()
        name = []
        position = []
        velocity = []
        effort = []


        for robot_name in self.state_js[srv.state_name]:
            name.clear()
            position.clear()
            velocity.clear()
            effort.clear()
            for joint_name in self.joint_name_to_index[robot_name]:
                name.append(joint_name)
                position.append(self.state_js[srv.state_name][robot_name][self.joint_name_to_index[robot_name][joint_name]][0])
                velocity.append(self.state_js[srv.state_name][robot_name][self.joint_name_to_index[robot_name][joint_name]][1])
                effort.append(self.state_js[srv.state_name][robot_name][self.joint_name_to_index[robot_name][joint_name]][3])
            jt_msg.header = Header()
            jt_msg.header.stamp = rospy.Time.now()
            jt_msg.name = name
            jt_msg.position = position
            jt_msg.velocity = velocity
            jt_msg.effort = effort
            self.jt_publishers[robot_name].publish(jt_msg)
        self.p.restoreState(self.state_id[srv.state_name])

        rospy.loginfo('state ' + srv.state_name + ' restored')
        self.tf_published = False
        self.objects_lock.release()

        while not self.tf_published:
            rospy.sleep(0.1)
        return 'true'


    def delete_state(self, srv):
        if not srv.state_name:
            rospy.logwarn('Name list is empty')
            return 'true'
        for state_name in srv.state_name:
            if state_name not in self.state_id.keys():
                rospy.logwarn(state_name + ', no state with this name')
            else:
                self.p.removeState(self.state_id[state_name])
                self.state_id.pop(state_name)
        return 'true'


    def collision_check(self):
        rate = rospy.Rate(2 / self.simulation_step_time)
        number_normal_contact = len(self.p.getContactPoints())

        while not rospy.is_shutdown():
            contact_points = self.p.getContactPoints()

            if (len(contact_points) > number_normal_contact):
                print('Collision')
            elif (len(contact_points) < number_normal_contact):
                number_normal_contact = len(contact_points)
                print('New number_normal_contact')

            rate.sleep()

