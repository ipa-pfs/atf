#!/usr/bin/python
import rospy
import rospkg
import smach
import smach_ros
import tf
import sys
import copy

import random

from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import RobotState, AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from simple_script_server import *
sss = simple_script_server()
active_arm = 'right'

def move_gripper(component_name, pos):
    error_code = -1
    counter = 0
    while not rospy.is_shutdown() and error_code != 0:
        print "trying to move", component_name, "to", pos, "retries: ", counter
        handle = sss.move(component_name, pos)
        handle.wait()
        error_code = handle.get_error_code()
        if counter > 100:
            rospy.logerr(component_name + "does not work any more. retries: " + str(counter) + ". Please reset USB connection and press <ENTER>.")
            sss.wait_for_input()
            return False
        counter += 1
    return True

class RotateCS(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded'],
            input_keys=['active_arm'])
        self.angle_offset_yaw = 0

        if active_arm == "left":
            self.angle_offset_roll = math.pi
            self.rose_position_y = 0.3
            self.direction_yaw = -1
        elif active_arm == "right":
            self.angle_offset_roll = 0
            self.rose_position_y = -0.3
            self.direction_yaw = 1
        
        rospy.Timer(rospy.Duration(0.1), self.broadcast_tf)
        self.br = tf.TransformBroadcaster()
        
    def broadcast_tf(self, event):
        self.br.sendTransform(
            (0.6, self.rose_position_y, 0.8),
            tf.transformations.quaternion_from_euler(self.angle_offset_roll, 0, self.angle_offset_yaw),
            event.current_real,
            "current_rose",
            "base_link")

    def execute(self, userdata):
        if self.angle_offset_yaw >= 0.5 * math.pi:
            self.direction_yaw = -1
        elif self.angle_offset_yaw <= -0.5 * math.pi:
            self.direction_yaw = 1

        if self.direction_yaw == 1:
            self.angle_offset_yaw += 5.0 / 180.0 * math.pi
        elif self.direction_yaw == -1:
            self.angle_offset_yaw -= 5.0 / 180.0 * math.pi

        if userdata.active_arm == "left":
            self.angle_offset_roll = math.pi
            self.rose_position_y = 0.3
        elif userdata.active_arm == "right":
            self.angle_offset_roll = 0
            self.rose_position_y = -0.3
        return "succeeded"

class Grasping(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','failed'],
            input_keys=['active_arm'])

        # initialize tf listener
        self.listener = tf.TransformListener()

        ### Create a handle for the Move Group Commander
        self.mgc_left = MoveGroupCommander("arm_left")
        self.mgc_right = MoveGroupCommander("arm_right")

        self.traj_pre_grasp = RobotTrajectory()
        self.traj_retreat = RobotTrajectory()

        ### Create a handle for the Planning Scene Interface
        #self.psi = PlanningSceneInterface()

        self.eef_step = 0.01
        self.jump_threshold = 2

        self.pre_grasp = False
        self.retreat = False

        rospy.sleep(1)

    def execute(self, userdata):

        if not self.plan_and_execute(userdata):
            return "failed"

        return "succeeded"

    ### Todo: Fix start pose for retreat
    def plan_movement(self, arm, pose):
        (config, error_code) = sss.compose_trajectory("arm_" + arm, pose)
        if error_code != 0:
            rospy.logerr("unable to parse " + pose + " configuration")

        start_state = RobotState()
        start_state.joint_state.name = config.joint_names
        if arm == "left":
            start_state.joint_state.position = self.mgc_left.get_current_joint_values()
            self.mgc_left.set_start_state(start_state)

            self.mgc_left.clear_pose_targets()
            self.mgc_left.set_joint_value_target(config.points[0].positions)

            plan = self.mgc_left.plan()

        elif arm == "right":
            start_state.joint_state.position = self.mgc_right.get_current_joint_values()
            self.mgc_right.set_start_state(start_state)

            self.mgc_right.clear_pose_targets()
            self.mgc_right.set_joint_value_target(config.points[0].positions)

            plan = self.mgc_right.plan()

        else:
            rospy.logerr("invalid arm_active")

        plan = self.smooth_cartesian_path(plan)
        plan = self.scale_joint_trajectory_speed(plan, 0.3)
        return plan

    def plan_and_execute(self, userdata):

        while not self.pre_grasp:
            try:
                self.traj_pre_grasp = self.plan_movement(userdata.active_arm, "pre_grasp")
            except (ValueError,IndexError):
                self.pre_grasp = False
            else:
                self.pre_grasp = True
                rospy.loginfo("pre_grasp")
                if userdata.active_arm == "left":
                    self.mgc_left.execute(self.traj_pre_grasp)
                elif userdata.active_arm == "right":
                    self.mgc_right.execute(self.traj_pre_grasp)

        ### Set next (virtual) start state
        start_state = RobotState()
        (pre_grasp_config, error_code) = sss.compose_trajectory("arm_" + userdata.active_arm, "pre_grasp")
        if error_code != 0:
            rospy.logerr("unable to parse pre_grasp configuration")
            return "failed"
        start_state.joint_state.name = pre_grasp_config.joint_names
        start_state.joint_state.position = pre_grasp_config.points[0].positions
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Approach
        approach_pose_offset = PoseStamped()
        approach_pose_offset.header.frame_id = "current_rose"
        approach_pose_offset.header.stamp = rospy.Time(0)
        approach_pose_offset.pose.position.x = -0.12
        approach_pose_offset.pose.orientation.w = 1
        try:
            approach_pose = self.listener.transformPose("odom_combined", approach_pose_offset)
        except Exception, e:
            rospy.logerr("could not transform pose. Exception: %s", str(e))
            return False

        if userdata.active_arm == "left":
            (traj_approach,frac_approach) = self.mgc_left.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_approach,frac_approach) = self.mgc_right.compute_cartesian_path([approach_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False

        traj_approach = self.smooth_cartesian_path(traj_approach)

        print frac_approach

        if not (frac_approach == 1.0):
            rospy.logerr("Unable to plan approach trajectory")
            return False

        ### Set next (virtual) start state
        traj_approach_endpoint = traj_approach.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_approach.joint_trajectory.joint_names
        start_state.joint_state.position = traj_approach_endpoint.positions
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Grasp
        grasp_pose_offset = PoseStamped()
        grasp_pose_offset.header.frame_id = "current_rose"
        grasp_pose_offset.header.stamp = rospy.Time(0)
        grasp_pose_offset.pose.orientation.w = 1
        grasp_pose = self.listener.transformPose("odom_combined", grasp_pose_offset)
        if userdata.active_arm == "left":
            (traj_grasp,frac_grasp) = self.mgc_left.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_grasp,frac_grasp) = self.mgc_right.compute_cartesian_path([grasp_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False

        traj_grasp = self.smooth_cartesian_path(traj_grasp)

        print frac_grasp

        if not (frac_grasp == 1.0):
            rospy.logerr("Unable to plan grasp trajectory")
            return False

        ### Set next (virtual) start state
        traj_grasp_endpoint = traj_grasp.joint_trajectory.points[-1]
        start_state = RobotState()
        start_state.joint_state.name = traj_grasp.joint_trajectory.joint_names
        start_state.joint_state.position = traj_grasp_endpoint.positions
        start_state.is_diff = True
        if userdata.active_arm == "left":
            self.mgc_left.set_start_state(start_state)
        elif userdata.active_arm == "right":
            self.mgc_right.set_start_state(start_state)
        else:
            rospy.logerr("invalid arm_active")
            return False

        ### Plan Lift
        lift_pose_offset = PoseStamped()
        lift_pose_offset.header.frame_id = "current_rose"
        lift_pose_offset.header.stamp = rospy.Time(0)
        if userdata.active_arm == "left":
            lift_pose_offset.pose.position.z = -0.2
        elif userdata.active_arm == "right":
            lift_pose_offset.pose.position.z = 0.2
        else:
            rospy.logerr("invalid active_arm: %s", userdata.active_arm)
            sys.exit()
        lift_pose_offset.pose.orientation.w = 1
        lift_pose = self.listener.transformPose("odom_combined", lift_pose_offset)

        if userdata.active_arm == "left":
            (traj_lift,frac_lift) = self.mgc_left.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        elif userdata.active_arm == "right":
            (traj_lift,frac_lift) = self.mgc_right.compute_cartesian_path([lift_pose.pose], self.eef_step, self.jump_threshold, True)
        else:
            rospy.logerr("invalid arm_active")
            return False

        traj_lift = self.smooth_cartesian_path(traj_lift)

        print frac_lift

        if not (frac_lift == 1.0):
            rospy.logerr("Unable to plan lift trajectory")
            return False

        if not (frac_approach == 1.0 and frac_grasp == 1.0 and frac_lift == 1.0):
            rospy.logerr("Unable to plan whole grasping trajectory")
            return False
        else:

            traj_approach = self.fix_velocities(traj_approach)
            traj_grasp = self.fix_velocities(traj_grasp)
            traj_lift = self.fix_velocities(traj_lift)

            ### execute
            if userdata.active_arm == "left":
                rospy.loginfo("approach")
                self.mgc_left.execute(traj_approach)
                move_gripper("gripper_" + userdata.active_arm, "open")
                rospy.loginfo("grasp")
                self.mgc_left.execute(traj_grasp)
                move_gripper("gripper_" + userdata.active_arm, "close")
                rospy.loginfo("lift")
                self.mgc_left.execute(traj_lift)
            elif userdata.active_arm == "right":
                rospy.loginfo("approach")
                self.mgc_right.execute(traj_approach)
                move_gripper("gripper_" + userdata.active_arm, "open")
                rospy.loginfo("grasp")
                self.mgc_right.execute(traj_grasp)
                move_gripper("gripper_" + userdata.active_arm, "close")
                rospy.loginfo("lift")
                self.mgc_right.execute(traj_lift)
            else:
                rospy.logerr("invalid arm_active")
                return False

        ### Plan Retreat
        while not self.retreat:
            try:
                self.traj_retreat = self.plan_movement(userdata.active_arm, "retreat")
            except (ValueError,IndexError):
                self.retreat = False
            else:
                self.retreat = True
                rospy.loginfo("retreat")
                if userdata.active_arm == "left":
                    self.mgc_left.execute(self.traj_retreat)
                elif userdata.active_arm == "right":
                    self.mgc_right.execute(self.traj_retreat)

        return True

    def smooth_cartesian_path(self, traj):

        time_offset = 0.2

        for i in range(len(traj.joint_trajectory.points)):
            traj.joint_trajectory.points[i].time_from_start += rospy.Duration(time_offset)

        traj.joint_trajectory.points[-1].time_from_start += rospy.Duration(time_offset)

        return traj

    def fix_velocities(self, traj):
        # fix trajectories to stop at the end
        traj.joint_trajectory.points[-1].velocities = [0]*7

        # fix trajectories to be slower
        speed_factor = 1.0
        for i in range(len(traj.joint_trajectory.points)):
            traj.joint_trajectory.points[i].time_from_start *= speed_factor

        return traj

    def scale_joint_trajectory_speed(self, traj, scale):
           # Create a new trajectory object
           new_traj = RobotTrajectory()

           # Initialize the new trajectory to be the same as the planned trajectory
           new_traj.joint_trajectory = traj.joint_trajectory

           # Get the number of joints involved
           n_joints = len(traj.joint_trajectory.joint_names)

           # Get the number of points on the trajectory
           n_points = len(traj.joint_trajectory.points)

           # Store the trajectory points
           points = list(traj.joint_trajectory.points)

           # Cycle through all points and scale the time from start, speed and acceleration
           for i in range(n_points):
               point = JointTrajectoryPoint()
               point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
               point.velocities = list(traj.joint_trajectory.points[i].velocities)
               point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
               point.positions = traj.joint_trajectory.points[i].positions

               for j in range(n_joints):
                   point.velocities[j] = point.velocities[j] * scale
                   point.accelerations[j] = point.accelerations[j] * scale * scale

               points[i] = point

           # Assign the modified points to the new trajectory
           new_traj.joint_trajectory.points = points

           # Return the new trajecotry
           return new_traj


class SwitchArm(smach.State):
    def __init__(self):
        smach.State.__init__(self,
            outcomes=['succeeded','finished'],
            input_keys=['active_arm'],
            output_keys=['active_arm'])

        self.counter = 1

    def execute(self, userdata):
        if self.counter == 2:
            return "finished"
        else:
            if userdata.active_arm == "left":
                userdata.active_arm = "right"
            elif userdata.active_arm == "right":
                userdata.active_arm = "left"
            self.counter += 1.0
            return "succeeded"


class SM(smach.StateMachine):
    def __init__(self):        
        smach.StateMachine.__init__(self,outcomes=['ended'])

        self.userdata.active_arm = active_arm
        
        with self:
            smach.StateMachine.add('GRASP',Grasping(),
                transitions={'succeeded':'SWITCH_ARM',
                    'failed':'ROTATE_COORDINATESYSTEM'})
            smach.StateMachine.add('ROTATE_COORDINATESYSTEM',RotateCS(),
                transitions={'succeeded':'GRASP'})
            smach.StateMachine.add('SWITCH_ARM',SwitchArm(),
                transitions={'succeeded':'ROTATE_COORDINATESYSTEM',
                    'finished':'ended'})

if __name__=='__main__':
    rospy.init_node('roses')    
    sm = SM()
    sis = smach_ros.IntrospectionServer('sm', sm, 'SM')
    sis.start()
    outcome = sm.execute()
    rospy.spin()
    sis.stop()
