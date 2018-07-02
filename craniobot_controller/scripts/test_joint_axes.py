#!/usr/bin/env python
#import roslib
#roslib.load_manifest('cool400_ros_experimental')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, motor_name):
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

        # Here the configuration is such that the parallel servos in joint2 have the names 
        # joint2(with servo id: 1) and and again joint2(with servo id: 2). The developer while commanding 
        # always needs to make sure that they always have the same values in the second and the third position.
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            goal.trajectory.joint_names = ['joint1','joint2','joint3','joint4','joint5','joint6']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(6)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              


def main():
            arm = Joint('cool400_trajectory')
            #Home Position
            arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
            arm.move_joint([2.5,0.0,0.0,0.0,0.0,0.0])
            arm.move_joint([-2.5,0.0,0.0,0.0,0.0,0.0])
            arm.move_joint([0.0,0.8,0.0,0.0,0.0,0.0])
            arm.move_joint([0.0,-0.8,0.0,0.0,0.0,0.0])
            arm.move_joint([0.0,0.0,2.5,0.0,0.0,0.0])
            arm.move_joint([0.0,0.0,-2.5,0.0,0.0,0.0])
            arm.move_joint([0.0,0.0,0.0,1.57,0.0,0.0])
            arm.move_joint([0.0,0.0,0.0,-1.57,0.0,0.0])
            arm.move_joint([0.0,0.0,0.0,0.0,1.57,0.0])
            arm.move_joint([0.0,0.0,0.0,0.0,-1.57,0.0])
            arm.move_joint([0.0,0.0,0.0,0.0,0.0,3.0])
            arm.move_joint([0.0,0.0,0.0,0.0,0.0,-3.0])
            arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
            
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()

