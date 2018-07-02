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
            rospy.loginfo('Waiting for joint trajectory action server')
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
            point.time_from_start = rospy.Duration(3)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              


def main():
            arm = Joint('craniobot_trajectory')
            #Home Position
            #arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
            #arm.move_joint([0.035, -0.82, 1.99, 0.253, 1.860, -0.46])
            #arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
            #arm.move_joint([0.00, -0.85, 1.99, 0.44, 1.61, -0.51])
            #arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
            #arm.move_joint([0.138, -0.92, 0.00, -0.64, 0.11, 1.28])
            #arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
            # Other tests
            #arm.move_joint([0.5,1.5,1.5,1.0,1.5,1.0])
            #arm.move_joint([6.28,3.14,6.28,3.14,6.28,3.14])
            #arm.move_joint([0.5,0.0,0.5,0.5,0.5,0.5])
            #arm.move_joint([1.0,0.0,1.0,1.0,1.0,1.0])
            #arm.move_joint([1.5,0.0,1.5,1.5,1.5,1.5])
            #arm.move_joint([2.0,0.0,2.0,2.0,2.0,2.0])
            #arm.move_joint([0.0,-1.5,0.0,0.0,0.0,0.0])
            #arm.move_joint([1.0,0.0,1.0,1.0,1.0,1.0])
            #arm.move_joint([0.5,1.5,1.5,1.0,1.5,1.0])
            #arm.move_joint([1.5,1.1,1.5,0.5,1.2,2.0])
            #arm.move_joint([0.0,0.0,0.0,0.0,0.0,0.0])
            
            #Samsung states
            arm.move_joint([0.015339807878856412, 1.722660424795575, -0.01687378866674205, -0.47093210188089185, 1.517106999218899, -2.1859226227370385])
            arm.move_joint([2.633845012799646, -1.4419419406125027, -1.5232429223704418, -0.5506991028509451, 1.483359421885415, 1.2])
            arm.move_joint([2.6353789935875316, -1.5017671913400428, -1.5201749607946704, -0.32673790781964157, 1.4664856332186729, 1.6183497312193513])
            arm.move_joint([2.0, -1.3775147475213059, -1.526310883946213, -0.059825250727540004, 1.359106978066678, 2.5])
            arm.move_joint([2.158310968555097, -1.3008157081270237, -0.5, -0.5, 1.9389517158874505, 2.9007576698917474])
            arm.move_joint([1.59073807703741, -1.55545651891604, -1.6474953661891787, -0.16106798272799233, 1.9941750242513334, 2.8976897083159763])
            arm.move_joint([1.5677283652191252, -1.3928545554001621, -2.710544052193928, -0.5399612373357456, 1.5661943844312396, 2.0])
            arm.move_joint([1.191903072187143, -1.573864288370668, -1.1596894756415448, 0.21168934872821848, 1.560058461279697, -1.9481556006147642])
            arm.move_joint([1.6198837120072371, -1.5293788455219843, 1.7840196563110007, -0.18867963690993386, 0.8114758367915041, 2.8992236891038616])
            arm.move_joint([0.28225246497095796, -1.5615924420675826, -1.5922720578252956, -0.4111068511533518, 1.5, -1.6])
            arm.move_joint( [-0.009203884727313847, 0.06289321230331128, 1.5938060386131812, 0.007669903939428206, 0.023009711818284616, 1.5186409800067848])
            arm.move_joint([1.5631264228554684, -0.7992039904884191, 1.5846021538858672, -0.023009711818284616, -1.4618836908550161, -0.010737865515199488])   
            arm.move_joint([0.015339807878856412, 1.722660424795575, -0.01687378866674205, -0.47093210188089185, 1.517106999218899, -2.1859226227370385])
                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()

