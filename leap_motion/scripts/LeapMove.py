#!/usr/bin/env python
import sys
import moveit_commander
import rospy
from leap_motion.msg import Human
from leap_motion.msg import Hand
import geometry_msgs.msg
from moveit_commander.conversions import pose_to_list

def all_close(goal, actual, tolerance):
  all_equal = True

  if type(goal) is list:
    for i in range(len(goal)):
      if abs(actual[i] - goal[i])>tolerance:
        return False

  if type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
  
  return True


class LeapMove(object):

  def __init__(self):
    super(LeapMove, self).__init__()

    ## First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('LeapMove', anonymous=True)
    robot = moveit_commander.RobotCommander()

    # Misc variables
    self.subscriber = None
    self.arm = moveit_commander.MoveGroupCommander("panda_arm")
    self.waiting = False
    self.human = Human()
    self.default_pose = None
    subscriber = rospy.Subscriber("/leap_motion/leap_filtered", Human, self.callback, queue_size=1, buff_size=52428800)

    #ready and spin
    self.waiting = True
    rospy.spin()


  #callback every time recieve a new leap message
  def callback(self, data):
    #gets the right hand data
    self.human = data.right_hand

    #get default pose on first run
    if(self.default_pose == None):
      self.default_pose = self.arm.get_current_pose("panda_link8")

    #goto pose if ready
    if self.waiting:
      self.waiting = False
      
      if(self.human.palm_center.x != 0 or self.human.palm_center.y != 0 or self.human.palm_center.z!= 0):
        self.go_to_pose_goal()

      self.waiting = True


  #move the arm to the right hands location
  def go_to_pose_goal(self):     
    arm_pose_goal = geometry_msgs.msg.Pose()
    arm_pose_goal.orientation = self.default_pose.pose.orientation
    arm_pose_goal.position.x = self.human.palm_center.x *2
    arm_pose_goal.position.z = self.human.palm_center.y *2
    arm_pose_goal.position.y = -1*self.human.palm_center.z *2

    print "Moving to:", arm_pose_goal.position.x, arm_pose_goal.position.y, arm_pose_goal.position.z

    self.arm.set_pose_target(arm_pose_goal, "panda_link8")
    self.arm.go(wait=True)
    self.arm.stop()
    self.arm.clear_pose_targets()

    current_arm_pose = self.arm.get_current_pose("panda_link8").pose

    #return
    return all_close(arm_pose_goal, current_arm_pose, 0.01)


def main():
  try:
    lm = LeapMove()
    print ("Script complete")
    raw_input()

  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
