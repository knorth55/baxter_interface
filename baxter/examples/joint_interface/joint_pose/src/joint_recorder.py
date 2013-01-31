#!/usr/bin/env python
import signal
import argparse

from GripperBaxterController import GripperBaxterController
from sensor_msgs.msg import JointState

import roslib
roslib.load_manifest('joint_pose')
import rospy

class JointRecorder(object):

  def __init__(self, filename, rate):
    self._filename = filename
    self._rate = rospy.Rate(rate)
    self._startTime = rospy.Time.now()
    self._leftPosition = {}
    self._rightPosition = {}
    self._done = False

    #TODO: factor out arm interfaces and reuse in JointPositionBaxterController
    self._subLeft = rospy.Subscriber('/robot/limb/left/joint_states', JointState, self.leftJointState)
    self._subRight = rospy.Subscriber('/robot/limb/right/joint_states', JointState, self.rightJointState)
    self._gripperLeft = GripperBaxterController('left')
    self._gripperRight = GripperBaxterController('right')

    signal.signal(signal.SIGINT, self.handleCtrlC)
    self.waitForData()

  def handleCtrlC(self, signum, frame):
    """ ctrl-c handler
    """
    self.stop()

  def stop(self):
    self._done = True

  def done(self):
    return self._done or rospy.is_shutdown()

  def leftJointState(self, data):
    """ callback function for the ROS subscriber of the left joint angles
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      key = 'left_'+data.name[i]
      self._leftPosition[key] = data.position[i]

  def rightJointState(self, data):
    """ callback function for the ROS subscriber of the right joint angles
    Args:
      data(sensor_msgs.msg.JointState): the ROS message containing the joint state
    """
    for i in range(len(data.name)):
      key = 'right_'+data.name[i]
      self._rightPosition[key] = data.position[i]

  def timeStamp(self):
    diff = rospy.Time.now() - self._startTime
    return diff.to_sec()

  def waitForData(self):
    rate = rospy.Rate(100)
    while not self.done():
      if len(self._leftPosition.keys()) and len(self._rightPosition.keys()):
        return True
      rate.sleep()
    return False

  def record(self):
    """ Records the current joint positions to a csv file
    if outputFilename was provided at construction
    this function will record the latest set of joint angles
    in a csv format.
    This function does not test to see if a file exists and
    will overwrite existing files.
    """
    if self._filename:
      with open(self._filename, 'w') as f:
        f.write('time,')
        f.write(','.join(self._leftPosition.keys()) + ',')
        f.write('left_gripper,')
        f.write(','.join(self._rightPosition.keys()) + ',')
        f.write('right_gripper\n')

        while not self.done():
          lVals = self._leftPosition.values()
          rVals = self._rightPosition.values()

          f.write("%f," % (self.timeStamp(),))

          f.write(','.join([str(x) for x in lVals]) + ',')
          f.write(str(self._gripperLeft.position) + ',')

          f.write(','.join([str(x) for x in rVals]) + ',')
          f.write(str(self._gripperRight.position) + '\n')

          self._rate.sleep()

if __name__ == '__main__':
  parser = argparse.ArgumentParser()
  parser.add_argument("filename", help="the file name to record to")
  parser.add_argument("-r", "--record-rate", dest="recordRate", type=int, default=10, help="rate at which to record")
  args = parser.parse_args()

  print("Initializing node... ")
  rospy.init_node("rethink_rsdk_joint_recorder")
  recorder = JointRecorder(args.filename, args.recordRate)
  print("recording...")
  recorder.record()
  print("done.")