#!/usr/bin/env python
import rospy
import sys
import csv
import os

from quest2ros.msg import OVR2ROSInputs, OVR2ROSHapticFeedback
from geometry_msgs.msg import PoseStamped, Twist 
import numpy as np 



class ros2quest:

  def __init__(self):

    #subscriber to quest
    self.ovr2ros_right_hand_pose_sub = rospy.Subscriber("/q2r_right_hand_pose",PoseStamped,self.ovr2ros_right_hand_pose_callback)
    self.ovr2ros_right_hand_twist_sub = rospy.Subscriber("/q2r_right_hand_twist",Twist,self.ovr2ros_right_hand_twist_callback)
    self.ovr2ros_right_hand_inputs_sub = rospy.Subscriber("/q2r_right_hand_inputs",OVR2ROSInputs,self.ovr2ros_right_hand_inputs_callback)
    self.ovr2ros_left_hand_pose_sub = rospy.Subscriber("/q2r_left_hand_pose",PoseStamped,self.ovr2ros_left_hand_pose_callback)
    self.ovr2ros_left_hand_twist_sub = rospy.Subscriber("/q2r_left_hand_twist",Twist,self.ovr2ros_left_hand_twist_callback)
    self.ovr2ros_left_hand_inputs_sub = rospy.Subscriber("/q2r_left_hand_inputs",OVR2ROSInputs,self.ovr2ros_left_hand_inputs_callback)
    
    #Puplisher to quest 
    self.ros2ovr_right_hand_haptic_feedback_pub = rospy.Publisher("/q2r_right_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
    self.ros2ovr_left_hand_haptic_feedback_pub = rospy.Publisher("/q2r_left_hand_haptic_feedback", OVR2ROSHapticFeedback, queue_size=1)
    self.ros2ovr_dice_twist_pub =rospy.Publisher("/dice_twist", Twist, queue_size=1)
    self.ros2ovr_q2r_twist_pub =rospy.Publisher("/q2r_twist", Twist, queue_size=1)

    #vars
    self.right_hand_pose = PoseStamped()
    self.right_hand_twist = Twist()
    self.right_hand_inputs = OVR2ROSInputs()

    self.left_hand_pose = PoseStamped()
    self.left_hand_twist = Twist()
    self.left_hand_inputs = OVR2ROSInputs()

    # CSV file paths
    self.csv_files = {
        'q2r_left_hand_pose': os.path.join(os.path.expanduser('~'), 'q2r_left_hand_pose.csv'),
        'q2r_right_hand_pose': os.path.join(os.path.expanduser('~'), 'q2r_right_hand_pose.csv'),
        'q2r_left_hand_inputs': os.path.join(os.path.expanduser('~'), 'q2r_left_hand_inputs.csv'),
        'q2r_right_hand_inputs': os.path.join(os.path.expanduser('~'), 'q2r_right_hand_inputs.csv')
    }

    # Initialize CSV files with headers if they don't exist
    self.init_csv_files()

  def init_csv_files(self):
    # Initialize pose CSV files
    for pose_file in ['q2r_left_hand_pose', 'q2r_right_hand_pose']:
        if not os.path.exists(self.csv_files[pose_file]):
            with open(self.csv_files[pose_file], 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
    
    # Initialize inputs CSV files
    for inputs_file in ['q2r_left_hand_inputs', 'q2r_right_hand_inputs']:
        if not os.path.exists(self.csv_files[inputs_file]):
            with open(self.csv_files[inputs_file], 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['button_upper', 'button_lower', 'thumb_stick_horizontal', 'thumb_stick_vertical', 'press_index', 'press_middle'])

  def write_pose_to_csv(self, filename, pose_data):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        writer.writerow([
            pose_data.pose.position.x,
            pose_data.pose.position.y,
            pose_data.pose.position.z,
            pose_data.pose.orientation.x,
            pose_data.pose.orientation.y,
            pose_data.pose.orientation.z,
            pose_data.pose.orientation.w
        ])

  def write_inputs_to_csv(self, filename, inputs_data):
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['button_upper', 'button_lower', 'thumb_stick_horizontal', 'thumb_stick_vertical', 'press_index', 'press_middle'])
        writer.writerow([
            inputs_data.button_upper,
            inputs_data.button_lower,
            inputs_data.thumb_stick_horizontal,
            inputs_data.thumb_stick_vertical,
            inputs_data.press_index,
            inputs_data.press_middle
        ])

  def ovr2ros_right_hand_pose_callback(self, data):    
    self.right_hand_pose = data
    self.write_pose_to_csv(self.csv_files['q2r_right_hand_pose'], data)

  def ovr2ros_right_hand_twist_callback(self, data):    
    self.right_hand_twist = data

  def ovr2ros_right_hand_inputs_callback(self, data):    
    self.right_hand_inputs = data
    self.write_inputs_to_csv(self.csv_files['q2r_right_hand_inputs'], data)

    #if the lower button is pressed send the twist back to the quest to move the q2r ; 0 otherwise
    q2r_twist=Twist()
    if self.right_hand_inputs.button_lower:
      q2r_twist=self.right_hand_twist
    self.ros2ovr_q2r_twist_pub.publish(q2r_twist)

    

    #send the triggers as frequency and amplitude of vibration back to the quest
    right_hand_haptic_feedback = OVR2ROSHapticFeedback()
    right_hand_haptic_feedback.frequency = self.right_hand_inputs.press_index
    right_hand_haptic_feedback.amplitude = self.right_hand_inputs.press_middle
    self.ros2ovr_right_hand_haptic_feedback_pub.publish(right_hand_haptic_feedback)



  def ovr2ros_left_hand_pose_callback(self, data):    
    self.left_hand_pose = data
    self.write_pose_to_csv(self.csv_files['q2r_left_hand_pose'], data)

  def ovr2ros_left_hand_twist_callback(self, data):    
    self.left_hand_twist = data

  def ovr2ros_left_hand_inputs_callback(self, data):    
    self.left_hand_inputs = data
    self.write_inputs_to_csv(self.csv_files['q2r_left_hand_inputs'], data)


    #if the lower button is pressed send the twist back to the quest to move the dice ; 0 otherwise
    dice_twist=Twist()
    if self.left_hand_inputs.button_lower:
      dice_twist=self.left_hand_twist
    self.ros2ovr_dice_twist_pub.publish(dice_twist)

    

    #send the triggers as frequency and amplitude of vibration back to the quest
    left_hand_haptic_feedback = OVR2ROSHapticFeedback()
    left_hand_haptic_feedback.frequency = self.left_hand_inputs.press_index
    left_hand_haptic_feedback.amplitude = self.left_hand_inputs.press_middle
    self.ros2ovr_left_hand_haptic_feedback_pub.publish(left_hand_haptic_feedback)





def main(args):
  rospy.init_node('quest2rosdemo', anonymous=True)


  r2q = ros2quest()  

  r = rospy.Rate(1000) #1000hz
  #print_commands()
  while not rospy.is_shutdown():
    r.sleep()


if __name__ == '__main__':
    main(sys.argv)