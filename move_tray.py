#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import SetModelConfiguration
import sys
import argparse

def move_tray(target_position):
    rospy.init_node('move_tray')

    # Wait for the service to be available
    rospy.wait_for_service('/gazebo/set_model_configuration')
    try:
        set_model_configuration = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        
        # Configuration details
        model_name = 'autonomous_robot_3'
        urdf_param_name = 'robot_description'
        joint_names = ['joint_tray']
        
        # Initial joint position
        initial_position = 0.007
        steps = 100
        increment = (target_position - initial_position) / steps
        current_position = initial_position
        
        for i in range(steps):
            # Update the current position
            current_position += increment
            
            # Set joint positions
            joint_positions = [current_position]
            
            # Call the service
            set_model_configuration(model_name, urdf_param_name, joint_names, joint_positions)
            
            # Wait for a short duration to create a gradual movement
            rospy.sleep(0.1)
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Move the tray to a target z position.')
    parser.add_argument('target_position', type=float, help='Target z position for the tray joint')
    args = parser.parse_args()

    try:
        move_tray(args.target_position)
    except rospy.ROSInterruptException:
        pass

