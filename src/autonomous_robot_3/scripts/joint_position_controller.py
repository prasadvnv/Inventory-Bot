#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

def move_joint():
    # Initialize the ROS node
    rospy.init_node('control_prismatic_joint', anonymous=True)

    # Create a publisher for the joint position controller
    joint_pub = rospy.Publisher('/joint_tray_controller/command', Float64, queue_size=10)

    # Set the rate at which to publish
    rate = rospy.Rate(10) # 10 Hz

    # Define the target positions
    target_positions = [0.0, 0.1, 0.2, 0.3, 0.35]  # Adjust these values as needed

    while not rospy.is_shutdown():
        for position in target_positions:
            # Create a Float64 message with the target position
            joint_position = Float64()
            joint_position.data = position

            # Publish the target position
            rospy.loginfo(f"Moving joint to position: {position}")
            joint_pub.publish(joint_position)

            # Wait for a while before moving to the next position
            rospy.sleep(2)  # Adjust the sleep duration as needed

        # Stop publishing after one cycle
        break

if __name__ == '__main__':
    try:
        move_joint()
    except rospy.ROSInterruptException:
        pass

