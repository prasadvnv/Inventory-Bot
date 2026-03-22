#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty

def move_to_pose(final_pose):
    group.set_named_target(final)  # Use the provided final_pose argument
    plan = group.go(wait=True)  # Plan and wait for the execution to complete
    group.stop()  # Ensure the robot stops smoothly after reaching the target
    group.clear_pose_targets()  # Clear the pose targets after execution

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_interface', anonymous=True)

    global group
    group = moveit_commander.MoveGroupCommander("tray")  # Specify the correct planning group name

    # Ensure that RViz is running and can visualize the robot
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    rospy.sleep(2)

    print("Moving to pose 1")
    move_to_pose("idle")  # Move to the pose named "final" (assuming it's defined in MoveIt config)

    rospy.sleep(2)

    print("Moving to pose 2")
    move_to_pose("final")  # Move to the pose named "idle" (assuming it's defined in MoveIt config)

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()

