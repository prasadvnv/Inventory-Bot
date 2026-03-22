#!/usr/bin/env python
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_tray():
    rospy.init_node('move_tray', anonymous=True)
    pub = rospy.Publisher('/tray_position_controller/command', JointTrajectory, queue_size=10)
    traj = JointTrajectory()
    traj.joint_names = ['joint_tray']
    point = JointTrajectoryPoint()
    point.positions = [0.3]  # desired position
    point.time_from_start = rospy.Duration(2)  # duration to reach the desired position
    traj.points.append(point)
    pub.publish(traj)
    rospy.sleep(2)  # keep the node running for the duration of the trajectory

if __name__ == '__main__':
    try:
        move_tray()
    except rospy.ROSInterruptException:
        pass

