#!/usr/bin/env python

import rospy
from time import sleep

def wait_for_service(service, timeout=None):
    rospy.loginfo(f"Waiting for service {service}")
    try:
        rospy.wait_for_service(service, timeout=timeout)
        rospy.loginfo(f"Service {service} is now available")
    except rospy.ROSException:
        rospy.logwarn(f"Timed out waiting for service {service}")

if __name__ == "__main__":
    rospy.init_node('wait_for_service_node')
    wait_for_service('/robot/controller_manager/list_controllers', timeout=10)

