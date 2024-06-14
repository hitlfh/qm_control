#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetLinkState

def get_link_pose(link_name):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp = get_link_state(link_name, 'world')
        return resp.link_state.pose
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('get_links_poses')

    link_names = [
        'qm::LF_hip',
        'qm::LH_hip',
        'qm::RF_hip',
        # Add more links as needed
    ]

    for link_name in link_names:
        pose = get_link_pose(link_name)
        rospy.loginfo("Pose of %s: %s" % (link_name, pose))
