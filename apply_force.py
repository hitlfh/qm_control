import rospy
import math
from geometry_msgs.msg import Wrench, Vector3
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
import time

def apply_force_at_angle(body_name, force_magnitude, angle):
    """
    Apply a force to a specified body at a given angle around the Z-axis.
    :param body_name: The name of the body to apply the force to.
    :param force_magnitude: The magnitude of the force to apply.
    :param angle: The angle in radians at which to apply the force, measured from the Y-axis.
    """
    rospy.wait_for_service('/gazebo/apply_body_wrench')
    try:
        apply_body_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        wrench = Wrench()
        wrench.force.x = 0
        wrench.force.y = force_magnitude * math.cos(angle)
        wrench.force.z = force_magnitude * math.sin(angle)
        request = ApplyBodyWrenchRequest()
        request.body_name = body_name
        request.reference_frame = body_name
        request.wrench = wrench
        request.start_time = rospy.Time(0)
        request.duration = rospy.Duration(0.1)
        apply_body_wrench(request)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def main():
    rospy.init_node('apply_varying_force')
    body_name = "qm::ft_sensor"  # Replace with your sensor link name
    force_magnitude = 30.0  # Force magnitude in Newtons
    start_angle = -math.pi / 2  # Start from -90 degrees (Y-axis negative direction)
    end_angle = math.pi / 2  # End at 90 degrees (Y-axis positive direction)
    num_steps = 100  # Number of steps to transition the force direction

    step_angle = (end_angle - start_angle) / num_steps
    rate = rospy.Rate(10)  # 10 Hz update rate

    for step in range(num_steps + 1):
        current_angle = start_angle + step * step_angle
        apply_force_at_angle(body_name, force_magnitude, current_angle)
        rate.sleep()

if __name__ == "__main__":
    main()
