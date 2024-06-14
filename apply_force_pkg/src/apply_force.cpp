#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>

void applyForceInXDirection(const std::string& body_name, double force_magnitude) {
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
    gazebo_msgs::ApplyBodyWrench srv;

    srv.request.body_name = body_name;
    srv.request.reference_frame = body_name;

    geometry_msgs::Wrench wrench;
    wrench.force.x = force_magnitude;
    wrench.force.y = 0;
    wrench.force.z = 0;
    srv.request.wrench = wrench;

    srv.request.start_time = ros::Time(0);
    srv.request.duration = ros::Duration(0.1);

    if (client.call(srv)) {
        ROS_INFO("Successfully applied force: %f N in X direction", force_magnitude);
    } else {
        ROS_ERROR("Failed to call service apply_body_wrench");
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "apply_varying_force_in_x");
    ros::NodeHandle n;

    std::string body_name = "qm::ft_sensor";  // Replace with your sensor link name
    double max_force = 55.0;  // Maximum force magnitude in Newtons
    double min_force = -35.0; // Minimum force magnitude in Newtons
    double force_step = 5;  // Step size for each iteration
    double current_force = min_force;
    bool increasing = true;

    ros::Rate rate(10);  // 10 Hz update rate

    while (ros::ok()) {
        applyForceInXDirection(body_name, current_force);

        if (increasing) {
            current_force += force_step;
            if (current_force >= max_force) {
                current_force = max_force;
                increasing = false;
            }
        } else {
            current_force -= force_step;
            if (current_force <= min_force) {
                current_force = min_force;
                increasing = true;
            }
        }

        rate.sleep();
    }

    return 0;
}
