#include "k_model.h"
#include "ros/console.h"
#include <visualization_msgs/Marker.h>
#include <cmath>

namespace k_model {

k_model_publisher::k_model_publisher() :
    nh(ros::NodeHandle()),
    nh_private(ros::NodeHandle("~")) {
    nh_private.param<double>("workspace_size", workspace_size, 1.5); // Square workspace (1.5 x 1.5 m)

    teleop_sub = nh.subscribe("/cmd_vel", 1, &k_model_publisher::teleop_callback, this);
    k_model_pub = nh.advertise<geometry_msgs::Twist>("cmd_cable_lengths", 1); // Publish cable lengths
    visualization_pub = nh.advertise<visualization_msgs::Marker>("platform_visualization", 1); // Visualization
}

void k_model_publisher::teleop_callback(const geometry_msgs::Twist &vel) {
    float x = vel.linear.x;  // Target X position
    float y = vel.linear.y;  // Target Y position

    // Check boundaries
    if (x < 0 || x > workspace_size || y < 0 || y > workspace_size) {
        ROS_WARN("Target coordinates are out of bounds.");
        return;
    }

    // Calculate cable lengths based on geometry
    float L_TL = calculate_cable_length(x - rect_width / 2, y + rect_height / 2, 0, workspace_size); // Top-left
    float L_TR = calculate_cable_length(x + rect_width / 2, y + rect_height / 2, workspace_size, workspace_size); // Top-right

    // Publish cable lengths to Arduino
    geometry_msgs::Twist msg;
    msg.linear.x = L_TL;
    msg.linear.y = L_TR;
    k_model_pub.publish(msg);

    // Publish platform visualization
    publish_visualization(x, y);
}

// Helper function to calculate cable lengths
float k_model_publisher::calculate_cable_length(float x, float y, float anchor_x, float anchor_y) {
    return sqrt(pow(x - anchor_x, 2) + pow(y - anchor_y, 2));
}

// Function to publish platform visualization in RViz
void k_model_publisher::publish_visualization(float x, float y) {
    visualization_msgs::Marker platform_marker;
    platform_marker.header.frame_id = "world";
    platform_marker.header.stamp = ros::Time::now();
    platform_marker.ns = "platform";
    platform_marker.id = 0;
    platform_marker.type = visualization_msgs::Marker::CUBE;
    platform_marker.action = visualization_msgs::Marker::ADD;

    // Set platform position
    platform_marker.pose.position.x = x;
    platform_marker.pose.position.y = y;
    platform_marker.pose.position.z = rect_height / 2.0; // Center of platform
    platform_marker.scale.x = rect_width;
    platform_marker.scale.y = rect_width;
    platform_marker.scale.z = rect_height;
    platform_marker.color.r = 0.0;
    platform_marker.color.g = 0.7;
    platform_marker.color.b = 1.0;
    platform_marker.color.a = 0.8;

    visualization_pub.publish(platform_marker);
}

} // namespace k_model
