#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <visualization_msgs/Marker.h>

namespace k_model {

class k_model_publisher {
public:
    k_model_publisher();

private:
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber teleop_sub;  // Subscriber for target positions
    ros::Publisher k_model_pub;  // Publisher for cable lengths
    ros::Publisher visualization_pub; // Publisher for visualization

    // Workspace parameters
    double workspace_size;  // Size of the square workspace (1.5m)
    double rect_width = 0.3;  // Width of the platform (meters)
    double rect_height = 0.1; // Height of the platform (meters)

    // Callback to process teleop commands
    void teleop_callback(const geometry_msgs::Twist &vel);

    // Helper function to calculate cable lengths
    float calculate_cable_length(float x, float y, float anchor_x, float anchor_y);

    // Function to publish visualization markers
    void publish_visualization(float x, float y);
};

} // namespace k_model
