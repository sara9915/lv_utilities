#include <rclcpp/rclcpp.hpp>
#include "lv_utilities/utilities.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto tf_listner_node = rclcpp::Node::make_shared("tf_listner_node");

    bool getTransform_ = false;
    geometry_msgs::msg::TransformStamped transform;


    while (!getTransform_)
        getTransform_ = lv::getTransform(tf_listner_node, "push_extension", "base_link", transform);

    if (getTransform_)
        lv::print_geometry_transform(transform.transform);

    rclcpp::shutdown();
    return 0;
}