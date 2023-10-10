#include "rclcpp/rclcpp.hpp"
#include "depth_optimization_interfaces/srv/depth_optimize.hpp"

class DepthOptimizeClient : public rclcpp::Node
{
public:
  DepthOptimizeClient() : Node("depth_optimize_client_node")
  {
    client_ = create_client<depth_optimization_interfaces::srv::DepthOptimize>("depth_optimize");

    if (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_ERROR(get_logger(), "Service not available. Exiting.");
      rclcpp::shutdown();
      return;
    }

    request_ = std::make_shared<depth_optimization_interfaces::srv::DepthOptimize::Request>();

    // Set request values from your data source
    // For example, read values from a configuration file, another data source, or user input.

    // Example: Set request values manually
    request_->estimated_pose.pose.position.x = 1.0;
    request_->estimated_pose.pose.position.y = 2.0;
    request_->estimated_pose.pose.position.z = 3.0;

    send_request();
  }

private:
  void send_request()
  {
    auto result = client_->async_send_request(request_);

    if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(get_logger(), "Service response:");
      RCLCPP_INFO(get_logger(), "Refined Pose: x=%f, y=%f, z=%f",
                  result.get()->refined_pose.pose.position.x,
                  result.get()->refined_pose.pose.position.y,
                  result.get()->refined_pose.pose.position.z);
      RCLCPP_INFO(get_logger(), "Scaled Cuboid Dimension: %f, %f, %f",
                  result.get()->scaled_cuboid_dimension[0],
                  result.get()->scaled_cuboid_dimension[1],
                  result.get()->scaled_cuboid_dimension[2]);
      RCLCPP_INFO(get_logger(), "Scale Object: %f", result.get()->scale_obj);
      RCLCPP_INFO(get_logger(), "Success: %d", result.get()->success);
    }
    else
    {
      RCLCPP_ERROR(get_logger(), "Unable to get a response from the service");
    }

    rclcpp::shutdown();
  }

  rclcpp::Client<depth_optimization_interfaces::srv::DepthOptimize>::SharedPtr client_;
  depth_optimization_interfaces::srv::DepthOptimize::Request::SharedPtr request_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthOptimizeClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
