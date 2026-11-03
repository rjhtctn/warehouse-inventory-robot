#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/ros_gz_bridge.hpp>

namespace
{
volatile std::sig_atomic_t running = 1;
void stop(int) { running = 0; }

class SimulationBridge : public ros_gz_bridge::RosGzBridge
{
public:
  void release()
  {
    heartbeat_timer_->cancel();
    // Break the ownership cycles in older bridge libraries before shutting down ROS.
    handles_.clear();
    services_.clear();
    gz_node_.reset();
  }
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  auto node = std::make_shared<SimulationBridge>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (running && rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  executor.remove_node(node);
  node->release();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
