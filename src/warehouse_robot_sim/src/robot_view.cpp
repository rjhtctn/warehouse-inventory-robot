#include <csignal>
#include <cstdio>
#include <dlfcn.h>
#include <memory>
#include <stdexcept>
#include <vector>

#include <QApplication>
#include <QDockWidget>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_client_abstraction_iface.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/visualizer_app.hpp>

namespace
{
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }

class ViewRosClient : public rviz_common::ros_integration::RosClientAbstractionIface
{
public:
  rviz_common::ros_integration::RosNodeAbstractionIface::WeakPtr init(
    int argc, char ** argv, const std::string & name, bool) override
  {
    rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    node_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(name);
    return node_;
  }

  bool ok() override { return rclcpp::ok(); }

  void shutdown() override
  {
    // RViz displays must be destroyed while their ROS context is still valid.
  }

private:
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node_;
};
}  // namespace

int main(int argc, char ** argv)
{
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  std::vector<char *> qt_arguments;
  for (auto & argument : arguments) {
    qt_arguments.push_back(argument.data());
  }
  int qt_argc = static_cast<int>(qt_arguments.size());
  qt_arguments.push_back(nullptr);
  auto close_library = [](void * library) { dlclose(library); };
  std::unique_ptr<void, decltype(close_library)> imu_library(nullptr, close_library);
  int result = 1;
  {
    QApplication application(qt_argc, qt_arguments.data());
    {
      rviz_common::VisualizerApp view(std::make_unique<ViewRosClient>());
      view.setApp(&application);
      if (view.init(argc, argv)) {
        // Pending TF callbacks retain plugin code beyond the display's lifetime.
        imu_library.reset(dlopen("librviz_imu_plugin.so", RTLD_NOW | RTLD_NOLOAD));
        if (!imu_library) { throw std::runtime_error(dlerror()); }
        std::fputs("RViz initialized\n", stderr);
        QTimer timer;
        QObject::connect(&timer, &QTimer::timeout, [&application]() {
          if (stopping) { application.exit(0); }
        });
        timer.start(50);
        result = application.exec();
        for (auto * widget : QApplication::topLevelWidgets()) {
          auto * frame = qobject_cast<rviz_common::VisualizationFrame *>(widget);
          if (frame) {
            // Panel destruction must not mutate RViz's panel list while it is cleared.
            for (auto * dock : frame->findChildren<QDockWidget *>()) {
              QObject::disconnect(dock, nullptr, frame, nullptr);
            }
            frame->getManager()->stopUpdate();
            frame->getManager()->removeAllDisplays();
            auto * panel = frame->getManager()->getRenderPanel();
            panel->setViewController(nullptr);
            // Window events can outlive the manager during RViz teardown.
            panel->initialize(nullptr, false);
            break;
          }
        }
      }
    }
    rclcpp::shutdown();
  }
  return result;
}
