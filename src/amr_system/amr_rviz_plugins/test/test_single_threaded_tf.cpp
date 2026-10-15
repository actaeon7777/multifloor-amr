#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "amr_rviz_plugins/single_threaded_tf.hpp"
#include "rviz_common/ros_integration/ros_node_abstraction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/message_filter.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class SingleThreadedTFTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
  void SetUp() override
  {
    node = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("tf_plugin_test");
    transformer.initialize(node, node->get_raw_node()->get_clock());
    executor.add_node(node->get_raw_node());
    broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(node->get_raw_node());
    // Allow DDS discovery of the same-process publisher and listener.
    for (int i = 0; i < 30; ++i) {executor.spin_some(); std::this_thread::sleep_for(10ms);}
  }
  bool spinUntil(const std::function<bool()> & predicate)
  {
    const auto deadline = std::chrono::steady_clock::now() + 2s;
    while (!predicate() && std::chrono::steady_clock::now() < deadline) {
      executor.spin_some();
      std::this_thread::sleep_for(1ms);
    }
    return predicate();
  }
  geometry_msgs::msg::TransformStamped tf(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped value;
    value.header.stamp = stamp;
    value.header.frame_id = "map";
    value.child_frame_id = "laser";
    value.transform.translation.x = 2.;
    value.transform.rotation.w = 1.;
    return value;
  }
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstraction> node;
  amr_rviz_plugins::SingleThreadedTF transformer;
  rclcpp::executors::SingleThreadedExecutor executor;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;
};

TEST_F(SingleThreadedTFTest, MissingTFIsDeliveredOnlyWhenTheDisplayExecutorSpins)
{
  using Filter = tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan,
      rviz_common::transformation::FrameTransformer>;
  Filter filter(transformer, "map", 10, node->get_raw_node());
  const auto gui_thread = std::this_thread::get_id();
  int delivered = 0;
  filter.registerCallback([&](sensor_msgs::msg::LaserScan::ConstSharedPtr) {
    EXPECT_EQ(std::this_thread::get_id(), gui_thread);
    ++delivered;
  });
  // Exercise the exact waitForTransform + concurrent DDS receipt pattern of the
  // frozen RViz, including the first map TF becoming available after a scan.
  for (int i = 0; i < 200; ++i) {
    const auto stamp = node->get_raw_node()->now();
    auto scan = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan->header.frame_id = "laser";
    scan->header.stamp = stamp;
    filter.add(scan);
    EXPECT_EQ(delivered, i);
    broadcaster->sendTransform(tf(stamp));
    if (i == 0) {
      std::this_thread::sleep_for(30ms);
      EXPECT_FALSE(transformer.canTransform("map", "laser", tf2_ros::fromRclcpp(stamp), nullptr));
      EXPECT_EQ(delivered, 0);
    }
    ASSERT_TRUE(spinUntil([&] {return delivered == i + 1;}));
  }
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "laser";
  pose.pose.orientation.w = 1.;
  auto result = transformer.transform(pose, "map");
  EXPECT_EQ(result.header.frame_id, "map");
  EXPECT_DOUBLE_EQ(result.pose.position.x, 2.);
}

TEST_F(SingleThreadedTFTest, MissingTransformFailsImmediatelyWithoutBlockingUI)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "absent";
  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(transformer.transform(pose, "map"),
    rviz_common::transformation::FrameTransformerException);
  EXPECT_LT(std::chrono::steady_clock::now() - start, 100ms);
}

TEST_F(SingleThreadedTFTest, PendingTransformTimesOutWhileExecutorRemainsResponsive)
{
  bool timed_out = false;
  auto future = transformer.waitForTransform("map", "missing", tf2::TimePointZero, 20ms,
    [&](const tf2_ros::TransformStampedFuture & result) {
      EXPECT_THROW(result.get(), tf2::TimeoutException);
      timed_out = true;
    });
  ASSERT_TRUE(spinUntil([&] {return timed_out;}));
  EXPECT_EQ(future.wait_for(0ms), std::future_status::ready);
}
