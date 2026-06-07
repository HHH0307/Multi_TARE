#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace tare_planner
{
class BoundaryFusionNode : public rclcpp::Node
{
public:
  BoundaryFusionNode()
  : rclcpp::Node("boundary_fusion_node")
  {
    declare_parameter<int>("robot_num", 3);
    declare_parameter<std::string>("global_boundary_topic", "/global_coverage_boundary");
    declare_parameter<std::string>("global_boundary_marker_topic", "/global_coverage_boundary_marker");
    declare_parameter<std::string>("global_boundary_frame_id", "map");
    declare_parameter<double>("boundary_sample_step", 0.25);
    declare_parameter<double>("boundary_voxel_size", 0.25);

    robot_num_ = get_parameter("robot_num").as_int();
    global_boundary_topic_ = get_parameter("global_boundary_topic").as_string();
    global_boundary_marker_topic_ = get_parameter("global_boundary_marker_topic").as_string();
    global_boundary_frame_id_ = get_parameter("global_boundary_frame_id").as_string();
    boundary_sample_step_ = get_parameter("boundary_sample_step").as_double();
    boundary_voxel_size_ = get_parameter("boundary_voxel_size").as_double();

    global_boundary_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>(global_boundary_topic_, 2);
    global_boundary_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(global_boundary_marker_topic_, 2);

    for (int robot_index = 1; robot_index <= robot_num_; ++robot_index)
    {
      const std::string topic = "/robot_" + std::to_string(robot_index) + "/sensor_coverage_planner/explored_boundary";
      boundary_subscribers_.push_back(create_subscription<geometry_msgs::msg::PolygonStamped>(
          topic, 10,
          [this, robot_index](const geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {
            OnBoundary(robot_index, *msg);
          }));
    }
  }

private:
  void OnBoundary(int robot_index, const geometry_msgs::msg::PolygonStamped& boundary_msg)
  {
    if (boundary_msg.polygon.points.empty())
    {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(boundary_mutex_);
      latest_boundaries_[robot_index] = boundary_msg.polygon;
    }

    PublishMergedBoundary();
  }

  static void AppendSampledPolygon(const geometry_msgs::msg::Polygon& polygon,
                                   double sample_step,
                                   std::vector<geometry_msgs::msg::Point32>& sampled_points)
  {
    if (polygon.points.empty())
    {
      return;
    }

    const std::size_t point_count = polygon.points.size();
    for (std::size_t i = 0; i < point_count; ++i)
    {
      const auto& start = polygon.points[i];
      const auto& end = polygon.points[(i + 1) % point_count];
      const double dx = static_cast<double>(end.x - start.x);
      const double dy = static_cast<double>(end.y - start.y);
      const double distance = std::hypot(dx, dy);
      const int sample_count = std::max(1, static_cast<int>(std::ceil(distance / std::max(0.01, sample_step))));

      for (int sample_index = 0; sample_index <= sample_count; ++sample_index)
      {
        const double ratio = static_cast<double>(sample_index) / static_cast<double>(sample_count);
        geometry_msgs::msg::Point32 point;
        point.x = static_cast<float>(start.x + ratio * dx);
        point.y = static_cast<float>(start.y + ratio * dy);
        point.z = 0.0F;
        sampled_points.push_back(point);
      }
    }
  }

  geometry_msgs::msg::Polygon BuildMergedPolygon() const
  {
    std::vector<geometry_msgs::msg::Point32> sampled_points;
    {
      std::lock_guard<std::mutex> lock(boundary_mutex_);
      for (const auto& entry : latest_boundaries_)
      {
        AppendSampledPolygon(entry.second, boundary_sample_step_, sampled_points);
      }
    }

    geometry_msgs::msg::Polygon merged_polygon;
    if (sampled_points.size() < 3)
    {
      return merged_polygon;
    }

    float min_x = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    for (const auto& point : sampled_points)
    {
      min_x = std::min(min_x, point.x);
      max_x = std::max(max_x, point.x);
      min_y = std::min(min_y, point.y);
      max_y = std::max(max_y, point.y);
    }

    const float margin = std::max(0.5f, static_cast<float>(boundary_voxel_size_ * 2.0));
    min_x -= margin;
    max_x += margin;
    min_y -= margin;
    max_y += margin;

    const float voxel = static_cast<float>(std::max(0.01, boundary_voxel_size_));
    const int cols = static_cast<int>(std::ceil((max_x - min_x) / voxel));
    const int rows = static_cast<int>(std::ceil((max_y - min_y) / voxel));
    const int max_dim = 4096;
    if (cols <= 0 || rows <= 0 || cols > max_dim || rows > max_dim)
    {
      return merged_polygon;
    }

    cv::Mat occupancy = cv::Mat::zeros(rows, cols, CV_32FC1);
    for (const auto& point : sampled_points)
    {
      const int col = static_cast<int>((point.x - min_x) / (max_x - min_x + 1e-6f) * (cols - 1));
      const int row = static_cast<int>((max_y - point.y) / (max_y - min_y + 1e-6f) * (rows - 1));
      if (row >= 0 && row < rows && col >= 0 && col < cols)
      {
        occupancy.at<float>(row, col) += 1.0f;
      }
    }

    cv::Mat binary;
    cv::threshold(occupancy, binary, 0.5, 255, cv::THRESH_BINARY);
    binary.convertTo(binary, CV_8UC1);

    const cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, close_kernel);

    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
    {
      return merged_polygon;
    }

    std::size_t best_contour_index = 0;
    double best_contour_area = 0.0;
    for (std::size_t i = 0; i < contours.size(); ++i)
    {
      const double area = std::abs(cv::contourArea(contours[i]));
      if (area > best_contour_area)
      {
        best_contour_area = area;
        best_contour_index = i;
      }
    }

    std::vector<cv::Point> approximated_contour;
    const double epsilon = std::max(1.0, static_cast<double>(voxel));
    cv::approxPolyDP(contours[best_contour_index], approximated_contour, epsilon, true);
    if (approximated_contour.size() < 3)
    {
      return merged_polygon;
    }

    merged_polygon.points.reserve(approximated_contour.size());
    for (const auto& contour_point : approximated_contour)
    {
      geometry_msgs::msg::Point32 point;
      point.x = min_x + (static_cast<float>(contour_point.x) + 0.5f) * voxel;
      point.y = max_y - (static_cast<float>(contour_point.y) + 0.5f) * voxel;
      point.z = 0.0F;
      merged_polygon.points.push_back(point);
    }

    return merged_polygon;
  }

  void PublishMergedBoundary()
  {
    const geometry_msgs::msg::Polygon merged_polygon = BuildMergedPolygon();
    if (merged_polygon.points.size() < 3)
    {
      return;
    }

    geometry_msgs::msg::PolygonStamped polygon_msg;
    polygon_msg.header.stamp = now();
    polygon_msg.header.frame_id = global_boundary_frame_id_;
    polygon_msg.polygon = merged_polygon;
    global_boundary_pub_->publish(polygon_msg);

    visualization_msgs::msg::Marker marker_msg;
    marker_msg.header = polygon_msg.header;
    marker_msg.ns = "global_coverage_boundary";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker_msg.action = visualization_msgs::msg::Marker::ADD;
    marker_msg.scale.x = 0.08;
    marker_msg.color.r = 0.2F;
    marker_msg.color.g = 1.0F;
    marker_msg.color.b = 0.35F;
    marker_msg.color.a = 0.95F;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.points.reserve(merged_polygon.points.size() + 1);
    for (const auto& point : merged_polygon.points)
    {
      geometry_msgs::msg::Point marker_point;
      marker_point.x = point.x;
      marker_point.y = point.y;
      marker_point.z = 0.0;
      marker_msg.points.push_back(marker_point);
    }
    geometry_msgs::msg::Point closing_point;
    closing_point.x = merged_polygon.points.front().x;
    closing_point.y = merged_polygon.points.front().y;
    closing_point.z = 0.0;
    marker_msg.points.push_back(closing_point);
    global_boundary_marker_pub_->publish(marker_msg);
  }

  int robot_num_{0};
  std::string global_boundary_topic_;
  std::string global_boundary_marker_topic_;
  std::string global_boundary_frame_id_;
  double boundary_sample_step_{0.25};
  double boundary_voxel_size_{0.25};

  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr global_boundary_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr global_boundary_marker_pub_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr> boundary_subscribers_;

  mutable std::mutex boundary_mutex_;
  std::map<int, geometry_msgs::msg::Polygon> latest_boundaries_;
};
}  // namespace tare_planner

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tare_planner::BoundaryFusionNode>());
  rclcpp::shutdown();
  return 0;
}