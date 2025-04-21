#include "../include/csv.hpp"
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>

using namespace std::chrono_literals;

struct Point {
    double x;
    double y;
    double z;
};

class MarkerPublisher : public rclcpp::Node {
  public:
    MarkerPublisher(const std::string &csv_path)
        : Node("marker_publisher"), csv_path_(csv_path) {
        publisher_dn_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/drone_ports", 10);
        publisher_pm_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/control_points", 10);
        timer_ = this->create_wall_timer(
            1s, std::bind(&MarkerPublisher::publishMarkers, this));
        loadCsv();
    }

  private:
    void loadCsv() {
        try {
            io::CSVReader<4> reader(csv_path_);
            reader.read_header(io::ignore_extra_column, "index", "x", "y", "z");
            std::string index;
            double x, y, z;
            while (reader.read_row(index, x, y, z)) {
                data_[index] = Point{x, y, z};
            }
            RCLCPP_INFO(get_logger(), "Loaded %zu entries from CSV",
                        data_.size());
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "Failed to load CSV: %s", e.what());
        }
    }

    void publishMarkers() {
        rclcpp::Time now = this->now();
        int id = 0;

        for (const auto &[key, point] : data_) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = now;
            marker.ns = key[0] == 'd' ? "drone_ports" : "control_points";
            marker.id = id++;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = point.x;
            marker.pose.position.y = point.y;
            marker.pose.position.z = point.z;
            marker.pose.orientation.w = 1.0;

            if (key[0] == 'd') {
                marker.type = visualization_msgs::msg::Marker::CUBE;
                marker.scale.x = 10.0;
                marker.scale.y = 10.0;
                marker.scale.z = 5.0;
                marker.color.r = 0.0f;
                marker.color.g = 0.0f;
                marker.color.b = 1.0f;
                marker.color.a = 0.8f;
                publisher_dn_->publish(marker);
            } else if (key[0] == 'p') {
                marker.type = visualization_msgs::msg::Marker::CYLINDER;
                marker.scale.x = 10.0;
                marker.scale.y = 10.0;
                marker.scale.z = 5.0;
                marker.color.r = 0.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 0.8f;
                publisher_pm_->publish(marker);
            }
        }
    }

  private:
    std::string csv_path_;
    std::unordered_map<std::string, Point> data_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_dn_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_pm_;
    rclcpp::TimerBase::SharedPtr timer_;
};
