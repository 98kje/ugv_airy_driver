// Converts rslidar_sdk output (POINT_TYPE XYZIRT) into the Autoware sensing input format.
//   pointcloud: x y z intensity(float) ring(uint16) timestamp(float64, absolute sec)
//            -> autoware::point_types::PointXYZIRCAEDT
//   imu:       acceleration in g, IMU chip axes
//            -> acceleration in m/s^2, rotated into the lidar axes, frame = imu_frame_id
//              (give imu_frame_id the same orientation as the lidar frame in the TF tree)

#include <autoware/point_types/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ugv_airy_driver
{
using autoware::point_types::PointXYZIRCAEDT;
using sensor_msgs::msg::Imu;
using sensor_msgs::msg::PointCloud2;
using sensor_msgs::msg::PointField;

static_assert(sizeof(PointXYZIRCAEDT) == 32, "unexpected PointXYZIRCAEDT layout");

class AiryConverter : public rclcpp::Node
{
public:
  explicit AiryConverter(const rclcpp::NodeOptions & options) : Node("airy_converter", options)
  {
    return_type_ = static_cast<std::uint8_t>(declare_parameter<int>(
      "return_type", static_cast<int>(autoware::point_types::ReturnType::SINGLE_STRONGEST)));
    imu_frame_id_ = declare_parameter<std::string>("imu_frame_id", "rs_airy/imu_link");
    // Airy IMU reports linear acceleration in g
    accel_scale_ = declare_parameter<double>("accel_scale", 9.80665);
    // IMU chip -> lidar rotation [x, y, z, w]. The Airy reports it in DIFOP (qx..qw, offset 1092, big endian float);
    // default = our unit (2026-10-28), a 180 deg turn about (1, -1, 0): x_lidar = -y_imu, y_lidar = -x_imu,
    // z_lidar = -z_imu. Confirmed on the desk by gravity in two poses and a CCW turn.
    const auto q = declare_parameter<std::vector<double>>(
      "imu_to_lidar_quaternion", {0.7081, -0.7061, -0.0005, 0.0047});
    if (q.size() != 4) {
      throw std::invalid_argument("imu_to_lidar_quaternion must be [x, y, z, w]");
    }
    set_rotation(q[0], q[1], q[2], q[3]);

    pointcloud_pub_ = create_publisher<PointCloud2>("output/pointcloud", rclcpp::SensorDataQoS());
    // imu_corrector subscribes reliable, so a best-effort publisher would never reach it
    imu_pub_ = create_publisher<Imu>("output/imu", rclcpp::QoS{10});

    pointcloud_sub_ = create_subscription<PointCloud2>(
      "input/pointcloud", rclcpp::SensorDataQoS(),
      std::bind(&AiryConverter::on_pointcloud, this, std::placeholders::_1));
    imu_sub_ = create_subscription<Imu>(
      "input/imu", rclcpp::SensorDataQoS(),
      std::bind(&AiryConverter::on_imu, this, std::placeholders::_1));
  }

private:
  static bool has_field(const PointCloud2 & msg, const std::string & name)
  {
    return std::any_of(msg.fields.begin(), msg.fields.end(), [&](const PointField & f) {
      return f.name == name;
    });
  }

  void on_pointcloud(const PointCloud2::ConstSharedPtr msg)
  {
    if (!has_field(*msg, "ring") || !has_field(*msg, "timestamp")) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "input has no ring/timestamp field: build rslidar_sdk with POINT_TYPE XYZIRT");
      return;
    }

    auto out = std::make_unique<PointCloud2>();
    out->header = msg->header;
    sensor_msgs::PointCloud2Modifier modifier(*out);
    modifier.setPointCloud2Fields(
      10, "x", 1, PointField::FLOAT32, "y", 1, PointField::FLOAT32, "z", 1, PointField::FLOAT32,
      "intensity", 1, PointField::UINT8, "return_type", 1, PointField::UINT8, "channel", 1,
      PointField::UINT16, "azimuth", 1, PointField::FLOAT32, "elevation", 1, PointField::FLOAT32,
      "distance", 1, PointField::FLOAT32, "time_stamp", 1, PointField::UINT32);
    modifier.resize(static_cast<std::size_t>(msg->width) * msg->height);

    const std::int64_t stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();

    sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> it_i(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<std::uint16_t> it_ring(*msg, "ring");
    sensor_msgs::PointCloud2ConstIterator<double> it_t(*msg, "timestamp");

    auto * dst = reinterpret_cast<PointXYZIRCAEDT *>(out->data.data());
    std::size_t count = 0;
    for (; it_x != it_x.end(); ++it_x, ++it_y, ++it_z, ++it_i, ++it_ring, ++it_t) {
      const float x = *it_x;
      const float y = *it_y;
      const float z = *it_z;
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
        continue;
      }

      PointXYZIRCAEDT & p = dst[count++];
      p.x = x;
      p.y = y;
      p.z = z;
      p.intensity = static_cast<std::uint8_t>(std::clamp(std::lround(*it_i), 0L, 255L));
      p.return_type = return_type_;
      p.channel = *it_ring;
      p.distance = std::sqrt(x * x + y * y + z * z);
      p.azimuth = std::atan2(y, x);
      if (p.azimuth < 0.0F) {
        p.azimuth += static_cast<float>(2.0 * M_PI);
      }
      p.elevation = std::atan2(z, std::hypot(x, y));
      // time_stamp: nanoseconds relative to header.stamp
      const std::int64_t dt_ns = std::llround(*it_t * 1e9) - stamp_ns;
      p.time_stamp = dt_ns > 0 ? static_cast<std::uint32_t>(dt_ns) : 0U;
    }

    modifier.resize(count);
    out->is_dense = true;
    pointcloud_pub_->publish(std::move(out));
  }

  void set_rotation(double x, double y, double z, double w)
  {
    const double n = std::sqrt(x * x + y * y + z * z + w * w);
    if (n < 1e-6) {
      throw std::invalid_argument("imu_to_lidar_quaternion has zero norm");
    }
    x /= n;
    y /= n;
    z /= n;
    w /= n;
    rot_ = {{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
             {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
             {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
  }

  void rotate(geometry_msgs::msg::Vector3 & v, double scale) const
  {
    const double x = v.x * scale;
    const double y = v.y * scale;
    const double z = v.z * scale;
    v.x = rot_[0][0] * x + rot_[0][1] * y + rot_[0][2] * z;
    v.y = rot_[1][0] * x + rot_[1][1] * y + rot_[1][2] * z;
    v.z = rot_[2][0] * x + rot_[2][1] * y + rot_[2][2] * z;
  }

  void on_imu(const Imu::ConstSharedPtr msg)
  {
    auto out = std::make_unique<Imu>(*msg);
    out->header.frame_id = imu_frame_id_;
    rotate(out->linear_acceleration, accel_scale_);
    rotate(out->angular_velocity, 1.0);
    out->orientation_covariance[0] = -1.0;  // no orientation estimate
    imu_pub_->publish(std::move(out));
  }

  std::uint8_t return_type_{};
  std::string imu_frame_id_;
  double accel_scale_{};
  std::array<std::array<double, 3>, 3> rot_{};

  rclcpp::Publisher<PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<Imu>::SharedPtr imu_sub_;
};

}  // namespace ugv_airy_driver

RCLCPP_COMPONENTS_REGISTER_NODE(ugv_airy_driver::AiryConverter)
