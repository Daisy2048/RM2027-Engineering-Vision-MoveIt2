#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class VisionTfBridge : public rclcpp::Node
{
public:
    VisionTfBridge()
        : Node("vision_tf_bridge")
    {
        // 1. 创建 TF 缓冲区和监听器
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(
            this->get_clock());

        tf_listener_ =
            std::make_shared<tf2_ros::TransformListener>(
                *tf_buffer_);

        // 2. 订阅 BIT 右侧 PnP 位姿
        pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/pnp/pose_1",
                10,
                [this](
                    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    this->pose_callback(msg);
                });

        // 3. 发布转换后的参考坐标系位姿
        pose_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/vision/reference_pose_base",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Vision TF bridge started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Input: /pnp/pose_1 (mm, camera)");

        RCLCPP_INFO(
            this->get_logger(),
            "Output: /vision/reference_pose_base (m, base_link)");
    }

private:
    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 4. 检查输入坐标系
        if (msg->header.frame_id != "camera")
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unexpected input frame: %s",
                msg->header.frame_id.c_str());
            return;
        }

        const auto &p = msg->pose.position;
        const auto &q = msg->pose.orientation;

        // 5. 检查位置和姿态是否为有限数值
        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z) ||
            !std::isfinite(q.x) ||
            !std::isfinite(q.y) ||
            !std::isfinite(q.z) ||
            !std::isfinite(q.w))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid pose: NaN or Inf detected.");
            return;
        }

        // 6. 检查四元数是否合法
        const double q_norm = std::sqrt(
            q.x * q.x +
            q.y * q.y +
            q.z * q.z +
            q.w * q.w);

        if (std::abs(q_norm - 1.0) > 0.01)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid quaternion norm: %.6f",
                q_norm);
            return;
        }

        // 7. 复制消息并转换单位：mm -> m
        geometry_msgs::msg::PoseStamped pose_m = *msg;

        pose_m.pose.position.x /= 1000.0;
        pose_m.pose.position.y /= 1000.0;
        pose_m.pose.position.z /= 1000.0;

        // BIT 的 camera 使用 OpenCV 光学坐标轴约定。
        // 内部使用独立帧名，避免与 BIT 原有 TF 冲突。
        pose_m.header.frame_id = "bit_camera_optical";

        // 8. 查询相机相对于机械臂基座的变换
        try
        {
            // 第一版使用最新可用 TF，后续实机需处理时间同步。
            auto transform = tf_buffer_->lookupTransform(
                "base_link",
                "bit_camera_optical",
                tf2::TimePointZero);

            // 9. 把完整位姿转换到 base_link
            geometry_msgs::msg::PoseStamped pose_base;

            tf2::doTransform(
                pose_m,
                pose_base,
                transform);

            pose_base.header.frame_id = "base_link";

            // 保留视觉消息的原始时间戳
            pose_base.header.stamp = msg->header.stamp;

            // 10. 发布结果
            pose_pub_->publish(pose_base);

            RCLCPP_INFO(
                this->get_logger(),
                "Converted pose: x=%.4f y=%.4f z=%.4f",
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "TF unavailable: %s",
                ex.what());
        }
    }

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<
        tf2_ros::TransformListener> tf_listener_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VisionTfBridge>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}