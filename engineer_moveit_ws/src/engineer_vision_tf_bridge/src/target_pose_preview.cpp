#include <cmath>
#include <memory>
#include <vector>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

class TargetPosePreview : public rclcpp::Node
{
public:
    TargetPosePreview()
        : Node("target_pose_preview")
    {
        // 1. 声明目标偏移参数（单位：米）
        // 默认全部为 0，避免未经配置就采用实机目标偏移。
        this->declare_parameter<std::vector<double>>(
            "offset_xyz_m",
            {0.0, 0.0, 0.0});

        offset_ = this->get_parameter("offset_xyz_m")
                      .as_double_array();

        if (offset_.size() != 3)
        {
            throw std::runtime_error(
                "offset_xyz_m must contain exactly 3 values");
        }

        for (double value : offset_)
        {
            if (!std::isfinite(value))
            {
                throw std::runtime_error(
                    "offset_xyz_m contains NaN or Inf");
            }
        }

        // 2. 订阅第二个接口输出的参考位姿
        pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/vision/reference_pose_base",
                10,
                [this](
                    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                {
                    this->pose_callback(msg);
                });

        // 3. 发布候选末端目标位姿
        // 注意：这是预览话题，不直接连接 MoveIt。
        pose_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/vision/candidate_tip_pose_base",
                10);

        RCLCPP_INFO(
            this->get_logger(),
            "Target pose preview started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Offset: x=%.4f y=%.4f z=%.4f m",
            offset_[0], offset_[1], offset_[2]);
    }

private:
    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // 4. 检查参考位姿坐标系
        if (msg->header.frame_id != "base_link")
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid reference frame: %s",
                msg->header.frame_id.c_str());
            return;
        }

        const auto &p = msg->pose.position;
        const auto &r = msg->pose.orientation;

        // 5. 检查数据是否为有限数值
        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z) ||
            !std::isfinite(r.x) ||
            !std::isfinite(r.y) ||
            !std::isfinite(r.z) ||
            !std::isfinite(r.w))
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid reference pose: NaN or Inf");
            return;
        }

        // 6. 构造参考坐标系的四元数
        tf2::Quaternion q_ref(
            r.x, r.y, r.z, r.w);

        const double norm = q_ref.length();

        if (std::abs(norm - 1.0) > 0.01)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid quaternion norm: %.6f",
                norm);
            return;
        }

        q_ref.normalize();

        // 7. 在兑换装置自己的坐标系下定义偏移
        tf2::Vector3 offset_local(
            offset_[0],
            offset_[1],
            offset_[2]);

        // 8. 把局部偏移旋转到 base_link 坐标系
        tf2::Vector3 offset_base =
            tf2::quatRotate(q_ref, offset_local);

        // 9. 计算候选目标位置
        geometry_msgs::msg::PoseStamped target = *msg;

        target.pose.position.x += offset_base.x();
        target.pose.position.y += offset_base.y();
        target.pose.position.z += offset_base.z();

        // 10. 第一版仅保留参考坐标系姿态作为演示假设。
        // 真正的 bat_center 姿态偏移尚未确定。
        target.pose.orientation.x = q_ref.x();
        target.pose.orientation.y = q_ref.y();
        target.pose.orientation.z = q_ref.z();
        target.pose.orientation.w = q_ref.w();

        // 11. 发布候选目标位姿
        pose_pub_->publish(target);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            3000,
            "Candidate tip pose: x=%.4f y=%.4f z=%.4f",
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z);
    }

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

    std::vector<double> offset_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TargetPosePreview>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}