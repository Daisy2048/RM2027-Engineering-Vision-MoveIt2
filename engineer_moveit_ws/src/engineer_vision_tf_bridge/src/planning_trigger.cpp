#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

class PlanningTrigger : public rclcpp::Node
{
public:
    PlanningTrigger()
        : Node("planning_trigger")
    {
        // 1. 安全开关：默认禁止发布规划目标
        simulation_enabled_ =
            this->declare_parameter<bool>(
                "simulation_enabled", false);

        // 2. 位姿时效性检查参数
        max_pose_age_sec_ =
            this->declare_parameter<double>(
                "max_pose_age_sec", 2.0);

        // 允许的最大未来时间偏差
        max_future_skew_sec_ =
            this->declare_parameter<double>(
                "max_future_skew_sec", 0.5);

        // 3. 软件测试范围，单位：米
        // 这些只是基础输入边界，不代表机械臂实际可达空间。
        min_x_ = this->declare_parameter<double>("min_x_m", -1.0);
        max_x_ = this->declare_parameter<double>("max_x_m",  1.0);

        min_y_ = this->declare_parameter<double>("min_y_m", -1.0);
        max_y_ = this->declare_parameter<double>("max_y_m",  1.0);

        min_z_ = this->declare_parameter<double>("min_z_m", -0.1);
        max_z_ = this->declare_parameter<double>("max_z_m",  1.5);

        if (max_pose_age_sec_ <= 0.0 ||
            max_future_skew_sec_ < 0.0 ||
            min_x_ >= max_x_ ||
            min_y_ >= max_y_ ||
            min_z_ >= max_z_)
        {
            throw std::runtime_error(
                "Invalid planning trigger parameters");
        }

        // 4. 接收候选末端目标位姿
        pose_sub_ =
            this->create_subscription<
                geometry_msgs::msg::PoseStamped>(
                "/vision/candidate_tip_pose_base",
                10,
                std::bind(
                    &PlanningTrigger::pose_callback,
                    this,
                    std::placeholders::_1));

        // 5. 向第一个接口发布规划目标
        target_pub_ =
            this->create_publisher<
                geometry_msgs::msg::PoseStamped>(
                "/vision/target_pose_base",
                10);

        // 6. 创建人工触发服务
        trigger_service_ =
            this->create_service<
                std_srvs::srv::Trigger>(
                "/vision/trigger_planning",
                std::bind(
                    &PlanningTrigger::trigger_callback,
                    this,
                    std::placeholders::_1,
                    std::placeholders::_2));

        RCLCPP_INFO(
            this->get_logger(),
            "Planning trigger started.");

        RCLCPP_INFO(
            this->get_logger(),
            "Simulation enabled: %s",
            simulation_enabled_ ? "true" : "false");

        RCLCPP_INFO(
            this->get_logger(),
            "Waiting for candidate poses.");

        RCLCPP_INFO(
            this->get_logger(),
            "Manual trigger service: /vision/trigger_planning");
    }

private:
    // 7. 检查候选位姿是否合法
    bool is_valid_pose(
        const geometry_msgs::msg::PoseStamped &msg)
    {
        if (msg.header.frame_id != "base_link")
        {
            return false;
        }

        const auto &p = msg.pose.position;
        const auto &q = msg.pose.orientation;

        if (!std::isfinite(p.x) ||
            !std::isfinite(p.y) ||
            !std::isfinite(p.z) ||
            !std::isfinite(q.x) ||
            !std::isfinite(q.y) ||
            !std::isfinite(q.z) ||
            !std::isfinite(q.w))
        {
            return false;
        }

        const double q_norm = std::sqrt(
            q.x * q.x +
            q.y * q.y +
            q.z * q.z +
            q.w * q.w);

        if (std::abs(q_norm - 1.0) > 0.01)
        {
            return false;
        }

        // 基础位置范围检查
        if (p.x < min_x_ || p.x > max_x_ ||
            p.y < min_y_ || p.y > max_y_ ||
            p.z < min_z_ || p.z > max_z_)
        {
            return false;
        }

        return true;
    }

    // 8. 接收并缓存最新候选位姿
    void pose_callback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!is_valid_pose(*msg))
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                3000,
                "Invalid candidate pose rejected.");
            return;
        }

        latest_pose_ = *msg;
        received_at_ = this->now();
    }

    // 9. 人工触发一次规划
    void trigger_callback(
        const std::shared_ptr<
            std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<
            std_srvs::srv::Trigger::Response> response)
    {
        (void)request;

        // 安全开关未开启：拒绝
        if (!simulation_enabled_)
        {
            response->success = false;
            response->message =
                "Simulation planning is disabled.";

            RCLCPP_WARN(
                this->get_logger(),
                "Planning rejected: simulation disabled.");
            return;
        }

        // 尚未收到有效候选位姿：拒绝
        if (!latest_pose_.has_value() ||
            !received_at_.has_value())
        {
            response->success = false;
            response->message =
                "No valid candidate pose received.";
            return;
        }

        // 拒绝零时间戳
        const auto &stamp = latest_pose_->header.stamp;

        if (stamp.sec == 0 && stamp.nanosec == 0)
        {
            response->success = false;
            response->message =
                "Candidate pose has zero timestamp.";
            return;
        }

        const auto now = this->now();

        const rclcpp::Time pose_time(
            stamp, now.get_clock_type());

        const double pose_age =
            (now - pose_time).seconds();

        const double receive_age =
            (now - *received_at_).seconds();

        // 10. 检查消息时间戳和本地接收时间
        if (!std::isfinite(pose_age) ||
            !std::isfinite(receive_age) ||
            pose_age > max_pose_age_sec_ ||
            receive_age > max_pose_age_sec_ ||
            pose_age < -max_future_skew_sec_ ||
            receive_age < -max_future_skew_sec_)
        {
            response->success = false;
            response->message =
                "Candidate pose is stale or has invalid time.";
            return;
        }

        // 11. 检查第一个规划接口是否有订阅者
        if (target_pub_->get_subscription_count() == 0)
        {
            response->success = false;
            response->message =
                "No planner subscriber available.";
            return;
        }

        // 12. 只发布一次目标
        target_pub_->publish(*latest_pose_);

        response->success = true;
        response->message =
            "One planning target published (plan only).";

        const auto &p = latest_pose_->pose.position;

        RCLCPP_INFO(
            this->get_logger(),
            "Planning triggered once: "
            "x=%.4f y=%.4f z=%.4f",
            p.x, p.y, p.z);
    }

    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;

    rclcpp::Service<
        std_srvs::srv::Trigger>::SharedPtr trigger_service_;

    std::optional<
        geometry_msgs::msg::PoseStamped> latest_pose_;

    std::optional<rclcpp::Time> received_at_;

    bool simulation_enabled_;

    double max_pose_age_sec_;
    double max_future_skew_sec_;

    double min_x_;
    double max_x_;

    double min_y_;
    double max_y_;

    double min_z_;
    double max_z_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<PlanningTrigger>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}