#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <algorithm>
#include <vector>
#include <string>

// Real-hardware teleop node, replacing both sim_control_node.cpp (for real
// hardware use) and arm_control_node_v2.py entirely.

// JOYSTICK CONTROLLER MAPPING TABLE ( Update here also!!! after changing the controller or its mapping in /joy_arm )

// --- AXIS MAPPING (Joint Movements) ---
// | Axis Index | Joint Target            | ROS Joint Name           | Motion Type       |
// |------------|-------------------------|--------------------------|-------------------|
// | axes[0]    | Base Joint              | base_link_joint          | Continuous (+/-)  |
// | axes[1]    | Shoulder Joint          | shoulder_link_joint      | Continuous (+/-)  |
// | axes[4]    | Elbow Joint             | elbow_link_joint         | Continuous (+/-)  |
// | axes[7]    | Wrist Pitch Joint       | wrist_link_joint         | Continuous (+/-)  |
// | axes[6]    | Palm Roll Joint         | palm_link_joint          | Continuous (+/-)  |
// | axes[3]    | Left/Right Finger Joints| left/right_finger_joint  | Discrete Open/Close|




// --- BUTTON MAPPING (State & Speed Control) ---
// | Button Index | Action / Feature                                                  |
// |--------------|-------------------------------------------------------------------|
// | button[0]    | Toggle Autonomous / Manual mode (Publish /arm_state)              |
// | button[1]    | Toggle Joint-Space / Position-Space (Publish /input_space)        |
// | button[6]    | Step Down k_level (Decrease Sensitivity / Speed)                  |
// | button[7]    | Step Up k_level (Increase Sensitivity / Speed)                    |

// Highest axis/button index this node reads. Used to bounds-check incoming
// /joy_arm messages before touching them -- see joy_callback. Bump these if
// the mapping table above ever grows to use a higher index.
static constexpr size_t kMinRequiredAxes = 8;    // indices 0..7 used (axes[7] is highest)
static constexpr size_t kMinRequiredButtons = 8; // indices 0..7 used (button[7] is highest)

class RealArmTeleopNode : public rclcpp::Node {
public:
    RealArmTeleopNode() : rclcpp::Node("real_arm_teleop_node") {
        this->declare_parameter<double>("time_from_start_s", 0.15);
        this->declare_parameter<double>("cooldown_s", 0.3);
        time_from_start_s_ = this->get_parameter("time_from_start_s").as_double();
        cooldown_s_ = this->get_parameter("cooldown_s").as_double();

        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy_arm", 10,
            std::bind(&RealArmTeleopNode::joy_callback, this, std::placeholders::_1));

        trajectory_pub = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10);
        arm_state_pub = this->create_publisher<std_msgs::msg::Bool>("/arm_state", 10);
        input_space_pub = this->create_publisher<std_msgs::msg::Bool>("/input_space", 10);

        joint_names = {
            "base_link_joint",
            "shoulder_link_joint",
            "elbow_link_joint",
            "wrist_link_joint",
            "palm_link_joint",
            "left_finger_link_joint",
            "right_finger_link_joint"
        };

        // Start centered at 0 for every joint. CONFIRM this is actually a
        // safe starting pose on the real arm before the first run.
        joint_positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // From rover_arm_raw.xacro's <limit> tags. Update here if the arm's
        // physical limits ever change -- nothing keeps these in sync with
        // the xacro automatically.
        lower_limits = {-1.570795, -1.5708, -3.141593, -1.570795, -3.141593, -1.570795, -1.570795};
        upper_limits = { 1.570795,  0.3927,  3.141593,  1.570795,  3.141593,  0.785398,  0.785398};

        // Step-size tiers -- replaces v2's pwm_array = {127, 180, 255}.
        // Start conservative; widen once you've confirmed real motor
        // response at the smallest tier.
        k_levels = {0.005, 0.01, 0.02};
        k_index = 0;

        last_state_toggle_time = this->now();
        last_space_toggle_time = this->now();

        RCLCPP_INFO(this->get_logger(), "RealArmTeleopNode initialized in MANUAL mode. Initial k = %.3f", k_levels[k_index]);
    }

private:
    static constexpr int autonomous_button_idx = 0;   // Button 'A' -- matches v2
    static constexpr int space_toggle_button_idx = 1; // Button 'B' -- matches v2
    static constexpr int mode_down_button_idx = 6;    // matches v2
    static constexpr int mode_up_button_idx = 7;      // matches v2

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_state_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr input_space_pub;

    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;
    std::vector<double> lower_limits;
    std::vector<double> upper_limits;
    std::vector<double> k_levels;
    size_t k_index;

    double time_from_start_s_;
    double cooldown_s_;

    bool arm_state_ = false;     // False = manual, True = autonomous
    bool input_space_ = false;   // published for parity; not acted on here
    rclcpp::Time last_state_toggle_time;
    rclcpp::Time last_space_toggle_time;
    bool prev_mode_up_btn_ = false;
    bool prev_mode_down_btn_ = false;

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
};

void RealArmTeleopNode::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // Guard against malformed/short Joy messages -- msg->axes[...] and
    // msg->buttons[...] below use operator[], which does NOT bounds-check.
    // A joystick driver that publishes fewer axes/buttons than we expect
    // (wrong controller, driver hiccup, bad remap) would otherwise be
    // undefined behavior -- on real hardware that can mean anything from a
    // crash to silently commanding garbage joint positions. Reject the
    // message instead and wait for a well-formed one.
    if (msg->axes.size() < kMinRequiredAxes || msg->buttons.size() < kMinRequiredButtons) {
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Ignoring /joy_arm message: got %zu axes / %zu buttons, need at least %zu / %zu. "
            "Check the joystick mapping table at the top of this file against 'ros2 topic echo /joy_arm'.",
            msg->axes.size(), msg->buttons.size(), kMinRequiredAxes, kMinRequiredButtons);
        return;
    }

    const rclcpp::Time now = this->now();

    // arm_state is published every callback regardless of mode -- this is
    // the single source of truth other nodes read.
    std_msgs::msg::Bool state_msg;
    state_msg.data = arm_state_;
    arm_state_pub->publish(state_msg);

    // AUTONOMOUS TOGGLE (button A), debounced.
    if (msg->buttons[autonomous_button_idx] &&
        (now - last_state_toggle_time).seconds() > cooldown_s_) {
        arm_state_ = !arm_state_;
        last_state_toggle_time = now;

        RCLCPP_INFO(this->get_logger(), "Switched Arm Mode to: %s", 
                    arm_state_ ? "AUTONOMOUS (MoveIt in control)" : "MANUAL (Joystick in control)");
    }

    if (arm_state_) {
        // Autonomous mode: MoveIt owns /arm_controller/joint_trajectory now.
        // Stay completely quiet -- don't compute or publish anything.
        return;
    }

    // INPUT SPACE TOGGLE (button B), debounced. Published for parity/future use.
    if (msg->buttons[space_toggle_button_idx] &&
        (now - last_space_toggle_time).seconds() > cooldown_s_) {
        input_space_ = !input_space_;
        last_space_toggle_time = now;

        RCLCPP_INFO(this->get_logger(), "Switched Input Space to: %s", 
                    input_space_ ? "POSITION SPACE (Task Space)" : "JOINT SPACE");
    }
    std_msgs::msg::Bool space_msg;
    space_msg.data = input_space_;
    input_space_pub->publish(space_msg);

    // MODE UP/DOWN (buttons 7/6) -- step-size tier select, edge-triggered
    if (msg->buttons[mode_down_button_idx]) {
        if (!prev_mode_down_btn_) {
            if (k_index > 0) {
                k_index--;
                RCLCPP_INFO(this->get_logger(), "Decreased k_level step-size tier to index %zu: k = %.3f", k_index, k_levels[k_index]);
            } else {
                RCLCPP_WARN(this->get_logger(), "Already at minimum k_level tier (k = %.3f)", k_levels[k_index]);
            }
            prev_mode_down_btn_ = true;
        }
    } else {
        prev_mode_down_btn_ = false;
    }

    if (msg->buttons[mode_up_button_idx]) {
        if (!prev_mode_up_btn_) {
            if (k_index < k_levels.size() - 1) {
                k_index++;
                RCLCPP_INFO(this->get_logger(), "Increased k_level step-size tier to index %zu: k = %.3f", k_index, k_levels[k_index]);
            } else {
                RCLCPP_WARN(this->get_logger(), "Already at maximum k_level tier (k = %.3f)", k_levels[k_index]);
            }
            prev_mode_up_btn_ = true;
        }
    } else {
        prev_mode_up_btn_ = false;
    }
    const double k = k_levels[k_index];

    // Joint-space motion -- same axis mapping as sim_control_node.cpp
    joint_positions[0] += k * msg->axes[0];     // Base
    joint_positions[1] += k * msg->axes[1];     // Shoulder
    joint_positions[2] += k * msg->axes[4];     // Elbow
    joint_positions[3] += k * msg->axes[7];     // Wrist (pitch)
    joint_positions[4] += k * msg->axes[6];     // Palm (roll)
    if (msg->axes[3] > 0) {
        joint_positions[5] += k;
        joint_positions[6] -= k;
    } else if (msg->axes[3] < 0) {
        joint_positions[5] -= k;
        joint_positions[6] += k;
    }

    // Clamp to real joint limits. Also warn (throttled, so a joint held
    // against its limit doesn't spam the console) whenever a clamp actually
    // changes the commanded value -- that's the signal that you just tried
    // to drive a joint past where it's mechanically allowed to go.
    for (size_t i = 0; i < joint_positions.size(); ++i) {
        const double pre_clamp = joint_positions[i];
        joint_positions[i] = std::clamp(joint_positions[i], lower_limits[i], upper_limits[i]);
        if (joint_positions[i] != pre_clamp) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Joint '%s' hit its limit and was clamped to %.3f rad.",
                joint_names[i].c_str(), joint_positions[i]);
        }
    }

    auto traj = trajectory_msgs::msg::JointTrajectory();
    traj.header.stamp = now;
    traj.joint_names = joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = joint_positions;
    point.time_from_start = rclcpp::Duration::from_seconds(time_from_start_s_);
    traj.points.push_back(point);
    trajectory_pub->publish(traj);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RealArmTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}