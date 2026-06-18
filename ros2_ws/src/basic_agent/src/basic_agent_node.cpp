#include "rclcpp/rclcpp.hpp"
#include "adas_msgs/msg/scenario_msg.hpp"
#include "adas_msgs/msg/manoeuvre_msg.hpp"
#include "basic_agent/mpc_controller.hpp"

static constexpr int    MPC_N         = 60;    // 3 s horizon — enough to plan a full stop
static constexpr double MPC_DT        = 0.05;
static constexpr double MPC_Q1        = 10.0;
static constexpr double MPC_Q2        = 50.0;
static constexpr double MPC_RA        = 1.0;
static constexpr double MPC_RJ        = 0.1;
static constexpr double MPC_AMIN      = -23.46;
static constexpr double MPC_AMAX      =   8.07;
static constexpr double MPC_JMIN      = -8.0;  // was -2 — allows reaching max decel in ~1 s
static constexpr double MPC_JMAX      =  8.0;
static constexpr double STOP_HORIZON  = 100.0; // was 60 — start braking early enough

class BasicAgentNode : public rclcpp::Node {
public:
  BasicAgentNode()
  : Node("basic_agent"),
    mpc_(MPC_N, MPC_DT, MPC_Q1, MPC_Q2, MPC_RA, MPC_RJ,
         MPC_AMIN, MPC_AMAX, MPC_JMIN, MPC_JMAX)
  {
    subscriber_ = this->create_subscription<adas_msgs::msg::ScenarioMsg>(
      "/scenario", 10,
      std::bind(&BasicAgentNode::scenario_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<adas_msgs::msg::ManoeuvreMsg>("/manoeuvre", 10);
  }

private:
  rclcpp::Subscription<adas_msgs::msg::ScenarioMsg>::SharedPtr subscriber_;
  rclcpp::Publisher<adas_msgs::msg::ManoeuvreMsg>::SharedPtr   publisher_;
  MPCController mpc_;

  void scenario_callback(const adas_msgs::msg::ScenarioMsg::SharedPtr msg)
  {
    auto manoeuvre = adas_msgs::msg::ManoeuvreMsg();
    manoeuvre.cycle_number = msg->cycle_number;
    manoeuvre.status       = msg->status;

    double s0    = msg->trf_light_dist;
    double v0    = msg->v_lgt_fild;
    double a0    = std::clamp(msg->a_lgt_fild, MPC_AMIN, MPC_AMAX);
    double v_ref = msg->requested_cruising_speed;
    bool stopping = false;

    if (msg->nr_trf_lights > 0 && s0 > 0.0 && s0 < STOP_HORIZON) {
      int state = msg->trf_light_curr_state;
      if (state == 2 || state == 3) {
        stopping = true;
      } else if (state == 1) {
        // green: only stop if the car clearly cannot clear before it turns red
        double t_reach = s0 / std::max(v0, v_ref);
        double t_red   = msg->trf_light_first_time_to_change;
        stopping = (t_reach > t_red + 2.0);  // 2 s buffer — avoids stopping on late-green
      }
    }

    double acc_cmd = mpc_.compute(s0, v0, a0, v_ref, s0, stopping);
    acc_cmd = std::clamp(acc_cmd, MPC_AMIN, MPC_AMAX);

    RCLCPP_INFO(this->get_logger(),
      "s0=%.1f v0=%.2f a0=%.2f state=%d stopping=%d -> acc=%.3f",
      s0, v0, a0, msg->trf_light_curr_state, (int)stopping, acc_cmd);

    manoeuvre.requested_acc          = acc_cmd;
    manoeuvre.requested_steer_whl_ag = 0.0;

    publisher_->publish(manoeuvre);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicAgentNode>());
  rclcpp::shutdown();
  return 0;
}
